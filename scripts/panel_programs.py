"""
Builds one light paint program per car panel in RoboDK (ABB IRB 5500-27 on
an IRBT 7004 track, robot on the car's -X side).

Each panel = a few straight zig-zag passes with 3 targets per pass
(start just off the panel, middle, end). For every target:
  - the surface point and normal come from a ray cast onto the car mesh,
  - the gun points along the normal (TCP = surface point, standoff is in the TCP),
  - the rail is set so the robot base sits roughly opposite the gun,
  - the arm IK is solved with J5 kept on one side of 0 (no wrist singularity).
Program: MoveJ approach -> MoveL along each pass, MoveJ between passes -> MoveL retract.

Usage (RoboDK open with the station loaded):
  D:/Softwares/RoboDK/Python-Embedded/python.exe scripts/panel_programs.py Door Rear_Fender Roof Trunk
"""
import sys
import numpy as np
from robodk.robolink import *
from robodk.robomath import *

ROBOT, TOOL, FRAME = 'ABB IRB 5500-27', 'Generic Paint Sprayer', 'car_frame'
SECTIONS = ['car_paint_%d' % i for i in range(1, 5)]
RAIL_Y0, RAIL_MAX = 2018.0, 3700.0     # car_frame Y of the robot base is RAIL_Y0 - d7
OVERRUN = 100                          # mm past a free panel edge
BACKOFF = 300                          # mm approach / retract along the spray axis
PAINT_SPEED, TRAVEL_SPEED = 500, 1000  # mm/s
J5_MARGIN = 15                         # deg away from the wrist singularity
SPINS = [0, 15, -15, 30, -30, 45, -45, 60, -60, 90, -90, 135, -135, 180]

# Panel definitions in car_frame (mm). Car length is along Y, front at Y = -2244.
#   kind 'top' : passes along Y at fixed X, ray cast downwards
#   kind 'side': passes along Y at fixed Z, ray cast from -X
#   passes: (fixed coordinate, y_start, y_end, overrun_at_start, overrun_at_end)
PANELS = {
    'Door': ('side', [(620, -760, 1150, False, False),
                      (480, -760, 1150, False, False),
                      (340, -760, 1150, False, False),
                      (200, -760, 1150, False, False)]),
    'Rear_Fender': ('side', [(620, 1250, 2450, False, True),
                             (480, 1250, 2450, False, True),
                             (300, 1800, 2450, False, True)]),
    'Roof': ('top', [(-700, -40, 1240, False, False),
                     (-475, -40, 1240, False, False),
                     (-250, -40, 1240, False, False),
                     (-25, -40, 1240, False, False)]),
    'Trunk': ('top', [(-850, 1480, 2540, False, True),
                      (-625, 1480, 2540, False, True),
                      (-400, 1480, 2540, False, True),
                      (-175, 1480, 2540, False, True),
                      (50, 1480, 2540, False, True)]),
}


def load_mesh(rdk, frame):
    to_frame = invH(frame.PoseAbs())
    tris = []
    for name in SECTIONS:
        obj = rdk.Item(name, ITEM_TYPE_OBJECT)
        h = to_frame * obj.PoseAbs()
        rot = np.array([[h[i, j] for j in range(3)] for i in range(3)])
        trans = np.array([h[i, 3] for i in range(3)])
        for f in range(10):
            pts, _ = obj.GetPoints(FEATURE_OBJECT_MESH, f)
            if not pts:
                break
            tris.append((np.array(pts)[:, :3] @ rot.T + trans).reshape(-1, 3, 3))
    return np.concatenate(tris)


def raycast(tris, origin, direction):
    """First hit (point, normal facing the ray) of a ray on the mesh, or None."""
    v0, e1, e2 = tris[:, 0], tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]
    h = np.cross(direction, e2)
    a = (e1 * h).sum(1)
    ok = np.abs(a) > 1e-9
    f = np.zeros_like(a)
    f[ok] = 1 / a[ok]
    s = origin - v0
    u = f * (s * h).sum(1)
    q = np.cross(s, e1)
    v = f * (q @ direction)
    t = f * (e2 * q).sum(1)
    hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 0)
    if not hit.any():
        return None
    k = np.where(hit)[0][np.argmin(t[hit])]
    n = np.cross(e1[k], e2[k])
    n /= np.linalg.norm(n)
    return origin + t[k] * direction, (-n if n @ direction > 0 else n)


def surface(tris, kind, c, y):
    """Surface point on the outer skin at (c, y); c is X for top panels, Z for sides."""
    if kind == 'top':
        r = raycast(tris, np.array([c, y, 3000.0]), np.array([0, 0, -1.0]))
        return r if r and r[1][2] > 0.3 else None
    r = raycast(tris, np.array([-3000.0, y, c]), np.array([1.0, 0, 0]))
    return r if r and r[1][0] < -0.3 and r[0][0] < -500 else None


def pass_points(tris, kind, c, y0, y1, ext0, ext1):
    """Start / middle / end of one pass, trimmed to where the panel exists."""
    ys = [y for y in np.arange(y0, y1 + 1, 20) if surface(tris, kind, c, y)]
    if len(ys) < 2:
        return []
    a, b = ys[0], ys[-1]
    pts = [surface(tris, kind, c, y) for y in (a, (a + b) / 2, b)]
    if ext0:
        pts[0] = (pts[0][0] - [0, OVERRUN, 0], pts[0][1])
    if ext1:
        pts[2] = (pts[2][0] + [0, OVERRUN, 0], pts[2][1])
    return pts


def tcp_pose(p, n, along):
    z = -n
    x = along - (along @ z) * z
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return Mat([[x[0], y[0], z[0], p[0]], [x[1], y[1], z[1], p[1]],
                [x[2], y[2], z[2], p[2]], [0, 0, 0, 1]])


class Solver:
    def __init__(self, robot, tool, frame):
        self.robot = robot
        self.to_base = invH(robot.Parent().PoseAbs()) * frame.PoseAbs()
        self.tinv = invH(tool.PoseTool())

    def ik(self, h, hint):
        j = self.robot.SolveIK(self.to_base * h * self.tinv, hint).list()
        return j if len(j) >= 7 else None

    def point(self, h, prev, y, sign):
        """Spin about the spray axis + rail offset that keeps J5 on `sign` side."""
        rail = RAIL_Y0 - y
        for off in (0, -300, 300, -600, 600):
            best = None
            for s in SPINS:
                hs = h * rotz(s * pi / 180)
                j = self.ik(hs, list(prev[:6]) + [float(np.clip(rail + off, 0, RAIL_MAX))])
                if j is None or sign * j[4] < J5_MARGIN:
                    continue
                cost = sum(abs(a - b) for a, b in zip(j[:6], prev[:6]))
                if best is None or cost < best[0]:
                    best = (cost, hs, j)
            if best:
                return best[1], best[2]
        return None

    def backoff(self, h, j):
        for d in (BACKOFF, 200, 120):
            hb = h * transl(0, 0, -d)
            for off in (0, -300, 300, -600, 600):
                jb = self.ik(hb, list(j[:6]) + [float(np.clip(j[6] + off, 0, RAIL_MAX))])
                if jb:
                    return hb, jb
        return None


def build_panel(rdk, name, tris, solver, robot, tool, frame):
    kind, passes = PANELS[name]
    prev = robot.Joints().list()
    solved = []                          # list of passes, each a list of (pose, joints)
    for i, (c0, y0, y1, e0, e1) in enumerate(passes):
        done = None
        # if out of reach: shift the pass sideways, drop the overrun, or trim its ends inward
        tries = ([(nd, e0, e1, 0, 0) for nd in (0, 50, -50, 100, -100)] +
                 [(0, False, False, a, b) for a, b in ((0, 0), (0, 100), (0, 200), (0, 300), (100, 0), (200, 0))])
        for nudge, ee0, ee1, trim0, trim1 in tries:
            pts = pass_points(tris, kind, c0 + nudge, y0 + trim0, y1 - trim1, ee0, ee1)
            along = np.array([0, 1.0, 0])
            if i % 2:                    # zig-zag
                pts.reverse()
                along = -along
            for sign in (-1, 1):         # one wrist side for the whole pass
                out, pv = [], prev
                for p, n in pts:
                    r = solver.point(tcp_pose(p, n, along), pv, p[1], sign)
                    if r is None:
                        break
                    out.append(r)
                    pv = r[1]
                if pts and len(out) == len(pts):
                    done = out
                    break
            if done:
                if nudge or (ee0, ee1) != (e0, e1) or trim0 or trim1:
                    print('  %s pass %d adjusted to stay in reach: shift %+d mm, overrun %s, ends trimmed %d/%d mm'
                          % (name, i + 1, nudge, 'kept' if (ee0, ee1) == (e0, e1) else 'dropped', trim0, trim1))
                break
        if done:
            solved.append(done)
            prev = done[-1][1]
        else:
            print('  %s pass %d: not reachable, skipped' % (name, i + 1))

    def target(tname, h, j):
        t = rdk.Item(tname, ITEM_TYPE_TARGET)
        if not (t.Valid() and t.Name() == tname):
            t = rdk.AddTarget(tname, frame, robot)
        t.setPose(h)
        t.setJoints(j)
        return t

    old = rdk.Item('Paint_' + name, ITEM_TYPE_PROGRAM)
    if old.Valid() and old.Name() == 'Paint_' + name:
        old.Delete()
    prog = rdk.AddProgram('Paint_' + name, robot)
    prog.setPoseFrame(frame)
    prog.setPoseTool(tool)
    prog.setSpeed(TRAVEL_SPEED)
    app = solver.backoff(*solved[0][0])
    prog.MoveJ(target('App_' + name, *app))
    prog.setSpeed(PAINT_SPEED)
    robot.setPoseFrame(frame)            # MoveL_Test works in the robot's active frame/tool
    robot.setPoseTool(tool)
    k = 0
    last = app[1]
    for i, pas in enumerate(solved):
        for m, (h, j) in enumerate(pas):
            k += 1
            t = target('%s %d' % (name, k), h, j)
            step = m == 0 and i > 0      # move to the next pass
            line_ok = last is not None and robot.MoveL_Test(last, h) == 0
            (prog.MoveL if (not step and line_ok) else prog.MoveJ)(t)
            last = j
    # retract straight out from the last point; at the car's ends that can be out
    # of reach, then retract from the previous point of the pass with a joint move
    ret = solver.backoff(*solved[-1][-1])
    if ret:
        prog.MoveL(target('Retract_' + name, *ret))
    else:
        for h, j in reversed(solved[-1][:-1]):
            ret = solver.backoff(h, j)
            if ret:
                prog.MoveJ(target('Retract_' + name, *ret))
                break
    _, time_s, dist, ok, msg = prog.Update(COLLISION_ON)
    print('Paint_%s: %d targets, %.1f s, valid %.2f, %s' % (name, k, time_s, ok, msg.replace('<br>', ' ')))
    robot.setJoints((ret or solved[-1][-1])[1])  # next panel starts from here


MAIN_ORDER = ['Hood', 'Fender', 'Door', 'Rear_Fender', 'Trunk', 'Roof']
LIFT_OUT, LIFT_UP, SAFE_Z = 400, 400, 1500   # mm: lift away from the car (-X) and up, minimum height
NEUTRAL_X, NEUTRAL_Z = -1400, 1700            # mm: neutral pose beside and above the car


def build_main(rdk, robot, tool, frame):
    """Main_Paint: calls the panel programs in order with controlled transitions.
    Retract -> Lift_out (same gun orientation, up and away, IK seeded from the
    retract, so the arm just slides back) -> Neutral (gun straight down, high
    beside the car, IK chosen to split any wrist flip evenly) -> Lift_in above
    the next panel (seeded from its approach) -> next panel's approach."""
    solver = Solver(robot, tool, frame)

    def lifted(target_name):
        t = rdk.Item(target_name, ITEM_TYPE_TARGET)
        h, j = t.Pose(), t.Joints().list()
        p = h.Pos()
        up = max(LIFT_UP, SAFE_Z - p[2])
        toward_mid = -np.sign(p[1])
        for out, dz, dy in ((LIFT_OUT, up, 0), (LIFT_OUT / 2, up, 0), (0, up, 0), (0, up / 2, 0),
                            (LIFT_OUT, up, 300), (LIFT_OUT, up, 600), (0, up, 600), (0, up / 2, 900)):
            hl = transl(-out, toward_mid * dy, dz) * h
            for off in (0, -300, 300, -600, 600):
                jl = solver.ik(hl, list(j[:6]) + [float(np.clip(j[6] + off, 0, RAIL_MAX))])
                if jl:
                    return jl
        print('  lift from %s not reachable, staying at the target' % target_name)
        return j

    def dist(a, b):
        return sum(abs(x - y) for x, y in zip(a[:6], b[:6]))

    def neutral_options(y, hints):
        """IK solutions with the gun straight down at (NEUTRAL_X, y, NEUTRAL_Z), over
        spins about the vertical gun axis and several seeds."""
        rail = float(np.clip(RAIL_Y0 - y, 0, RAIL_MAX))
        sols = []
        for spin in range(0, 360, 30):
            h = transl(NEUTRAL_X, y, NEUTRAL_Z) * rotx(pi) * rotz(spin * pi / 180)
            for hint in hints:
                j = solver.ik(h, list(hint[:6]) + [rail])
                if j and abs(j[4]) >= 10:
                    sols.append(j)
        return sols

    def transition(j_out, j_in, y):
        """Neutral pose(s) between two panels. Small wrist change: none. Otherwise the
        gun-down pose with the least total travel; if the wrist side (sign of J5) must
        change, a second gun-down pose on the new side, so the flip happens in place."""
        if max(abs(a - b) for a, b in zip(j_out[3:6], j_in[3:6])) <= 90:
            return []
        sols = neutral_options(y, (j_out, j_in))
        if not sols:
            return []
        a = min(sols, key=lambda j: dist(j_out, j) + dist(j, j_in))
        if np.sign(a[4]) == np.sign(j_in[4]):
            return [a]
        same_side = [j for j in sols if np.sign(j[4]) == np.sign(j_in[4])]
        if not same_side:
            return [a]
        b = min(same_side, key=lambda j: dist(a, j) + dist(j, j_in))
        return [a, b]

    def jt(name, j):
        t = rdk.Item(name, ITEM_TYPE_TARGET)
        if not (t.Valid() and t.Name() == name):
            t = rdk.AddTarget(name, frame, robot)
        t.setAsJointTarget()
        t.setJoints(j)
        return t

    for old_name in [i.Name() for i in rdk.ItemList(ITEM_TYPE_TARGET)]:   # clear earlier versions
        if old_name.startswith(('Safe_', 'Lift_', 'Neutral_')):
            rdk.Item(old_name, ITEM_TYPE_TARGET).Delete()
    old = rdk.Item('Main_Paint', ITEM_TYPE_PROGRAM)
    if old.Valid() and old.Name() == 'Main_Paint':
        old.Delete()
    main_prog = rdk.AddProgram('Main_Paint', robot)
    main_prog.setSpeed(TRAVEL_SPEED)
    prev = None
    for name in MAIN_ORDER:
        j_in = lifted('App_' + name)
        if prev is None:
            main_prog.MoveJ(jt('Lift_in_' + name, j_in))
        else:
            j_out = lifted('Retract_' + prev)
            y = (rdk.Item('Retract_' + prev, ITEM_TYPE_TARGET).Pose().Pos()[1] +
                 rdk.Item('App_' + name, ITEM_TYPE_TARGET).Pose().Pos()[1]) / 2
            main_prog.MoveJ(jt('Lift_out_' + prev, j_out))
            for k, j_mid in enumerate(transition(j_out, j_in, y)):
                main_prog.MoveJ(jt('Neutral_%s_%s' % ('AB'[k], name), j_mid))
            main_prog.MoveJ(jt('Lift_in_' + name, j_in))
        main_prog.RunInstruction('Paint_' + name, INSTRUCTION_CALL_PROGRAM)
        prev = name
    main_prog.MoveJ(jt('Lift_out_' + prev, lifted('Retract_' + prev)))
    _, time_s, dist, ok, msg = main_prog.Update(COLLISION_ON)
    print('Main_Paint: %.1f s, valid %.2f, %s' % (time_s, ok, msg.replace('<br>', ' ')))


def main(names):
    rdk = Robolink()
    robot = rdk.Item(ROBOT, ITEM_TYPE_ROBOT)
    tool = rdk.Item(TOOL, ITEM_TYPE_TOOL)
    frame = rdk.Item(FRAME, ITEM_TYPE_FRAME)
    tris = load_mesh(rdk, frame)
    solver = Solver(robot, tool, frame)
    for name in names:
        if name == 'Main':
            build_main(rdk, robot, tool, frame)
        else:
            build_panel(rdk, name, tris, solver, robot, tool, frame)


if __name__ == '__main__':
    main(sys.argv[1:] or list(PANELS))
