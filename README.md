# Paint Booth Simulation

Robotic painting cell for a car body, built in RoboDK for the Yuktii AI Labs robotics simulation assessment.

- **Name:** Mohammed Salah Chogle
- **Software used:** RoboDK 6.1 (simulation + offline programming), Python RoboDK API, Fusion 360 (booth and conveyor models), Blender (car model cleanup)
- **Robot selected:** ABB IRB 5500-27 paint robot on an ABB IRBT 7004 linear track (7th axis)

![Paint simulation](docs%20n%20images/paint_simulation.gif)

*Part of `Main_Paint`, rendered from RoboDK's HTML export.*

## What the simulation does

A car body rides in on a conveyor, stops in the booth, gets painted panel by panel, and leaves through the exit.

```
Cell_Cycle
  Robot_Park_Start   robot parks clear of the car path
  Conveyor_Reset     car to the entry (conveyor d = 0)
  Conveyor_In        car drives to the paint stop (d = 4500 mm)
  Main_Paint         paints hood, front fender, door, rear fender, trunk, roof
  Robot_Park_End     robot parks clear again
  Conveyor_Out       car leaves the booth (d = 13248 mm)
```

All paint paths are taught with the car at the paint stop, in `car_frame`, which rides on the conveyor.

## Cell layout

- Booth based on the reference layout: entry zone, paint booth, exit zone, support rooms. The car moves through on a roller conveyor with a skid (cart).
- One IRB 5500-27 on the left side of the car, mounted on a 4 m IRBT 7004 track that runs along the car. The track lets a 2.2 m reach arm cover a 4.8 m car.
- A single fixed arm can't reach the far side of the car without hitting it (checked with a cross-section reach study), so a real line would use a second, mirrored robot on the other side. This sim programs one side.

## Paint paths

Each panel is its own program (`Paint_Hood`, `Paint_Fender`, `Paint_Door`, `Paint_Rear_Fender`, `Paint_Trunk`, `Paint_Roof`):

- Zig-zag passes, 3 targets per pass (start just off the panel edge, middle, end).
- Gun kept normal to the surface. The tool centre point sits 250 mm in front of the nozzle, so a target on the surface gives the right standoff.
- Spray width 300 mm, pass spacing 150 to 225 mm, paint speed 500 mm/s.
- The track moves with the gun, so the arm keeps a similar pose along each pass.
- Wrist joint J5 is kept at least 15 degrees away from 0 inside a pass to avoid the wrist singularity. Where needed, the gun is spun about its own spray axis to get there.
- Linear moves along the strokes, joint moves for the steps between passes and where a straight line wasn't reachable.
- The spray gun switches on only along the strokes (`SprayOn(1)` / `SprayOn(0)` per pass).

| Program | Targets | Notes |
|---|---|---|
| Paint_Hood | 15 | 5 passes, front to rear |
| Paint_Fender | 8 | 2 passes plus a stroke around the wheel arch |
| Paint_Door | 12 | 4 horizontal passes |
| Paint_Rear_Fender | 9 | 2 full passes plus a low pass behind the arch |
| Paint_Trunk | 15 | 5 passes, starts at the corner next to the rear fender |
| Paint_Roof | 12 | 4 passes, starts at the corner next to the trunk |

`Main_Paint` takes about 180 s for one side.

## Collision checking

All programs pass RoboDK's path check with collision checking on ("No problems found for this path, No collisions detected"), and the parked robot was checked against the car over the full conveyor travel. Two pairs are excluded on purpose:

- gun vs robot wrist link 6: the gun is bolted to the flange, so they always touch
- gun vs car: the downloaded gun model has a solid decorative spray cone that "hits" the car near curved panels

Robot links vs car, booth and conveyor stay checked.

## Spray simulation

`scripts/SprayOn.py` is the RoboDK SprayOn macro, adapted:

- spray cone built from the standoff (250 mm) and spray width (300 mm)
- the car is split into 4 sections (`car_paint_1` to `car_paint_4`) because RoboDK's particle projection stopped registering hits on one object spanning the whole 4.8 m car
- the guns are created once per run and then only switched on and off

Rough transfer efficiency from the spray stats: about 70% of particles land on the body. That's a simulation estimate, not a measured number.

About the car model: it's a free model I picked from Sketchfab, not engineering CAD. It has a lot of geometry issues (messy mesh, very uneven triangle density), so the spray doesn't stick properly on a few parts of the body even when the gun is in the right place. That's the mesh, not the paths. A clean CAD body would fix it.

## Repo layout

```
simulation/paint_booth.rdk     RoboDK station (open this)
scripts/panel_programs.py      builds the door, rear fender, roof and trunk programs
scripts/SprayOn.py             spray macro used by the paint programs
models/car/                    car body (source .glb, cleaned .obj/.blend), licence
models/booth/                  booth STEP and wall STLs
models/conveyor/               conveyor rails, supports and cart STLs
models/robodk_library/         robot, track and gun files from the RoboDK library
docs n images/screenshots/     RoboDK and Fusion 360 screenshots
docs n images/paint_simulation.gif
```

The demo video (about 200 MB) and the RoboDK HTML exports (55 MB each) are too big for GitHub, so they're on Google Drive with the rest of the submission.

## Running it

1. Open `simulation/paint_booth.rdk` in RoboDK.
2. Run `Cell_Cycle` for the full cycle, or `Main_Paint` / any `Paint_*` program on its own.
3. Turn off live collision checking (Tools > Check collisions) for smoother playback. The paths are already verified.
4. Press Esc to clear the paint.

`panel_programs.py` needs RoboDK open with the station loaded and runs with RoboDK's bundled Python:

```
D:/Softwares/RoboDK/Python-Embedded/python.exe scripts/panel_programs.py Door
```

Hood and fender were tuned by hand, and the trunk and roof were reordered afterwards, so rerunning the script for those panels would undo that.

## Known limitations

- 3 points per pass, so the gun cuts slightly across curved panels and the standoff drifts a little. A production program would use denser points.
- A few strokes are joint moves where the straight line wasn't reachable.
- The rear end of the trunk is trimmed by 200 mm on one pass (out of reach at that angle). The neighbouring passes overlap it.
- One side of the car only.

## Credits

Car model: This work is based on "dodge car body" (https://sketchfab.com/3d-models/dodge-car-body-f9c68ef92d0147e3ac03403811117a94) by Dorin.Stoica (https://sketchfab.com/Dorin.Stoica) licensed under CC-BY-4.0 (http://creativecommons.org/licenses/by/4.0/)

Robot, track and spray gun models from the RoboDK library.
