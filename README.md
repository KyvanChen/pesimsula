# Rebuild Arena

A browser 3D driving simulator for the 2026 FRC game, REBUILT. One HTML file, three.js for drawing and Rapier for physics.

## Play

Any static file server works. From this folder:

```bash
python -m http.server 5173
```

Then open http://localhost:5173. Opening `index.html` directly from disk does not work, because the browser blocks the model files.

## Controls

| Key | Action |
| --- | --- |
| W A S D | Drive |
| Arrow keys | Turn |
| Left Shift | Intake |
| Space | Shoot |
| B | Stow intake and raise the shot blocker |
| C | Climb |
| V | Change camera |
| F | Field-relative / robot-relative |
| H | Hide the HUD |
| R | Reset (free play) |
| Esc | Pause |

Keys can be rebound in Settings. A gamepad also works.

## What is in here

- `index.html` — the whole game.
- `field.glb` — the official field, converted from the FIRST field CAD.
- `robot_calgames.glb`, `robot_calgames.json` — the Cal Games robot and its measured linkage (turret, four-bar intake, shot blocker).
- `tools/build_field.py`, `tools/build_robot.py` — rebuild the models from STEP files. They need Python with `cascadio`, `trimesh` and `fast-simplification`.

## Notes

Robot speeds, capacities and accuracy are estimates. Collisions use simple shapes, not the CAD mesh. Single player against AI only.
