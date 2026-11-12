# Pesimsula

A browser 3D driving simulator for the 2026 FRC game, REBUILT. One HTML file, three.js for drawing and Rapier for physics.

## Play

**Play it in your browser: https://kyvanchen.github.io/pesimsula/**

Nothing to install. To run it yourself instead, any static file server works. From this folder:

```bash
python -m http.server 5173
```

Then open http://localhost:5173. Opening `index.html` directly from disk does not work, because the browser blocks the model files.

## Controls

| Key | Action |
| --- | --- |
| W A S D | Drive |
| J / L, or arrow keys | Turn |
| Left Shift | Intake |
| Space | Shoot |
| K | Elite Ball Knowledge: stow intake and raise the shot blocker. Peninsula Pups: hold to eject fuel |
| C | Climb |
| V | Change camera |
| F | Field-relative / robot-relative |
| H | Hide the HUD |
| R | Reset (free play) |
| Esc | Pause |

Keys can be rebound in Settings. A gamepad also works.

## Multiplayer

Menu → Multiplayer. One player hosts and gets a five-letter room code; up to five others join with it, pick a robot and an alliance, and the host starts the match. AI fills the empty spots.

The host's browser runs the whole match, so it ends for everyone if the host leaves or closes the tab. Browsers first try to connect directly to each other (WebRTC, introduced through the public PeerJS service). When that fails, which is common between two computers on the same Wi-Fi and on school networks, the joiner switches by itself to a relay through a public MQTT broker; the lobby says so. The relay is a little slower and is a free public service with no guarantees.

## What is in here

- `index.html` — the whole game.
- `field.glb` — the official field, converted from the FIRST field CAD.
- `robot_calgames.glb`, `robot_calgames.json` — the Elite Ball Knowledge robot and its measured linkage (turret, four-bar intake, shot blocker).
- `robot_reblitz.glb` — Re-Blitz, simplified from Team 2910's public CAD release, with stats from their 2026 tech binder.
- `robot_shuttler.glb` — Peninsula Pups, a tall fuel shuttle with no shooter, simplified from its CAD.
- `tools/build_field.py`, `tools/build_robot.py`, `tools/build_robot_static.py` — rebuild the models from STEP files. They need Python with `cascadio`, `trimesh` and `fast-simplification`.

## Notes

Robot speeds, capacities and accuracy are estimates. Fuel in a hopper is a loose pile that shifts as the robot drives; the Elite Ball Knowledge and Peninsula Pups hoppers are open, so anything piled above their walls rolls back onto the field, while Re-Blitz stops at 50. AI drivers play each robot its own way: Elite Ball Knowledge scores while it keeps collecting, Re-Blitz fills up and parks in front of the hub to unload, and Peninsula Pups ferries fuel from the neutral zone to its shooters (at most one per alliance, never alone). Collisions use simple shapes, not the CAD mesh. Collisions and scoring are decided on the host in multiplayer.

## Credits

Re-Blitz is the 2026 robot of FRC Team 2910, Jack in the Bot. The model and numbers come from the CAD and tech binder they published; this project is not affiliated with or endorsed by the team. The field model comes from the official FIRST field CAD.
