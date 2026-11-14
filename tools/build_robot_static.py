#!/usr/bin/env python3
"""
Turns a robot CAD assembly (STEP) into a light, single-pose GLB for the game.

    py -3.12 tools/build_robot_static.py reblitz "12 - Robot 2 Top Level Assembly.step" [--hood HOOD.step] [--drop REGEX]
                                         [--front-x CX,CY] [--clear REGEX]
    py -3.12 tools/build_robot_static.py skadi ROBOT.step --front-nz CX,CZ --turret X,Y,Z,DX,DZ REGEX [--tilt REGEX]

Needs: pip install cascadio trimesh fast-simplification

For robots whose CAD is already centred on the chassis with Z up and the intake
towards -Y (true of the 2910 Re-Blitz release). Nothing is animated: parts are
merged per colour and decimated. Bumpers in the CAD are dropped, because the game
draws its own in the alliance colour. Clear hopper panels are kept see-through.

--hood names a second STEP holding just the shooter hood, exported in place. Those parts become
hood_* nodes so the game can tilt them, and the hinge (the axle the hood turns on) is printed.

--front-x CX,CY is for CAD that is not centred and has its intake towards +X instead: CX,CY is the middle of
the chassis in CAD metres. --clear REGEX names more parts to draw see-through (hopper panels with no useful name).

--front-nx CX,CZ is for CAD with Y up and the intake towards -X: CX,CZ is the middle of the chassis in CAD metres.
--front-nz CX,CZ is the same with the intake towards -Z.

--turret X,Y,Z,DX,DZ REGEX puts the parts whose path matches REGEX into turret_* nodes, for the game to spin. X,Y,Z is a
point on the turret's axis and DX,DZ the way it fires as drawn, all in game axes; the nodes are stored in the turret's own
axes (origin at that point, +X fires). --tilt REGEX names the turret parts that tilt with the hood: they become tilt_* nodes.

Output axes match the game's robots: +X is the front (intake side), Y is up,
the chassis is centred on the origin and sits on the floor.
"""
import collections
import os
import re
import shutil
import sys
import tempfile

import cascadio
import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial

np.set_printoptions(precision=3, suppress=True)

DROP = re.compile(r"Screw|Bolt|Washer|Nut\b|Locknut|\bRivet|Spacer|Bearing|Insert|Shaft Collar|Origin Cube|^Fuel|"
                  r"Bumper|Chain_?\d*$|Zip Tie|PEM", re.I)
CLEAR = re.compile(r"Hopper", re.I)        # sub-assembly whose big thin panels are polycarbonate
BUDGET = 110_000                           # triangles

hood_src = None
extra_drop = None                          # --drop REGEX: more part paths to leave out (bumpers with no useful name, say)
if "--drop" in sys.argv:
    k = sys.argv.index("--drop")
    extra_drop = re.compile(sys.argv[k + 1])
    del sys.argv[k:k + 2]
front_x = None
if "--front-x" in sys.argv:
    k = sys.argv.index("--front-x")
    front_x = [float(v) for v in sys.argv[k + 1].split(",")]
    del sys.argv[k:k + 2]
extra_clear = None
if "--clear" in sys.argv:
    k = sys.argv.index("--clear")
    extra_clear = re.compile(sys.argv[k + 1])
    del sys.argv[k:k + 2]
front_nx = None
if "--front-nx" in sys.argv:
    k = sys.argv.index("--front-nx")
    front_nx = [float(v) for v in sys.argv[k + 1].split(",")]
    del sys.argv[k:k + 2]
front_nz = False
if "--front-nz" in sys.argv:
    k = sys.argv.index("--front-nz")
    front_nx = [float(v) for v in sys.argv[k + 1].split(",")]
    front_nz = True
    del sys.argv[k:k + 2]
turret = tilt = None
if "--tilt" in sys.argv:
    k = sys.argv.index("--tilt")
    tilt = re.compile(sys.argv[k + 1])
    del sys.argv[k:k + 2]
if "--turret" in sys.argv:
    k = sys.argv.index("--turret")
    turret = ([float(v) for v in sys.argv[k + 1].split(",")], re.compile(sys.argv[k + 2]))
    del sys.argv[k:k + 3]
if "--hood" in sys.argv:
    k = sys.argv.index("--hood")
    hood_src = sys.argv[k + 1]
    del sys.argv[k:k + 2]
name, src = sys.argv[1], sys.argv[2]
here = os.path.dirname(os.path.abspath(__file__))
out = os.path.join(here, "..", f"robot_{name}.glb")
raw = os.path.join(tempfile.gettempdir(), f"robot_{name}_raw.glb")
if len(sys.argv) > 3:
    raw = sys.argv[3]                      # an already converted GLB
elif not os.path.exists(raw) or os.path.getmtime(raw) < os.path.getmtime(src):
    with tempfile.TemporaryDirectory() as tmp:
        plain = os.path.join(tmp, "in.step")           # the converter needs an ASCII path
        shutil.copyfile(src, plain)
        if cascadio.step_to_glb(plain, raw, tol_linear=3.0, tol_angular=0.8) != 0:
            raise SystemExit("STEP conversion failed")


def to_glb(step, glb):
    with tempfile.TemporaryDirectory() as tmp:
        plain = os.path.join(tmp, "in.step")
        shutil.copyfile(step, plain)
        if cascadio.step_to_glb(plain, glb, tol_linear=3.0, tol_angular=0.8) != 0:
            raise SystemExit("STEP conversion failed")


scene = trimesh.load(raw)
graph = scene.graph
hood_keys = set()
if hood_src:
    hood_raw = os.path.join(tempfile.gettempdir(), f"robot_{name}_hood_raw.glb")
    if not os.path.exists(hood_raw) or os.path.getmtime(hood_raw) < os.path.getmtime(hood_src):
        to_glb(hood_src, hood_raw)
    hs = trimesh.load(hood_raw)
    for node in hs.graph.nodes_geometry:
        t, g = hs.graph[node]
        m = hs.geometry[g]
        if hasattr(m, "faces"):
            hood_keys.add((len(m.faces), tuple(np.round(trimesh.transform_points(m.vertices, t).mean(0), 3))))


def path_of(node):
    names = []
    while node is not None and node != graph.base_frame:
        names.append(str(node))
        node = graph.transforms.parents.get(node)
    return " / ".join(reversed(names))


parts = []
for node in graph.nodes_geometry:
    transform, geom = graph[node]
    mesh = scene.geometry[geom]
    if not hasattr(mesh, "faces"):         # sketch curves, not solids
        continue
    try:
        rgba = tuple(int(c) for c in mesh.visual.material.main_color)
    except Exception:
        rgba = (128, 128, 128, 255)
    verts = trimesh.transform_points(mesh.vertices, transform)
    parts.append(dict(path=path_of(node), verts=verts, faces=mesh.faces, rgba=rgba,
                      hood=(len(mesh.faces), tuple(np.round(verts.mean(0), 3))) in hood_keys))

floor = min(p["verts"][:, 1 if front_nx else 2].min() for p in parts if not re.search("Origin Cube", p["path"]))
# CAD -Y (intake side) -> game +X; CAD Z -> game Y (up); CAD X -> game Z.
to_game = np.array([[0, -1, 0, 0], [0, 0, 1, -floor], [1, 0, 0, 0], [0, 0, 0, 1]], dtype=float)
if front_x:                                # CAD +X (intake side) -> game +X; CAD Z -> game Y; CAD Y -> game -Z
    to_game = np.array([[1, 0, 0, -front_x[0]], [0, 0, 1, -floor], [0, -1, 0, front_x[1]], [0, 0, 0, 1]], dtype=float)
if front_nx:                               # CAD -X (intake side) -> game +X; CAD Y -> game Y; CAD Z -> game -Z
    to_game = np.array([[-1, 0, 0, front_nx[0]], [0, 1, 0, -floor], [0, 0, -1, front_nx[1]], [0, 0, 0, 1]], dtype=float)
if front_nz:                               # CAD -Z (intake side) -> game +X; CAD Y -> game Y; CAD X -> game Z
    to_game = np.array([[0, 0, -1, front_nx[1]], [0, 1, 0, -floor], [1, 0, 0, -front_nx[0]], [0, 0, 0, 1]], dtype=float)
to_turret = None
if turret:
    (tx, ty, tz, dx, dz), turret_re = turret
    n = np.hypot(dx, dz)
    dx, dz = dx / n, dz / n
    rot = np.array([[dx, 0, dz], [0, 1, 0], [-dz, 0, dx]])
    to_turret = np.eye(4)
    to_turret[:3, :3] = rot
    to_turret[:3, 3] = -rot @ np.array([tx, ty, tz])

groups = collections.defaultdict(list)
heavy_of = {}                              # (group, index) -> the part's path in the assembly
dropped = collections.Counter()
kept = 0
for p in parts:
    verts = trimesh.transform_points(p["verts"], to_game)
    size = verts.max(0) - verts.min(0)
    hit = DROP.search(p["path"]) or (extra_drop and extra_drop.search(p["path"]))
    fuel = size.min() > 0.14 and size.max() < 0.16 and p["rgba"][0] > 200 and p["rgba"][2] < 80      # a game piece left in the CAD
    if hit or fuel or size.max() < 0.035:
        dropped[hit.group(0).lower() if hit else "fuel" if fuel else "small"] += 1
        continue
    clear = bool(CLEAR.search(p["path"])) and size.min() < 0.01 and np.sort(size)[1] > 0.12 or p["rgba"][3] < 250 or bool(extra_clear and extra_clear.search(p["path"]))
    kind = "hood" if p["hood"] else "body"
    if turret and turret_re.search(p["path"]):
        kind = "tilt" if tilt and tilt.search(p["path"]) else "turret"
        verts = trimesh.transform_points(verts, to_turret)
    key = (kind, clear, p["rgba"][:3])
    heavy_of[(key, len(groups[key]))] = p["path"]
    groups[key].append((verts, p["faces"]))
    kept += len(p["faces"])

if front_x or front_nx:
    # CAD like this is one colour, so everything lands in one group, and decimating that as a single mesh tears the thin
    # plates into shards. Thin each part by itself instead. Plates and other simple parts are left alone; the few very
    # detailed ones lose most of their triangles; and the big bought-in sub-assemblies (swerve modules: a hundred parts
    # each, most of the triangles in the file, mostly hidden behind the bumpers) take whatever cut is needed to fit.
    sizes = collections.Counter(path.split(" / ")[1] for path in heavy_of.values() if path.count(" / ") > 1)
    plan, fixed, flex = [], 0, 0
    for key, pieces in groups.items():
        for k, (v, f) in enumerate(pieces):
            path = heavy_of[(key, k)]
            if path.count(" / ") > 1 and sizes[path.split(" / ")[1]] > 20:
                plan.append((key, k, None)); flex += len(f)
            else:
                r = 1.0 if len(f) <= 2500 else 0.6 if len(f) <= 12000 else 0.25
                plan.append((key, k, r)); fixed += int(len(f) * r)
    squeeze = min(1.0, max(0.03, (BUDGET * 1.6 - fixed) / max(1, flex)))
    for key, k, r in plan:
        v, f = groups[key][k]
        r = squeeze if r is None else r
        if r >= 1.0 or len(f) < 200:
            continue
        try:
            m = trimesh.Trimesh(v, f, process=True).simplify_quadric_decimation(face_count=max(60, int(len(f) * r)))
            groups[key][k] = (m.vertices, m.faces)
        except Exception as err:
            print("  could not decimate a part", err)
    kept = 0                               # already thinned: the merged meshes below are left as they are

result = trimesh.Scene()
total = 0
lo, hi = np.full(3, 9.0), np.full(3, -9.0)
for i, ((kind, clear, rgb), pieces) in enumerate(sorted(groups.items())):
    offset, vs, fs = 0, [], []
    for v, f in pieces:
        vs.append(v)
        fs.append(f + offset)
        offset += len(v)
    mesh = trimesh.Trimesh(np.vstack(vs), np.vstack(fs), process=True)
    want = int(len(mesh.faces) * min(1.0, BUDGET / kept)) if kept else 0
    if want >= 60 and len(mesh.faces) > 300:
        try:
            mesh = mesh.simplify_quadric_decimation(face_count=want)
        except Exception as err:
            print("  could not decimate", rgb, err)
    colour = [c / 255 for c in rgb] + [0.28 if clear else 1.0]
    mesh.visual = trimesh.visual.TextureVisuals(material=PBRMaterial(
        name=f"{'clear' if clear else 'body'}_{i}", baseColorFactor=colour, metallicFactor=0.3, roughnessFactor=0.6,
        alphaMode="BLEND" if clear else "OPAQUE", doubleSided=True))
    result.add_geometry(mesh, node_name=f"{kind if kind != 'body' else 'clear' if clear else 'body'}_{i}", geom_name=f"g_{i}")
    total += len(mesh.faces)
    print(f"  {kind}{' clear' if clear else ''} {rgb}: {len(mesh.faces):,} triangles, {mesh.vertices.min(0)} .. {mesh.vertices.max(0)}")
    if kind in ("body", "hood"):
        lo, hi = np.minimum(lo, mesh.vertices.min(0)), np.maximum(hi, mesh.vertices.max(0))

result.export(out)
np.set_printoptions(precision=3, suppress=True)
print(f"{out}: {len(groups)} meshes, {total:,} triangles, {os.path.getsize(out) / 1e6:.2f} MB")
print("bounds (game axes):", lo, hi, " floor z in CAD:", round(float(floor), 4))
print("dropped:", dict(dropped))
if hood_src:
    hp = [p for p in parts if p["hood"]]
    print(f"hood: {len(hp)} of {len(hood_keys)} parts found in the robot")
    # The hood turns on the widest shaft bearings in the file (the main drum axle).
    axle = [trimesh.transform_points(p["verts"], to_game) for p in hp if "1.125\" OD" in p["path"]]
    if axle:
        a = np.vstack(axle)
        print("hood hinge (game x forward, y up):", ((a.min(0) + a.max(0)) / 2)[:2])
    allv = np.vstack([trimesh.transform_points(p["verts"], to_game) for p in hp])
    print("hood bounds (game axes):", allv.min(0), allv.max(0))
