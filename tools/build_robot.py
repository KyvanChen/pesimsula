#!/usr/bin/env python3
"""
Turns a robot CAD assembly (STEP) into a light GLB the game can animate.

    py -3.12 tools/build_robot.py calgames FULL.step --shooter SHOOTER.step --intake INTAKE.step

Needs: pip install cascadio trimesh fast-simplification

The full assembly gives the body. The isolated shooter and intake files are
used to split the robot into separate node groups so the game can move them:

    body_*     fixed to the chassis
    turret_*   spins about the turret axis (stored in its own frame: +X fires, origin on the axis)
    head_*     the intake head; stays level, carried by the four-bar
    swingN_*   each pivoting link (the two four-bar arms, the lower roller arm)
    blocker_*  the shot-blocker plate, hinged on the head

A robot_<name>.json is written beside the GLB with the measured pivots and swing angles.

Output axes match the game's robots: +X is the front (intake side), Y is up,
the chassis is centred on the origin and sits on the floor.
"""
import argparse
import collections
import json
import os
import random
import re
import shutil
import tempfile

import cascadio
import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial

DROP = re.compile(r"CONN-|R0402|LED0805|C0402|C0603|SHCS|BHCS|Screw|Bolt|Washer|\bNut\b|Bearing|Spacer|"
                  r"retaining|Rivet|^102742|^id\d+|ZX360|RR-MSAFPM|regulator|Shaft Collar", re.I)
BUDGET = {"body": 60_000, "turret": 26_000, "head": 22_000, "arm": 5_000, "sub": 8_000, "blocker": 2_000}

ap = argparse.ArgumentParser()
ap.add_argument("name")
ap.add_argument("full")
ap.add_argument("--shooter")
ap.add_argument("--intake")
args = ap.parse_args()
here = os.path.dirname(os.path.abspath(__file__))
out = os.path.join(here, "..", f"robot_{args.name}.glb")


def convert(src, tag):
    """STEP -> raw GLB, cached in the temp folder (the converter needs an ASCII path)."""
    raw = os.path.join(tempfile.gettempdir(), f"robot_{args.name}{tag}_raw.glb")
    if not os.path.exists(raw) or os.path.getmtime(raw) < os.path.getmtime(src):
        with tempfile.TemporaryDirectory() as tmp:
            plain = os.path.join(tmp, "in.step")
            shutil.copyfile(src, plain)
            if cascadio.step_to_glb(plain, raw, tol_linear=3.0, tol_angular=0.8) != 0:
                raise SystemExit(f"STEP conversion failed: {src}")
    return trimesh.load(raw)


def colour_of(mesh):
    try:
        return tuple(int(c) for c in mesh.visual.material.main_color[:3])
    except Exception:
        return (128, 128, 128)


def parts_of(scene):
    rows = []
    for node in scene.graph.nodes_geometry:
        transform, geom = scene.graph[node]
        mesh = scene.geometry[geom]
        verts = trimesh.transform_points(mesh.vertices, transform)
        rows.append(dict(name=node, verts=verts, faces=mesh.faces, rgb=colour_of(mesh), centroid=verts.mean(0),
                         sig=(len(mesh.vertices), len(mesh.faces), tuple(np.round(np.sort(mesh.extents), 4)))))
    return rows


# ---- robot-specific layout (measured from this CAD; CAD axes are X across, Y along, Z up) ----
FLOOR_Z = -0.247                       # bottom of the wheels
FRAME_X = (-0.65, 0.09)                # drivetrain frame rails
FRAME_Y = (0.15, 0.78)
cx, cy = sum(FRAME_X) / 2, sum(FRAME_Y) / 2
# CAD -Y (the intake side) becomes game +X; CAD Z becomes game Y (up); CAD X becomes game Z.
to_game = np.array([[0, -1, 0, cy], [0, 0, 1, -FLOOR_Z], [1, 0, 0, -cx], [0, 0, 0, 1]], dtype=float)
# Box around the blocker plate, game axes.
BLOCKER = (np.array([0.53, 0.49, -0.39]), np.array([0.94, 0.66, 0.39]))
INTAKE_MIN_X = 0.30                    # intake parts all sit ahead of the front frame rail
TURRET_AT = np.array([-0.136, 0.366, -0.187])   # centre of the turret ring (measured from the ring parts)
TURRET_RADIUS = 0.17

full = parts_of(convert(args.full, ""))
for r in full:
    r["verts"] = trimesh.transform_points(r["verts"], to_game)
    r["centroid"] = r["verts"].mean(0)
    r["group"] = "body"

# ---- intake linkage -------------------------------------------------------------------------
# The full assembly has the intake OUT; the isolated intake file has it STOWED. Comparing the two
# poses recovers each rigid link and how it moves, so the game can drive a real four-bar.
info = {}
extra = []
rig = {}


def kabsch(a, b):
    ca, cb = a.mean(0), b.mean(0)
    u, _, vt = np.linalg.svd((a - ca).T @ (b - cb))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rot = vt.T @ np.diag([1, 1, d]) @ u.T
    return rot, cb - rot @ ca


def fit2(a, b):
    """Planar rigid fit: angle, rotation, translation with b ~= a @ R.T + t."""
    ca, cb = a.mean(0), b.mean(0)
    h = (a - ca).T @ (b - cb)
    ang = np.arctan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    rot = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
    return ang, rot, cb - rot @ ca


if args.intake:
    rng = random.Random(5)
    sub = parts_of(convert(args.intake, "_intake"))
    by_sig = collections.defaultdict(list)
    for r in full:
        by_sig[r["sig"]].append(r)

    # 1. Register the file onto the robot using the parts that do not move (the roller bed in the frame).
    cand = [(r, f) for r in sub for f in by_sig.get(r["sig"], [])]
    A = np.array([c[0]["centroid"] for c in cand])
    B = np.array([c[1]["centroid"] for c in cand])
    best = (0, None)
    for _ in range(6000):
        i, j, k = rng.sample(range(len(cand)), 3)
        a, b = A[[i, j, k]], B[[i, j, k]]
        if np.linalg.norm(np.cross(a[1] - a[0], a[2] - a[0])) < 2e-3:
            continue
        if any(abs(np.linalg.norm(a[m] - a[n]) - np.linalg.norm(b[m] - b[n])) > 0.0015 for m, n in ((0, 1), (0, 2), (1, 2))):
            continue
        rot, t = kabsch(a, b)
        n = int((np.linalg.norm(A @ rot.T + t - B, axis=1) < 0.002).sum())
        if n > best[0]:
            best = (n, (rot, t))
    rot, t = best[1]
    keep = np.linalg.norm(A @ rot.T + t - B, axis=1) < 0.003
    rot, t = kabsch(A[keep], B[keep])
    for r in sub:                                        # the file, placed on the robot = the STOWED pose
        r["verts"] = r["verts"] @ rot.T + t
        r["centroid"] = r["verts"].mean(0)

    # 2. Pair each file part with the same part in the full robot. The motion is planar, so Z is unchanged.
    pairs = [(i, r, f) for i, r in enumerate(sub) for f in by_sig.get(r["sig"], [])
             if abs(r["centroid"][2] - f["centroid"][2]) < 0.0015]
    still = {i for i, r, f in pairs if np.linalg.norm(r["centroid"] - f["centroid"]) < 0.002}
    moving = [q for q in pairs if q[0] not in still]
    S = np.array([q[1]["centroid"][:2] for q in moving])     # stowed
    O = np.array([q[2]["centroid"][:2] for q in moving])     # out
    ids = np.array([q[0] for q in moving])

    # 3. Pull out rigid links one at a time: each is a set of parts sharing one planar motion.
    free = np.ones(len(sub), bool)
    links = []
    for _ in range(12):
        pool = [k for k in range(len(moving)) if free[ids[k]]]
        if len({ids[k] for k in pool}) < 3:
            break
        best = (0, None)
        for _ in range(20000):
            i, j = rng.sample(pool, 2)
            gap = np.linalg.norm(S[i] - S[j])
            if ids[i] == ids[j] or gap < 0.03 or abs(gap - np.linalg.norm(O[i] - O[j])) > 0.001:
                continue
            _, r2, t2 = fit2(S[[i, j]], O[[i, j]])
            err = np.linalg.norm(S[pool] @ r2.T + t2 - O[pool], axis=1)
            n = len({ids[pool[k]] for k in np.where(err < 0.002)[0]})
            if n > best[0]:
                best = (n, (r2, t2))
        if best[0] < 3:
            break
        r2, t2 = best[1]
        inl = [k for k in range(len(moving)) if free[ids[k]] and np.linalg.norm(S[k] @ r2.T + t2 - O[k]) < 0.003]
        ang, r2, t2 = fit2(S[inl], O[inl])
        inl = [k for k in range(len(moving)) if free[ids[k]] and np.linalg.norm(S[k] @ r2.T + t2 - O[k]) < 0.003]
        members = {ids[k]: moving[k][2] for k in inl}
        for k in members:
            free[k] = False
        size = max((m["verts"].max(0) - m["verts"].min(0)).max() for m in members.values())
        links.append(dict(ang=ang, rot=r2, t=t2, parts=list(members.values()), size=size))

    # 4. Name the links by how they move (stowed -> out).
    #    head:  pure translation (parallelogram four-bar keeps it level)
    #    arm:   the largest remaining clockwise swing about a pivot on the frame
    #    plate: swings the other way, hinged on the head
    #    sub:   anything else that carries real parts (the lower roller arm)
    head = min(links, key=lambda k: abs(k["ang"]))
    rest = [k for k in links if k is not head and k["size"] > 0.15]
    plate = max(rest, key=lambda k: k["ang"])
    arm = min(rest, key=lambda k: k["ang"])
    swings = [k for k in rest if k is not plate]         # every pivoting link, the two four-bar arms included
    for k in links:
        pole = np.linalg.solve(np.eye(2) - k["rot"], k["t"]) if abs(k["ang"]) > 1e-3 else None
        print(f"  link: {len(k['parts']):3d} parts, {np.degrees(k['ang']):7.1f} deg stowed->out, pole {pole}, size {k['size']:.2f}")
    for f in head["parts"]:
        f["group"] = "head"
    for f in plate["parts"]:
        f["group"] = "blocker"
    for n, link in enumerate(swings):
        for f in link["parts"]:
            f["group"] = f"swing{n}"
    # Intake parts that could not be paired ride with whichever link they sit closest to.
    linked = [f for f in full if f["group"] != "body"]
    pts = np.array([f["centroid"] for f in linked])
    sigs = {r["sig"] for r in sub}
    fixed = {id(q[2]) for q in pairs if q[0] in still}
    for f in full:
        if f["group"] == "body" and f["sig"] in sigs and id(f) not in fixed:
            dist = np.linalg.norm(pts - f["centroid"], axis=1)
            if dist.min() < 0.10:
                f["group"] = linked[int(dist.argmin())]["group"]

    # 5. The numbers the game needs, all in robot axes for the OUT pose (x forward, y up).
    eye = np.eye(2)
    d = head["t"]                                        # head shift, stowed -> out
    pivot = np.linalg.solve(eye - arm["rot"], arm["t"])  # arm's pivot on the frame
    joint_out = pivot + arm["rot"] @ np.linalg.solve(arm["rot"] - eye, d)   # where the arm meets the head
    hinge_out = np.linalg.solve(plate["rot"] - eye, d - plate["t"]) + d     # plate hinge on the head
    plate_parts = [f for f in full if f["group"] == "blocker"]
    plo = np.min([f["verts"].min(0) for f in plate_parts], 0)
    phi = np.max([f["verts"].max(0) for f in plate_parts], 0)
    rig = dict(
        arm=dict(pivot=pivot.tolist(), swing=float(-arm["ang"]), reach=(joint_out - pivot).tolist()),
        plate=dict(hinge=hinge_out.tolist(), swing=float(-plate["ang"]), length=float(phi[0] - hinge_out[0]),
                   halfWidth=float((phi[2] - plo[2]) / 2)),
        swings=[dict(pivot=np.linalg.solve(eye - k["rot"], k["t"]).tolist(), swing=float(-k["ang"])) for k in swings],
        stowShift=(-d).tolist(),
    )
    for tag in ["head", "blocker"] + [f"swing{n}" for n in range(len(swings))]:
        sel = [f for f in full if f["group"] == tag]
        if sel:
            info[tag] = (np.min([f["verts"].min(0) for f in sel], 0), np.max([f["verts"].max(0) for f in sel], 0), len(sel))

# ---- turret: the shooter in the full assembly is an older version, so it is cleared out and the
# isolated shooter is dropped onto the turret ring. Turret-local axes: +X fires, Y up, origin on the axis.
if args.shooter:
    for r in full:
        rel = r["centroid"] - TURRET_AT
        if np.hypot(rel[0], rel[2]) < TURRET_RADIUS and rel[1] > -0.03 and r["group"] == "body":
            r["group"] = None
    to_turret = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1]], dtype=float)
    extra = parts_of(convert(args.shooter, "_turret"))
    for r in extra:
        r["verts"] = trimesh.transform_points(r["verts"], to_turret)
        r["group"] = "turret"
    info["turret"] = (np.min([r["verts"].min(0) for r in extra], 0), np.max([r["verts"].max(0) for r in extra], 0), len(extra))

# Mirrored parts can be missing from the export (the converter skips them). Where a large flat
# intake plate on one side has no twin on the other side, add its mirror image.
side = [r for r in full if r["group"] not in ("body", None) and abs(r["centroid"][2]) > 0.25]
for r in side:
    size = r["verts"].max(0) - r["verts"].min(0)
    if size.max() < 0.2 or size[2] > 0.02:
        continue
    twin = r["centroid"] * [1, 1, -1]
    if any(np.linalg.norm(o["centroid"] - twin) < 0.03 for o in side):
        continue
    verts = r["verts"] * [1, 1, -1]
    extra.append(dict(r, verts=verts, faces=r["faces"][:, ::-1], centroid=verts.mean(0)))
    print(f"mirrored missing plate: {r['name']} (z {r['centroid'][2]:+.2f} -> {twin[2]:+.2f}), size {np.round(size, 2)}")

groups = {}
kept = collections.Counter()
for r in full + extra:
    size = r["verts"].max(0) - r["verts"].min(0)
    if r["group"] is None or DROP.search(r["name"]) or size.max() < 0.035:
        continue
    groups.setdefault((r["group"], r["rgb"]), []).append((r["verts"], r["faces"]))
    kept[r["group"]] += len(r["faces"])

result = trimesh.Scene()
total = collections.Counter()
for i, ((part, rgb), pieces) in enumerate(sorted(groups.items())):
    offset, vs, fs = 0, [], []
    for v, f in pieces:
        vs.append(v)
        fs.append(f + offset)
        offset += len(v)
    mesh = trimesh.Trimesh(np.vstack(vs), np.vstack(fs), process=True)
    want = int(len(mesh.faces) * min(1.0, BUDGET.get(part, 6_000) / kept[part]))
    if want >= 60 and len(mesh.faces) > 300:
        try:
            mesh = mesh.simplify_quadric_decimation(face_count=want)
        except Exception as err:                         # keep the full mesh rather than lose the part
            print("  could not decimate", part, rgb, err)
    mesh.visual = trimesh.visual.TextureVisuals(material=PBRMaterial(
        name=f"{part}_{i}", baseColorFactor=[c / 255 for c in rgb] + [1.0],
        metallicFactor=0.3, roughnessFactor=0.6, doubleSided=True))
    result.add_geometry(mesh, node_name=f"{part}_{i}", geom_name=f"{part}_{i}")
    total[part] += len(mesh.faces)

result.export(out)
print(f"{out}: {dict(total)} triangles, {os.path.getsize(out) / 1e6:.2f} MB")
np.set_printoptions(precision=3, suppress=True)
for tag, (lo, hi, n) in info.items():
    print(f"{tag}: {n} parts, bounds {lo} .. {hi}")
rig["turret"] = dict(at=TURRET_AT.tolist())
with open(os.path.splitext(out)[0] + ".json", "w") as fh:
    json.dump(rig, fh, indent=1)
print(json.dumps(rig, indent=1))
