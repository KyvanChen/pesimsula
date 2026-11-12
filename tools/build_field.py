#!/usr/bin/env python3
"""
Turns the official REBUILT field CAD (STEP) into rebuild-web/field.glb.

    py -3.12 tools/build_field.py "C:/path/to/FE-2026- REBUILT Playing Field.step"

Needs: pip install cascadio trimesh

What it does:
  * tessellates the STEP file
  * drops the staged FUEL (the sim has its own) and tiny hardware (rivets, PEMs...)
  * moves it into the sim's axes: X from the blue wall, Y up, Z across, metres
  * merges ~3000 parts into one mesh per colour so the browser draws it fast
  * makes the thin perimeter sheets (driver-station and guardrail panels) see-through
"""
import os
import re
import shutil
import sys
import tempfile

import cascadio
import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial

IN = 0.0254
L, W = 651.2 * IN, 317.7 * IN
DROP = re.compile(r"GE-26900: Fuel|PEM |Rivet|REV-21-2246|Flanged Bearing|Screw|Bolt|Washer|\bNut\b|"
                  r"Hook Tape|Loop Tape|Wheel Intergrated|Stealth Wheel", re.I)

here = os.path.dirname(os.path.abspath(__file__))
out = os.path.join(here, "..", "field.glb")
src = sys.argv[1]

with tempfile.TemporaryDirectory() as tmp:
    plain = os.path.join(tmp, "field.step")        # the converter cannot open non-ASCII paths
    shutil.copyfile(src, plain)
    raw = os.path.join(tmp, "raw.glb")
    if cascadio.step_to_glb(plain, raw, tol_linear=8.0, tol_angular=0.8) != 0:
        sys.exit("STEP conversion failed")
    scene = trimesh.load(raw)

# CAD axes: X along the field (blue at +X), Y across, Z up, origin at field centre.
to_sim = np.array([[-1, 0, 0, L / 2], [0, 0, 1, 0], [0, 1, 0, W / 2], [0, 0, 0, 1]], dtype=float)

groups = {}
kept = dropped = 0
for node in scene.graph.nodes_geometry:
    transform, name = scene.graph[node]
    mesh = scene.geometry[name]
    if DROP.search(node):
        dropped += len(mesh.faces)
        continue
    verts = trimesh.transform_points(mesh.vertices, to_sim @ transform)
    lo, hi = verts.min(0), verts.max(0)
    size = hi - lo
    if size.max() < 0.04:                           # loose hardware
        dropped += len(mesh.faces)
        continue
    rgb = tuple(int(c) for c in mesh.visual.material.main_color[:3])
    centre = (lo + hi) / 2
    outside = centre[0] < 0.03 or centre[0] > L - 0.03 or centre[2] < 0.03 or centre[2] > W - 0.03
    glass = outside and size.min() < 0.02 and size.max() > 0.5 and hi[1] > 0.3
    # Everything behind an alliance wall (driver stations, outposts) is kept as its own group, so the
    # game can fade it out when the camera sits behind it.
    zone = "endBlue" if centre[0] < 0.03 else "endRed" if centre[0] > L - 0.03 else "field"
    groups.setdefault((zone, rgb, glass), []).append((verts, mesh.faces))
    kept += len(mesh.faces)

result = trimesh.Scene()
for i, ((zone, rgb, glass), parts) in enumerate(sorted(groups.items(), key=lambda kv: (kv[0][2], kv[0][0]))):
    offset, vs, fs = 0, [], []
    for v, f in parts:
        vs.append(v)
        fs.append(f + offset)
        offset += len(v)
    mesh = trimesh.Trimesh(np.vstack(vs), np.vstack(fs), process=False)
    colour = [c / 255 for c in rgb] + [0.18 if glass else 1.0]
    mesh.visual = trimesh.visual.TextureVisuals(material=PBRMaterial(
        name=f"{'glass' if glass else 'part'}_{i}", baseColorFactor=colour, metallicFactor=0.1,
        roughnessFactor=0.7, alphaMode="BLEND" if glass else "OPAQUE", doubleSided=True))
    result.add_geometry(mesh, node_name=f"{zone}_{i}", geom_name=f"{zone}_{i}")

result.export(out)
print(f"{out}: {len(groups)} meshes, {kept:,} triangles kept, {dropped:,} dropped, "
      f"{os.path.getsize(out) / 1e6:.1f} MB")
