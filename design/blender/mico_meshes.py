"""Export the Mico shell as per-link STL meshes for URDF visuals (RViz2, MuJoCo).

    python mico_meshes.py     # numpy only; writes src/humanoid_description/meshes/visual/mico/

One binary STL per (link, colour): every part of that colour bolted to that link is merged into it.
Vertices are in the link frame, metres, so the URDF visual needs no origin offset. manifest.json lists
the files and the colours; gen_robot.py reads it to write the mesh visuals into mico.urdf.xacro.
"""
import json
import struct
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import mico_parts as mp  # noqa: E402

OUT = mp.REPO / "src" / "humanoid_description" / "meshes" / "visual" / "mico"
RES = dict(n_lat=20, n_lon=40)        # ~1.5k triangles per part: smooth enough in a viewer, small files

# sRGB colours for the URDF materials (same palette as the Blender model)
COLOURS = {
    "white": "#F4F5F7", "orange": "#FF7A1A", "grey": "#C9D1DC", "joint": "#2A3D5E", "navy": "#1E3557",
    "visor": "#13233D", "window": "#06080C", "cyan": "#5BC8F5", "eye_glow": "#2E86B0", "pupil": "#0E1B30",
    "glint": "#FFFFFF", "red": "#DA251D", "yellow": "#FFCD00",
}
ALPHA = {}


def rgba(look):
    h = COLOURS[look].lstrip("#")
    return [round(int(h[i:i + 2], 16) / 255, 3) for i in (0, 2, 4)] + [ALPHA.get(look, 1.0)]


def write_stl(path, V, F):
    tri = V[F]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    rec = np.zeros(len(F), dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    rec["n"], rec["v"] = n, tri
    with open(path, "wb") as f:
        f.write(b"Mico shell, link frame, metres".ljust(80, b" "))
        f.write(struct.pack("<I", len(F)))
        f.write(rec.tobytes())


def main():
    groups = defaultdict(list)                  # (link, look) -> [(V, F)]
    for p in mp.parts():
        V, F = mp.part_mesh(p, **RES)
        groups[(p.link, p.look)].append((V, F))
    for _, link, look, V, F in mp.decals():
        groups[(link, look)].append((V, F))

    OUT.mkdir(parents=True, exist_ok=True)
    for old in OUT.glob("*.stl"):
        old.unlink()
    manifest = {"colours": {k: rgba(k) for k in COLOURS}, "links": defaultdict(list)}
    total = 0
    for (link, look), meshes in sorted(groups.items()):
        Vs, Fs, k = [], [], 0
        for V, F in meshes:
            Vs.append((V - np.array(mp.ORIGIN[link])) / 1000)
            Fs.append(F + k)
            k += len(V)
        name = f"{link}__{look}.stl"
        write_stl(OUT / name, np.vstack(Vs), np.vstack(Fs))
        manifest["links"][link].append({"file": name, "look": look})
        total += (OUT / name).stat().st_size
    (OUT / "manifest.json").write_text(json.dumps(manifest, indent=1) + "\n")
    print(f"wrote {sum(len(v) for v in manifest['links'].values())} STL files, {total / 1e6:.1f} MB, in {OUT}")


if __name__ == "__main__":
    main()
