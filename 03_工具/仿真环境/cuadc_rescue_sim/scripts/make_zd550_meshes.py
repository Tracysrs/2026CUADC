#!/usr/bin/env python3
"""Generate ZD550 meshes for the iris_d435i_airframe model.

Input : Frame ZD550.stl (CAD export, millimetres, z-up, origin at body centre)
Output: meshes/zd550_frame.stl          full-detail visual (metres)
        meshes/zd550_body_collision.stl convex hull of body+arms (metres)

Transform: scale mm -> m and shift z by -21.5 mm so the model origin sits at
the frame body centre (plates span raw z 0..43 mm), matching the iris
convention. No other reshaping: the landing gear (struts + beam + skids) is
connected in the source CAD. Its low-poly collision is done with SDF box
primitives in model.sdf (column / beam / 2 skids), see the geometry survey:
  body plates + arms + motor mounts : raw z 0..43 mm, motors at (+-219.6, +-219.6)
  under-body post / column          : raw z -20..-60 mm
  X beam                            : raw z -220..-240 mm, x +-120, y +-15
  2 skid rails along Y              : raw z -250..-268 mm, x +-102..126, y +-165
"""

import os
import struct

import numpy as np
from scipy.spatial import ConvexHull

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRC = os.path.join(PKG, "Frame ZD550.stl")
OUT_DIR = os.path.join(PKG, "models", "iris_d435i_airframe", "meshes")

Z_SHIFT_M = -0.0215  # origin -> frame body centre
MM = 0.001


def load_binary_stl(path):
    with open(path, "rb") as f:
        f.read(80)
        (n,) = struct.unpack("<I", f.read(4))
        data = np.frombuffer(f.read(n * 50), dtype=np.uint8).reshape(n, 50)
    return data[:, 12:48].copy().view("<f4").reshape(n, 3, 3).astype(np.float64)


def save_binary_stl(path, tris):
    tris = np.asarray(tris, dtype=np.float32)
    n = len(tris)
    normal = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    norm = np.linalg.norm(normal, axis=1, keepdims=True)
    normal = np.divide(normal, norm, out=np.zeros_like(normal), where=norm > 0)
    with open(path, "wb") as f:
        f.write(b"zd550 mesh, units metres".ljust(80, b"\0"))
        f.write(struct.pack("<I", n))
        for i in range(n):
            f.write(struct.pack("<3f", *normal[i]))
            for v in tris[i]:
                f.write(struct.pack("<3f", *v))
            f.write(struct.pack("<H", 0))
    print(f"wrote {path}: {n} tris")


def convex_hull_tris(points):
    hull = ConvexHull(points)
    tris = hull.points[hull.simplices]
    centre = points.mean(axis=0)
    for i, t in enumerate(tris):
        n = np.cross(t[1] - t[0], t[2] - t[0])
        if np.dot(n, t.mean(axis=0) - centre) < 0:
            tris[i] = t[[0, 2, 1]]
    return tris


def main():
    tris = load_binary_stl(SRC) * MM
    tris[:, :, 2] += Z_SHIFT_M

    save_binary_stl(os.path.join(OUT_DIR, "zd550_frame.stl"), tris)
    save_binary_stl(os.path.join(OUT_DIR, "zd550_body_collision.stl"),
                    convex_hull_tris(tris.reshape(-1, 3)[tris.reshape(-1, 3)[:, 2] > -0.015 + Z_SHIFT_M]))

    v = tris.reshape(-1, 3)
    print("visual bounds x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]"
          % (v[:, 0].min(), v[:, 0].max(), v[:, 1].min(), v[:, 1].max(), v[:, 2].min(), v[:, 2].max()))


if __name__ == "__main__":
    main()
