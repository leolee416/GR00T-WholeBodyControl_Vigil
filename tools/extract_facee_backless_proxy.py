#!/usr/bin/env python3
"""Extract E0019's audited 41 cm backless collision proxy as a Wavefront OBJ."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "gear_sonic/data/robot_model/model_data/g1/meshes/facee_backless_41cm_proxy.obj"
)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    root = ET.parse(args.source_scene.resolve()).getroot()
    matches = [
        item
        for item in root.findall(".//mesh")
        if item.get("name") == "yellow_stool_41cm_mesh"
    ]
    if len(matches) != 1:
        raise ValueError("source scene must contain one yellow_stool_41cm_mesh")
    mesh = matches[0]
    vertices = np.fromstring(mesh.get("vertex", ""), sep=" ").reshape(-1, 3)
    faces = np.fromstring(mesh.get("face", ""), sep=" ", dtype=np.int64).reshape(-1, 3)
    if vertices.shape != (258, 3) or faces.shape != (512, 3):
        raise ValueError(
            f"unexpected E0019 proxy topology: {vertices.shape}, {faces.shape}"
        )
    np.testing.assert_allclose(vertices.min(axis=0), [-0.225, -0.25, 0.0])
    np.testing.assert_allclose(vertices.max(axis=0), [0.225, 0.25, 0.41])
    if faces.min() != 0 or faces.max() >= len(vertices):
        raise ValueError("E0019 proxy contains invalid face indices")

    lines = [
        "# FaceE E0019 audited backless 0.45 x 0.50 x 0.41 m collision proxy",
        f"# source_scene={args.source_scene.resolve()}",
    ]
    lines.extend(f"v {x:.9g} {y:.9g} {z:.9g}" for x, y, z in vertices)
    lines.extend(
        f"f {a + 1} {b + 1} {c + 1}" for a, b, c in faces
    )
    args.output.resolve().write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(args.output.resolve())


if __name__ == "__main__":
    main()
