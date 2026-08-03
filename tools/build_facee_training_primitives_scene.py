#!/usr/bin/env python3
"""Build the repository FaceE MuJoCo scene with E0019/Isaac physics.

The released G1 MJCF contains visual mesh geometry but its default collision
and joint-dynamics contract is not the one used to train/evaluate the FaceE
E0019 policy.  This generator applies the same deterministic conversion used
by the authoritative CPU MuJoCo evaluator:

* 29 primitive colliders from ``main_nodex.urdf``;
* no robot self collision, while robot-floor/chair contacts remain enabled;
* Isaac g1_model_12 armatures and effort limits;
* zero joint damping/friction loss;
* MuJoCo Newton solver, 5 ms step, 50 iterations;
* the exact 0.50 x 0.45 x 0.41 m rounded backless-chair proxy.

The chair is deliberately parked below the floor.  The runtime moves it to the
selected catalog layout before the first ``mj_forward`` call.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = (
    REPO_ROOT
    / "gear_sonic/data/assets/robot_description/mjcf/g1_29dof_rev_1_0.xml"
)
DEFAULT_OUTPUT = (
    REPO_ROOT
    / "gear_sonic/data/assets/robot_description/mjcf/scene_facee_training_primitives.xml"
)
DEFAULT_TRAINING_URDF_CANDIDATES = (
    REPO_ROOT
    / "gear_sonic/data/assets/robot_description/urdf/g1/main_nodex.urdf",
    Path(
        "/workspace/fangs1@xiaopeng.com/workspace_fs/"
        "r2s_ego_physxv3_release_49125c/exp_s2r_sit_chair/external/GRAIL/"
        "imports/SONIC/gear_sonic/data/assets/robot_description/urdf/g1/"
        "main_nodex.urdf"
    ),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--training-urdf", type=Path)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def resolve_training_urdf(explicit: Path | None) -> Path:
    candidates = (explicit,) if explicit is not None else DEFAULT_TRAINING_URDF_CANDIDATES
    for candidate in candidates:
        if candidate is not None and candidate.is_file():
            return candidate.resolve()
    searched = "\n  ".join(str(path) for path in candidates if path is not None)
    raise FileNotFoundError(
        "main_nodex.urdf is required to reproduce the Isaac primitive collision "
        f"contract. Searched:\n  {searched}\nPass --training-urdf explicitly."
    )


def vector(raw: str | None, size: int) -> tuple[float, ...]:
    values = tuple(float(value) for value in (raw or "").split())
    if not values:
        return (0.0,) * size
    if len(values) != size:
        raise ValueError(f"expected {size} values, got {raw!r}")
    return values


def rpy_to_wxyz(rpy: tuple[float, float, float]) -> tuple[float, float, float, float]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def joint_dynamics(joint_name: str) -> tuple[float, float]:
    if any(token in joint_name for token in ("hip_pitch", "hip_roll", "knee")):
        return 0.025101925, 139.0
    if "hip_yaw" in joint_name or joint_name == "waist_yaw_joint":
        return 0.010177520, 88.0
    if "ankle_" in joint_name or joint_name in (
        "waist_roll_joint",
        "waist_pitch_joint",
    ):
        return 0.007219450, 50.0
    if "wrist_pitch" in joint_name or "wrist_yaw" in joint_name:
        return 0.00425, 5.0
    return 0.003609725, 25.0


def stool_mesh(
    depth: float = 0.45,
    width: float = 0.50,
    height: float = 0.41,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    samples = 64
    edge_inset = 0.025

    def signed_power(value: float) -> float:
        return math.copysign(abs(value) ** 0.28, value)

    layers = (
        (0.0, edge_inset),
        (edge_inset, 0.0),
        (height - edge_inset, 0.0),
        (height, edge_inset),
    )
    vertices: list[tuple[float, float, float]] = []
    for z_value, inset in layers:
        for index in range(samples):
            theta = 2.0 * math.pi * index / samples
            vertices.append(
                (
                    (depth * 0.5 - inset) * signed_power(math.cos(theta)),
                    (width * 0.5 - inset) * signed_power(math.sin(theta)),
                    z_value,
                )
            )
    bottom_center = len(vertices)
    vertices.append((0.0, 0.0, 0.0))
    top_center = len(vertices)
    vertices.append((0.0, 0.0, height))

    faces: list[tuple[int, int, int]] = []
    for layer in range(len(layers) - 1):
        for index in range(samples):
            nxt = (index + 1) % samples
            a = layer * samples + index
            b = layer * samples + nxt
            c = (layer + 1) * samples + nxt
            d = (layer + 1) * samples + index
            faces.extend(((a, b, c), (a, c, d)))
    top_ring = (len(layers) - 1) * samples
    for index in range(samples):
        nxt = (index + 1) % samples
        faces.append((bottom_center, nxt, index))
        faces.append((top_center, top_ring + index, top_ring + nxt))
    return vertices, faces


def build(source: Path, training_urdf: Path, output: Path) -> None:
    tree = ET.parse(source)
    root = tree.getroot()

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.SubElement(root, "compiler")
    # The output sits beside the source MJCF, so the original relative meshdir
    # remains portable inside the repository.
    compiler.set("meshdir", "../meshes/g1/")

    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        root.insert(1, option)
    option.set("timestep", "0.005000")
    option.set("solver", "Newton")
    option.set("iterations", "50")

    body_by_name = {body.get("name"): body for body in root.iter("body")}
    robot_worldbody = next(
        worldbody
        for worldbody in root.findall("worldbody")
        if worldbody.find("body[@name='pelvis']") is not None
    )
    for geom in robot_worldbody.iter("geom"):
        geom.set("contype", "0")
        geom.set("conaffinity", "0")

    collision_count = 0
    urdf_root = ET.parse(training_urdf).getroot()
    for link in urdf_root.findall("link"):
        link_name = link.get("name")
        body = body_by_name.get(link_name)
        if body is None:
            continue
        for collision_index, collision in enumerate(link.findall("collision")):
            geometry = collision.find("geometry")
            if geometry is None:
                continue
            attributes = {
                "name": f"isaac_collision_{link_name}_{collision_index}",
                "contype": "1",
                "conaffinity": "2",
                "density": "0",
                "friction": "1.0 0.005 0.0001",
                "rgba": "0.15 0.75 1.0 0.0",
            }
            origin = collision.find("origin")
            if origin is not None:
                xyz = vector(origin.get("xyz"), 3)
                rpy = vector(origin.get("rpy"), 3)
                attributes["pos"] = " ".join(f"{value:.9g}" for value in xyz)
                if any(abs(value) > 0.0 for value in rpy):
                    attributes["quat"] = " ".join(
                        f"{value:.9g}" for value in rpy_to_wxyz(rpy)
                    )
            sphere = geometry.find("sphere")
            cylinder = geometry.find("cylinder")
            box = geometry.find("box")
            if sphere is not None:
                attributes["type"] = "sphere"
                attributes["size"] = sphere.get("radius", "0")
            elif cylinder is not None:
                attributes["type"] = "capsule"
                attributes["size"] = (
                    f"{float(cylinder.get('radius', '0')):.9g} "
                    f"{0.5 * float(cylinder.get('length', '0')):.9g}"
                )
            elif box is not None:
                attributes["type"] = "box"
                attributes["size"] = " ".join(
                    f"{0.5 * value:.9g}" for value in vector(box.get("size"), 3)
                )
            else:
                continue
            ET.SubElement(body, "geom", attributes)
            collision_count += 1
    if collision_count != 29:
        raise ValueError(f"expected 29 Isaac primitive colliders, got {collision_count}")

    for joint in root.iter("joint"):
        name = joint.get("name", "")
        if not name or name == "floating_base_joint":
            continue
        armature, effort = joint_dynamics(name)
        joint.set("armature", f"{armature:.9g}")
        joint.set("damping", "0")
        joint.set("frictionloss", "0")
        joint.set("actuatorfrclimited", "true")
        joint.set("actuatorfrcrange", f"{-effort:.9g} {effort:.9g}")
    for motor in root.iter("motor"):
        _, effort = joint_dynamics(motor.get("joint", ""))
        motor.set("ctrllimited", "true")
        motor.set("ctrlrange", f"{-effort:.9g} {effort:.9g}")

    vertices, faces = stool_mesh()
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset,
        "mesh",
        {
            "name": "facee_backless_41cm_proxy",
            "vertex": " ".join(
                f"{value:.8g}" for vertex in vertices for value in vertex
            ),
            "face": " ".join(str(value) for face in faces for value in face),
        },
    )

    scene_worldbody = next(
        (
            worldbody
            for worldbody in root.findall("worldbody")
            if worldbody.find("geom[@name='floor']") is not None
        ),
        robot_worldbody,
    )
    chair = ET.Element("body", {"name": "facee_chair", "pos": "0 0 -2"})
    ET.SubElement(
        chair,
        "geom",
        {
            "name": "facee_chair_collision",
            "type": "mesh",
            "mesh": "facee_backless_41cm_proxy",
            "density": "0",
            "contype": "2",
            "conaffinity": "1",
            "friction": "1.0 0.005 0.0001",
            "rgba": "0.92 0.72 0.12 1",
        },
    )
    scene_worldbody.insert(0, chair)
    floor = scene_worldbody.find("geom[@name='floor']")
    if floor is None:
        raise ValueError("source MJCF has no floor geom")
    floor.set("contype", "2")
    floor.set("conaffinity", "1")
    floor.set("friction", "1.0 0.005 0.0001")

    visual = root.find("visual")
    if visual is not None:
        global_element = visual.find("global")
        if global_element is None:
            global_element = ET.SubElement(visual, "global")
        global_element.set("offwidth", "1280")
        global_element.set("offheight", "720")

    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="unicode")
    print(f"source={source.resolve()}")
    print(f"training_urdf={training_urdf.resolve()}")
    print(f"output={output.resolve()}")
    print("isaac_primitive_colliders=29")


def main() -> None:
    args = parse_args()
    build(args.source.resolve(), resolve_training_urdf(args.training_urdf), args.output.resolve())


if __name__ == "__main__":
    main()
