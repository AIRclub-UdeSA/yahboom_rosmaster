#!/usr/bin/env python3
"""Generate deterministic, link-local OBJ visuals from the assembled CAD GLB."""

from __future__ import annotations

import argparse
import hashlib
import math
import shutil
import tempfile
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh


PACKAGE_DIR = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = PACKAGE_DIR / "cad" / "rosmaster_unified.glb"
DEFAULT_OUTPUT = PACKAGE_DIR / "meshes" / "rosmaster_x3" / "cad_visual"
SOURCE_SHA256 = "5db7240c8e8ea715ac7a30f89547ab6e338cacadd650b0c93eff36f601a9446c"
EXPECTED_NODE_COUNT = 358
EXPECTED_TRIANGLE_COUNT = 421_443
MAX_GENERATED_BYTES = 30_000_000
# Gazebo and RViz import OBJ through Assimp without joining identical vertices,
# expanding each triangle to three vertices. Keep every imported group below
# the 65,535-vertex limit while preserving the nine conceptual link/material
# sets and every source triangle.
MAX_TRIANGLES_PER_RENDER_GROUP = 20_000
EXPECTED_RENDER_GROUP_COUNT = 27
MTL_FILENAME = "rosmaster_cad.mtl"

# PBR values are retained here both as source-contract checks and as the input
# to the closest portable approximation available in Wavefront MTL.
MATERIALS = {
    "Standoff_Brass": ((235, 191, 46, 255), 0.90, 0.25),
    "Motor_Steel": ((173, 189, 204, 255), 0.92, 0.22),
    "Wheel_Carbon": ((28, 33, 31, 255), 0.45, 0.60),
    "Chassis_Green": ((61, 219, 122, 255), 0.82, 0.28),
    "Lidar_Black": ((20, 23, 20, 255), 0.70, 0.35),
    "Camera_Astra": ((26, 31, 28, 255), 0.80, 0.32),
}

OUTPUT_FILES = {
    "base": "rosmaster_base.obj",
    "front_left": "front_left_wheel.obj",
    "front_right": "front_right_wheel.obj",
    "back_left": "back_left_wheel.obj",
    "back_right": "back_right_wheel.obj",
    "lidar": "lidar.obj",
    "camera": "camera.obj",
}

EXPECTED_PARTITION_NODES = {
    "base": 208,
    "front_left": 37,
    "front_right": 37,
    "back_left": 37,
    "back_right": 37,
    "lidar": 1,
    "camera": 1,
}

EXPECTED_PARTITION_TRIANGLES = {
    "base": 165_280,
    "front_left": 24_066,
    "front_right": 24_072,
    "back_left": 24_034,
    "back_right": 24_090,
    "lidar": 154_549,
    "camera": 5_352,
}

EXPECTED_MATERIAL_NODE_COUNTS = {
    "base": Counter({"Motor_Steel": 183, "Standoff_Brass": 22, "Chassis_Green": 3}),
    "front_left": Counter({"Wheel_Carbon": 37}),
    "front_right": Counter({"Wheel_Carbon": 37}),
    "back_left": Counter({"Wheel_Carbon": 37}),
    "back_right": Counter({"Wheel_Carbon": 37}),
    "lidar": Counter({"Lidar_Black": 1}),
    "camera": Counter({"Camera_Astra": 1}),
}

EXPECTED_BOUNDS = {
    "base": ((-0.110982, -0.069001, -0.050744), (0.116520, 0.069001, 0.072002)),
    "front_left": ((-0.032240, -0.017601, -0.032196), (0.032240, 0.017601, 0.032196)),
    "front_right": ((-0.032197, -0.017601, -0.032240), (0.032197, 0.017601, 0.032240)),
    "back_left": ((-0.032222, -0.017601, -0.032249), (0.032222, 0.017601, 0.032249)),
    "back_right": ((-0.032292, -0.017601, -0.032249), (0.032292, 0.017601, 0.032249)),
    "lidar": ((-0.039479, -0.035132, -0.041243), (0.058013, 0.036647, 0.015802)),
    "camera": ((-0.098967, -0.082216, -0.043243), (-0.038773, 0.081987, 0.009424)),
}

# Link origins relative to base_link. These are deliberately duplicated from
# xacro so visual baking cannot silently move a functional sensor frame.
SENSOR_LINK_POSES = {
    "camera": (np.array([0.105, 0.0, 0.050]), 0.0),
    "lidar": (np.array([0.043, 0.0, 0.110]), math.pi),
}


@dataclass(frozen=True)
class Part:
    name: str
    material: str
    vertices: np.ndarray
    normals: np.ndarray
    faces: np.ndarray


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _fail(message: str) -> None:
    raise RuntimeError(message)


def _partition(name: str, bounds: np.ndarray) -> str:
    if name == "lidar_sensor":
        return "lidar"
    if name == "rgbd_camera":
        return "camera"
    if name.startswith("wheel_"):
        center = bounds.mean(axis=0)
        longitudinal = "front" if center[2] >= 0.0 else "back"
        lateral = "left" if center[0] >= 0.0 else "right"
        return f"{longitudinal}_{lateral}"
    if name.startswith(("part_", "plate_", "standoff_")):
        return "base"
    _fail(f"unrecognised GLB node: {name}")


def _load_and_validate(source: Path) -> dict[str, list[Part]]:
    if _sha256(source) != SOURCE_SHA256:
        _fail(f"source hash does not match {SOURCE_SHA256}: {source}")

    scene = trimesh.load_scene(source)
    nodes = sorted(scene.graph.nodes_geometry)
    if len(nodes) != EXPECTED_NODE_COUNT or len(scene.geometry) != EXPECTED_NODE_COUNT:
        _fail(
            f"expected {EXPECTED_NODE_COUNT} one-mesh nodes, got "
            f"{len(nodes)} nodes and {len(scene.geometry)} meshes"
        )

    parts: dict[str, list[Part]] = defaultdict(list)
    source_materials: dict[str, tuple[tuple[int, ...], float, float]] = {}
    triangle_count = 0
    for node in nodes:
        transform, geometry_name = scene.graph.get(node)
        mesh = scene.geometry[geometry_name].copy()
        mesh.apply_transform(transform)
        vertices = np.asarray(mesh.vertices, dtype=np.float64)
        normals = np.asarray(mesh.vertex_normals, dtype=np.float64)
        faces = np.asarray(mesh.faces, dtype=np.int64)
        if vertices.ndim != 2 or vertices.shape[1] != 3 or faces.ndim != 2 or faces.shape[1] != 3:
            _fail(f"{node} is not a finite triangular mesh")
        if normals.shape != vertices.shape:
            _fail(f"{node} has invalid vertex normals")
        if not np.isfinite(vertices).all() or not np.isfinite(normals).all():
            _fail(f"{node} contains non-finite geometry")
        material = mesh.visual.material
        material_name = material.name
        if material_name not in MATERIALS:
            _fail(f"{node} uses unexpected material {material_name!r}")
        source_materials[material_name] = (
            tuple(int(value) for value in material.baseColorFactor),
            float(material.metallicFactor),
            float(material.roughnessFactor),
        )
        key = _partition(node, mesh.bounds)
        parts[key].append(Part(node, material_name, vertices, normals, faces))
        triangle_count += len(faces)

    if triangle_count != EXPECTED_TRIANGLE_COUNT:
        _fail(f"expected {EXPECTED_TRIANGLE_COUNT} triangles, got {triangle_count}")
    if source_materials.keys() != MATERIALS.keys():
        _fail(f"source materials changed: {sorted(source_materials)}")
    for name, expected in MATERIALS.items():
        actual = source_materials[name]
        if actual[0] != expected[0] or not np.allclose(actual[1:], expected[1:], atol=1e-7):
            _fail(f"source material {name} changed: {actual}")
    for key, expected in EXPECTED_PARTITION_NODES.items():
        if len(parts[key]) != expected:
            _fail(f"expected {expected} {key} nodes, got {len(parts[key])}")
        triangles = sum(len(part.faces) for part in parts[key])
        if triangles != EXPECTED_PARTITION_TRIANGLES[key]:
            _fail(f"expected {EXPECTED_PARTITION_TRIANGLES[key]} {key} triangles, got {triangles}")
        materials = Counter(part.material for part in parts[key])
        if materials != EXPECTED_MATERIAL_NODE_COUNTS[key]:
            _fail(f"unexpected {key} material assignment: {materials}")
    return parts


def _cad_to_ros(vertices: np.ndarray) -> np.ndarray:
    """Map CAD millimetres to ROS metres: ROS (x,y,z) = CAD (z,x,y)."""
    return vertices[:, [2, 0, 1]] * 0.001


def _cad_normal_to_ros(normals: np.ndarray) -> np.ndarray:
    """Apply the rotational part of the CAD-to-ROS mapping to normals."""
    return normals[:, [2, 0, 1]]


def _build_groups(
    parts: dict[str, list[Part]],
) -> dict[
    str,
    dict[str, list[tuple[np.ndarray, np.ndarray, np.ndarray]]],
]:
    wheel_keys = ("front_left", "front_right", "back_left", "back_right")
    wheel_centers = {}
    for key in wheel_keys:
        vertices = np.vstack([part.vertices for part in parts[key]])
        wheel_centers[key] = vertices.min(axis=0) / 2.0 + vertices.max(axis=0) / 2.0

    # Align the CAD assembly's mean wheel centre with the model's wheel plane.
    # Individual wheel visuals can then be placed without changing joint or
    # collision geometry.
    cad_wheel_mean = np.mean(np.stack(list(wheel_centers.values())), axis=0)
    assembly_offset = np.array([0.0, 0.0, -0.0325]) - _cad_to_ros(cad_wheel_mean[None, :])[0]

    groups: dict[
        str,
        dict[str, list[tuple[np.ndarray, np.ndarray, np.ndarray]]],
    ] = defaultdict(lambda: defaultdict(list))
    for key, key_parts in parts.items():
        for part in key_parts:
            vertices = part.vertices.copy()
            local_normals = _cad_normal_to_ros(part.normals)
            if key in wheel_keys:
                local_vertices = _cad_to_ros(vertices - wheel_centers[key])
            else:
                # The website shifts the camera 10 mm rearward in CAD z before
                # applying the same cyclic CAD-to-ROS coordinate mapping.
                if key == "camera":
                    vertices[:, 2] -= 10.0
                local_vertices = _cad_to_ros(vertices) + assembly_offset
                if key in SENSOR_LINK_POSES:
                    translation, yaw = SENSOR_LINK_POSES[key]
                    cosine, sine = math.cos(yaw), math.sin(yaw)
                    rotation = np.array(
                        (
                            (cosine, -sine, 0.0),
                            (sine, cosine, 0.0),
                            (0.0, 0.0, 1.0),
                        )
                    )
                    # Row-vector form of p_link = R_base_link^T (p_base - t).
                    local_vertices = (local_vertices - translation) @ rotation
                    local_normals = local_normals @ rotation
            lengths = np.linalg.norm(local_normals, axis=1)
            # The source contains a handful of zero-area CAD triangles whose
            # averaged vertex normal is undefined. They carry no visible area;
            # give those vertices a deterministic finite fallback normal.
            missing_normals = lengths < 1e-12
            local_normals[missing_normals] = (0.0, 0.0, 1.0)
            lengths[missing_normals] = 1.0
            local_normals = local_normals / lengths[:, None]
            if not np.isfinite(local_vertices).all() or not np.isfinite(
                local_normals
            ).all():
                _fail(f"non-finite transformed geometry in {part.name}")
            groups[key][part.material].append(
                (local_vertices, local_normals, part.faces)
            )

    for key, expected in EXPECTED_BOUNDS.items():
        vertices = np.vstack(
            [
                vertices
                for meshes in groups[key].values()
                for vertices, _, _ in meshes
            ]
        )
        actual = np.stack((vertices.min(axis=0), vertices.max(axis=0)))
        if not np.allclose(actual, np.asarray(expected), atol=3e-6):
            _fail(f"{key} bounds changed: {actual.tolist()}")
    if sum(len(materials) for materials in groups.values()) != 9:
        _fail("expected exactly nine link/material render groups")
    return groups


def _write_mtl(path: Path) -> None:
    lines = ["# Generated by tools/convert_rosmaster_cad.py; do not edit."]
    for name, (rgba, metallic, roughness) in MATERIALS.items():
        base = np.asarray(rgba[:3], dtype=float) / 255.0
        specular = 0.04 * (1.0 - metallic) + base * metallic
        shininess = min(1000.0, max(1.0, 2.0 / (roughness ** 4) - 2.0))
        lines.extend((
            "",
            f"newmtl {name}",
            f"Ka {base[0] * 0.2:.8f} {base[1] * 0.2:.8f} {base[2] * 0.2:.8f}",
            f"Kd {base[0]:.8f} {base[1]:.8f} {base[2]:.8f}",
            f"Ks {specular[0]:.8f} {specular[1]:.8f} {specular[2]:.8f}",
            f"Ns {shininess:.8f}",
            f"d {rgba[3] / 255.0:.8f}",
            "illum 2",
        ))
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def _write_obj(
    path: Path,
    key: str,
    material_groups: dict[
        str, list[tuple[np.ndarray, np.ndarray, np.ndarray]]
    ],
) -> None:
    lines = [
        "# Generated by tools/convert_rosmaster_cad.py; do not edit.",
        f"mtllib {MTL_FILENAME}",
        f"o rosmaster_{key}",
    ]
    vertex_offset = 0
    for material in MATERIALS:
        meshes = material_groups.get(material)
        if not meshes:
            continue
        material_group = 0
        triangles_in_group = MAX_TRIANGLES_PER_RENDER_GROUP
        for vertices, normals, faces in meshes:
            lines.extend(
                f"v {x:.6f} {y:.6f} {z:.6f}"
                for x, y, z in vertices
            )
            lines.extend(
                f"vn {x:.5f} {y:.5f} {z:.5f}"
                for x, y, z in normals
            )
            one_based = faces + vertex_offset + 1
            face_offset = 0
            while face_offset < len(one_based):
                if triangles_in_group == MAX_TRIANGLES_PER_RENDER_GROUP:
                    lines.extend((
                        f"g {key}__{material}__{material_group:02d}",
                        f"usemtl {material}",
                    ))
                    material_group += 1
                    triangles_in_group = 0
                face_count = min(
                    MAX_TRIANGLES_PER_RENDER_GROUP - triangles_in_group,
                    len(one_based) - face_offset,
                )
                chunk = one_based[face_offset:face_offset + face_count]
                lines.extend(
                    f"f {a}//{a} {b}//{b} {c}//{c}"
                    for a, b, c in chunk
                )
                face_offset += face_count
                triangles_in_group += face_count
            vertex_offset += len(vertices)
    path.write_text("\n".join(lines) + "\n", encoding="ascii")


def _validate_outputs(output: Path) -> None:
    declared_materials = set()
    for raw_line in (output / MTL_FILENAME).read_text(encoding="ascii").splitlines():
        fields = raw_line.split()
        if fields and fields[0] == "newmtl" and len(fields) == 2:
            declared_materials.add(fields[1])
    if declared_materials != set(MATERIALS):
        _fail(f"generated MTL materials are invalid: {sorted(declared_materials)}")

    render_groups = 0
    for key, filename in OUTPUT_FILES.items():
        vertex_count = 0
        normal_count = 0
        triangle_count = 0
        triangles_in_group = 0
        referenced_materials = set()
        mtl_refs = []
        obj_lines = (output / filename).read_text(
            encoding="ascii"
        ).splitlines()
        for line_number, raw_line in enumerate(obj_lines, 1):
            fields = raw_line.split()
            if not fields or fields[0].startswith("#"):
                continue
            if fields[0] == "mtllib":
                mtl_refs.append(fields[1])
            elif fields[0] == "g":
                if triangles_in_group > MAX_TRIANGLES_PER_RENDER_GROUP:
                    _fail(f"oversized render group in {filename}")
                render_groups += 1
                triangles_in_group = 0
            elif fields[0] == "usemtl":
                referenced_materials.add(fields[1])
            elif fields[0] == "v":
                valid = len(fields) == 4 and all(
                    math.isfinite(float(value)) for value in fields[1:]
                )
                if not valid:
                    _fail(f"invalid vertex in {filename}:{line_number}")
                vertex_count += 1
            elif fields[0] == "vn":
                valid = len(fields) == 4 and all(
                    math.isfinite(float(value)) for value in fields[1:]
                )
                if not valid:
                    _fail(f"invalid normal in {filename}:{line_number}")
                normal_count += 1
            elif fields[0] == "f":
                if len(fields) != 4:
                    _fail(f"non-triangle face in {filename}:{line_number}")
                pairs = [value.split("//") for value in fields[1:]]
                if any(len(pair) != 2 for pair in pairs):
                    _fail(f"missing normal index in {filename}:{line_number}")
                indices = [int(pair[0]) for pair in pairs]
                normal_indices = [int(pair[1]) for pair in pairs]
                if min(indices) < 1 or max(indices) > vertex_count:
                    _fail(f"invalid OBJ index in {filename}:{line_number}")
                if min(normal_indices) < 1 or max(normal_indices) > normal_count:
                    _fail(f"invalid normal index in {filename}:{line_number}")
                triangle_count += 1
                triangles_in_group += 1
        if mtl_refs != [MTL_FILENAME]:
            _fail(f"{filename} does not reference the shared MTL exactly once")
        if not referenced_materials or not referenced_materials <= declared_materials:
            _fail(f"{filename} references invalid materials: {referenced_materials}")
        if triangle_count != EXPECTED_PARTITION_TRIANGLES[key]:
            _fail(f"{filename} has {triangle_count} triangles")
        if normal_count != vertex_count:
            _fail(f"{filename} must have one indexed normal per vertex")
        if triangles_in_group > MAX_TRIANGLES_PER_RENDER_GROUP:
            _fail(f"oversized final render group in {filename}")
    if render_groups != EXPECTED_RENDER_GROUP_COUNT:
        _fail(
            f"expected {EXPECTED_RENDER_GROUP_COUNT} renderer-safe groups, "
            f"got {render_groups}"
        )
    generated = [output / MTL_FILENAME, *(output / name for name in OUTPUT_FILES.values())]
    generated_bytes = sum(path.stat().st_size for path in generated)
    if generated_bytes >= MAX_GENERATED_BYTES:
        _fail(
            f"generated assets use {generated_bytes} bytes; "
            f"budget is below {MAX_GENERATED_BYTES}"
        )


def _generate(source: Path, output: Path) -> None:
    parts = _load_and_validate(source)
    groups = _build_groups(parts)
    output.mkdir(parents=True, exist_ok=True)
    _write_mtl(output / MTL_FILENAME)
    for key, filename in OUTPUT_FILES.items():
        _write_obj(output / filename, key, groups[key])
    _validate_outputs(output)


def _check_committed(source: Path, committed: Path) -> None:
    with tempfile.TemporaryDirectory(prefix="rosmaster-cad-") as temporary:
        generated = Path(temporary)
        _generate(source, generated)
        expected_names = {MTL_FILENAME, *OUTPUT_FILES.values()}
        actual_names = {path.name for path in committed.iterdir() if path.is_file()}
        if actual_names != expected_names:
            _fail(f"committed generated files differ: {sorted(actual_names)}")
        for name in sorted(expected_names):
            if (generated / name).read_bytes() != (committed / name).read_bytes():
                _fail(f"committed asset is stale: {name}")
    _validate_outputs(committed)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--check",
        action="store_true",
        help="byte-compare regenerated and committed assets",
    )
    arguments = parser.parse_args()

    if arguments.check:
        _check_committed(arguments.source, arguments.output_dir)
        print("CAD source and committed generated assets are valid and reproducible.")
    else:
        # Generate into a sibling temporary directory, then replace individual
        # outputs only after every source and geometry contract has passed.
        with tempfile.TemporaryDirectory(
            prefix="rosmaster-cad-", dir=arguments.output_dir.parent
        ) as temporary:
            staged = Path(temporary)
            _generate(arguments.source, staged)
            arguments.output_dir.mkdir(parents=True, exist_ok=True)
            for path in staged.iterdir():
                shutil.copyfile(path, arguments.output_dir / path.name)
        _validate_outputs(arguments.output_dir)
        total = sum(
            path.stat().st_size
            for path in arguments.output_dir.iterdir()
            if path.is_file()
        )
        print(f"Generated seven OBJ files and one MTL ({total / 1_000_000:.2f} MB).")


if __name__ == "__main__":
    main()
