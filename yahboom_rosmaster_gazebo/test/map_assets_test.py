#!/usr/bin/env python3
"""
Contract tests for the occupancy maps shipped under maps/.

Checks, without launching Gazebo or ROS, that every map's YAML points at a
valid image, that every map has a matching world, and that the map lines up
with that world. The alignment check rasterizes the world's collision boxes
(following nested model:// includes) at the LiDAR's scan height, slides the
map's occupied cells over them, and requires the best fit to sit at the
declared origin -- not shifted, rotated, or mirrored. "At the declared
origin" means within MAX_OFFSET_CELLS (0.1 m at 0.05 m/cell), so an origin
off by 0.2 m is always caught and one off by 0.1 m is not.
"""

from pathlib import Path
import math
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from real_robot_contract import RealRobotContract  # noqa: E402


PACKAGE = Path(__file__).resolve().parents[1]
MAPS_DIR = PACKAGE / "maps"
WORLDS_DIR = PACKAGE / "worlds"
MODELS_DIR = PACKAGE / "models"

# Map basenames that don't match their world file 1:1.
WORLD_NAME_OVERRIDES = {}

# A LiDAR-built map only shows geometry crossing the scan plane, so only
# collision boxes spanning this height are rasterized. Floor slabs end at
# z = 0 and the lowest walls (the mazes' panels) top out at 0.5 m.
LEDGER = RealRobotContract.load()
SCAN_HEIGHT_M = (
    LEDGER.nominal("frames.base_footprint_to_base_link_z_m")
    + LEDGER.nominal("frames.mounts.laser_link")["xyz"][2])

# An occupied map cell counts as "on a wall" within this distance of one.
WALL_TOLERANCE_M = 0.05

# The map is slid up to this many cells each way to find its best fit, which
# must land within MAX_OFFSET_CELLS of the declared origin. SLAM captures
# land within one cell (0.05 m); a wrong origin lands further out.
SEARCH_CELLS = 6
MAX_OFFSET_CELLS = 2

# The map as declared must fit better than any rotated or mirrored copy of
# it by this much. The shipped maps win by 0.14 or more.
MIN_ORIENTATION_MARGIN = 0.05

# Fraction of occupied cells that must land on a wall at the best fit. The
# mazes score 0.85 or more; the cafe scores 0.64 because its kitchen and
# counter exist only in the visual mesh (cafe.dae), with no collision:
# gpu_lidar renders visuals, so they show up in the map but not here.
MIN_ALIGNMENT = 0.5

REQUIRED_KEYS = (
    "image", "mode", "resolution", "origin", "negate",
    "occupied_thresh", "free_thresh",
)


def map_yamls():
    return sorted(MAPS_DIR.glob("*.yaml"))


def world_for(yaml_path):
    name = WORLD_NAME_OVERRIDES.get(yaml_path.stem, yaml_path.stem)
    return WORLDS_DIR / f"{name}.world"


def read_pgm(path):
    """Parse a binary (P5) 8-bit PGM into a uint8 array."""
    data = path.read_bytes()
    tokens = []
    position = 0
    while len(tokens) < 4:
        while data[position:position + 1].isspace():
            position += 1
        if data[position:position + 1] == b"#":
            position = data.index(b"\n", position) + 1
            continue
        start = position
        while not data[position:position + 1].isspace():
            position += 1
        tokens.append(data[start:position])
    magic, width, height, max_value = tokens
    if magic != b"P5":
        raise ValueError(f"{path.name}: expected a P5 PGM, got {magic!r}")
    width, height, max_value = int(width), int(height), int(max_value)
    if max_value > 255:
        raise ValueError(f"{path.name}: only 8-bit PGMs are supported")
    pixels = data[position + 1:position + 1 + width * height]
    if len(pixels) != width * height:
        raise ValueError(
            f"{path.name}: expected {width * height} pixels, "
            f"found {len(pixels)}")
    return np.frombuffer(pixels, dtype=np.uint8).reshape(height, width)


def load_map(yaml_path):
    with open(yaml_path, encoding="utf-8") as yaml_file:
        doc = yaml.safe_load(yaml_file)
    return doc, read_pgm(yaml_path.parent / doc["image"])


def occupied_cells(doc, grid):
    """Mask of cells map_server loads as occupied (trinary mode)."""
    occupancy = grid.astype(float) / 255.0
    if not doc["negate"]:
        occupancy = 1.0 - occupancy
    return occupancy > doc["occupied_thresh"]


def pose_matrix(text):
    """SDF pose 'x y z roll pitch yaw' as a 4x4 homogeneous transform."""
    values = [float(v) for v in (text or "").split()] or [0.0] * 6
    x, y, z, roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    matrix[:3, 3] = [x, y, z]
    return matrix


def child_pose(element):
    pose = element.find("pose")
    return pose_matrix(pose.text if pose is not None else None)


def included_model(uri):
    if not uri.startswith("model://"):
        return None
    path = MODELS_DIR / uri[len("model://"):] / "model.sdf"
    if not path.exists():
        # Skipping it would drop its walls and surface as a confusing
        # alignment failure; every mapped world's models live in models/.
        raise FileNotFoundError(f"{uri} not found: no {path}")
    return ET.parse(path).getroot().find("model")


def collision_boxes(model, parent):
    """Yield (transform, size) for every box collision under a model."""
    for link in model.findall("link"):
        link_pose = parent @ child_pose(link)
        for collision in link.findall("collision"):
            box = collision.find("geometry/box/size")
            if box is not None:
                size = np.array([float(v) for v in box.text.split()])
                yield link_pose @ child_pose(collision), size
    for nested in model.findall("model"):
        yield from collision_boxes(nested, parent @ child_pose(nested))
    for include in model.findall("include"):
        submodel = included_model(include.findtext("uri", "").strip())
        if submodel is None:
            continue
        # An <include> pose replaces the included model's own pose.
        if include.find("pose") is not None:
            pose = child_pose(include)
        else:
            pose = child_pose(submodel)
        yield from collision_boxes(submodel, parent @ pose)


def world_boxes(world_path):
    world = ET.parse(world_path).getroot().find("world")
    yield from collision_boxes(world, np.eye(4))


def convex_hull(points):
    """Counter-clockwise convex hull of 2D points (monotone chain)."""
    points = sorted(set(map(tuple, np.round(points, 9))))
    if len(points) <= 2:
        return points

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for point in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def wall_mask(world_path, doc, shape, tolerance):
    """Cells within `tolerance` of a collision box crossing the scan plane."""
    height, width = shape
    resolution = doc["resolution"]
    origin_x, origin_y = doc["origin"][0], doc["origin"][1]
    columns, rows = np.meshgrid(np.arange(width), np.arange(height))
    # Row 0 of the image is the map's top edge (largest y).
    cell_x = origin_x + (columns + 0.5) * resolution
    cell_y = origin_y + (height - 1 - rows + 0.5) * resolution
    mask = np.zeros(shape, dtype=bool)
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                      for sz in (-1, 1)])
    for transform, size in world_boxes(world_path):
        corners = (signs * size / 2.0) @ transform[:3, :3].T + transform[:3, 3]
        if not corners[:, 2].min() <= SCAN_HEIGHT_M <= corners[:, 2].max():
            continue
        hull = convex_hull(corners[:, :2])
        if len(hull) < 3:
            continue
        inside = np.ones(shape, dtype=bool)
        for (ax, ay), (bx, by) in zip(hull, hull[1:] + hull[:1]):
            edge = math.hypot(bx - ax, by - ay)
            # Signed distance to the edge, positive inside a CCW polygon.
            distance = ((bx - ax) * (cell_y - ay)
                        - (by - ay) * (cell_x - ax)) / edge
            inside &= distance >= -tolerance
        mask |= inside
    return mask


def shifted(mask, rows, columns):
    """Shift a mask by whole cells, filling the uncovered edge with False."""
    out = np.zeros_like(mask)
    height, width = mask.shape
    out[max(rows, 0):height + min(rows, 0),
        max(columns, 0):width + min(columns, 0)] = mask[
            max(-rows, 0):height + min(-rows, 0),
            max(-columns, 0):width + min(-columns, 0)]
    return out


def best_fit(occupied, walls):
    """(score, row shift, column shift) of the best-fitting shift."""
    total = occupied.sum()
    span = range(-SEARCH_CELLS, SEARCH_CELLS + 1)
    return max(
        ((shifted(occupied, rows, columns) & walls).sum() / total,
         rows, columns)
        for rows in span for columns in span)


def alignment(yaml_path):
    """Best fit of the map as declared, and of its rotated/mirrored copies."""
    doc, grid = load_map(yaml_path)
    occupied = occupied_cells(doc, grid)
    walls = wall_mask(world_for(yaml_path), doc, grid.shape, WALL_TOLERANCE_M)
    variants = {
        "rotated 180 degrees": np.rot90(occupied, 2),
        "mirrored left-right": occupied[:, ::-1],
        "mirrored top-bottom": occupied[::-1],
    }
    return best_fit(occupied, walls), {
        name: best_fit(variant, walls)[0]
        for name, variant in variants.items()}


class TestMapReferences(unittest.TestCase):
    """Every map YAML must resolve to a usable image and a real world."""

    def test_maps_exist(self):
        self.assertTrue(map_yamls(), "no maps found under maps/")

    def test_yaml_fields_and_image(self):
        for yaml_path in map_yamls():
            with self.subTest(map=yaml_path.name):
                with open(yaml_path, encoding="utf-8") as yaml_file:
                    doc = yaml.safe_load(yaml_file)
                missing = [key for key in REQUIRED_KEYS if key not in doc]
                self.assertFalse(missing, f"missing keys: {missing}")
                image = yaml_path.parent / doc["image"]
                self.assertTrue(image.exists(), f"{doc['image']} not found")
                grid = read_pgm(image)
                self.assertGreater(grid.size, 0)
                self.assertGreater(doc["resolution"], 0.0)
                self.assertEqual(len(doc["origin"]), 3)
                # The alignment check ignores yaw; a rotated origin would fail
                # it with a misleading message instead of here.
                self.assertEqual(doc["origin"][2], 0, "origin yaw must be 0")
                self.assertLess(doc["free_thresh"], doc["occupied_thresh"])

    def test_every_map_has_a_world(self):
        for yaml_path in map_yamls():
            with self.subTest(map=yaml_path.name):
                self.assertTrue(
                    world_for(yaml_path).exists(),
                    f"no world for {yaml_path.name}; add it to "
                    "WORLD_NAME_OVERRIDES if the names differ")

    def test_no_orphan_images(self):
        referenced = set()
        for yaml_path in map_yamls():
            with open(yaml_path, encoding="utf-8") as yaml_file:
                referenced.add(yaml.safe_load(yaml_file)["image"])
        orphans = sorted(
            p.name for p in MAPS_DIR.glob("*.pgm") if p.name not in referenced)
        self.assertFalse(orphans, f"images no map YAML points at: {orphans}")


class TestMapAlignment(unittest.TestCase):
    """Each map's occupied cells must land on its world's walls."""

    def test_maps_align_with_their_worlds(self):
        for yaml_path in map_yamls():
            with open(yaml_path, encoding="utf-8") as yaml_file:
                image = yaml_path.parent / yaml.safe_load(yaml_file)["image"]
            if not image.exists() or not world_for(yaml_path).exists():
                continue  # TestMapReferences reports these.
            world = world_for(yaml_path).name
            try:
                (score, rows, columns), variants = alignment(yaml_path)
            except FileNotFoundError as error:
                with self.subTest(map=yaml_path.name, check="models"):
                    self.fail(f"{world} includes a missing model: {error}")
                continue
            with self.subTest(map=yaml_path.name, check="overlap"):
                self.assertGreaterEqual(
                    score, MIN_ALIGNMENT,
                    f"only {score:.0%} of {yaml_path.name}'s occupied cells "
                    f"land on a wall of {world} even at the best shift; was "
                    "it captured from that world?")
            with self.subTest(map=yaml_path.name, check="origin"):
                self.assertLessEqual(
                    max(abs(rows), abs(columns)), MAX_OFFSET_CELLS,
                    f"{yaml_path.name} fits {world} best shifted "
                    f"{rows} rows and {columns} columns; fix its origin")
            with self.subTest(map=yaml_path.name, check="orientation"):
                for name, variant_score in variants.items():
                    self.assertGreaterEqual(
                        score - variant_score, MIN_ORIENTATION_MARGIN,
                        f"{yaml_path.name} fits {world} about as well "
                        f"{name} ({variant_score:.0%} vs {score:.0%}); "
                        "check the capture's orientation")


if __name__ == "__main__":
    unittest.main()
