#!/usr/bin/env python3
"""
Prototype: generate a top-down world photo + occupancy-map preview per maze.

For every <world>.yaml under maps/, this:
  - reads origin/resolution + the .pgm's dimensions to get the maze's
    world-frame bounding-box center (no hardcoded per-world numbers),
  - launches <world>.world (and <world>_victimas.world, if present) headed,
    moves the GUI camera to a centered top-down pose over that bounding box,
    and saves a screenshot via Gazebo's own /gui/screenshot service,
  - converts the .pgm itself into a legible PNG (nearest-neighbor upscale,
    rotated to match the screenshot's orientation, thin border).

Run manually after adding a map: `python3 scripts/generate_map_previews.py`
from yahboom_rosmaster_gazebo/, with a Gazebo Fortress GUI available (a real
or virtual X display). Needs no ROS build -- it drives Gazebo purely through
its own /gui/move_to/pose and /gui/screenshot services.
"""
import os
import shutil
import subprocess
import sys
import time

import yaml

PKG_GZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(PKG_GZ)
MAPS_DIR = os.path.join(PKG_GZ, "maps")
WORLDS_DIR = os.path.join(PKG_GZ, "worlds")
OUT_DIR = os.path.join(REPO, "docs", "media", "maps")

# Map basenames that don't match their world file 1:1 -- extend this if a
# future map/world pair is named differently.
WORLD_NAME_OVERRIDES = {
    "cafe_world_map": "cafe",
}

# cafe_world_map.pgm's current capture is too noisy to preview as-is;
# recapture it (see the README/CONTRIBUTING note asking for a fresh one)
# before removing it from this list.
SKIP_MAPS = {"cafe_world_map"}

CAMERA_Z = 6.0
SETTLE_SECONDS = 12.0
SERVICE_TIMEOUT_MS = 5000
IGN = shutil.which("ign")


def pgm_dimensions(pgm_path):
    with open(pgm_path, "rb") as pgm_file:
        header = pgm_file.read(64)
    tokens = header.split()
    # tokens[0] is the magic number (P5); width/height follow.
    return int(tokens[1]), int(tokens[2])


def bounding_box_center(yaml_path):
    with open(yaml_path, encoding="utf-8") as yaml_file:
        doc = yaml.safe_load(yaml_file)
    origin_x, origin_y = doc["origin"][0], doc["origin"][1]
    resolution = doc["resolution"]
    pgm_path = os.path.join(os.path.dirname(yaml_path), doc["image"])
    width_px, height_px = pgm_dimensions(pgm_path)
    center_x = origin_x + (width_px * resolution) / 2.0
    center_y = origin_y + (height_px * resolution) / 2.0
    return center_x, center_y


def run_env():
    env = os.environ.copy()
    env["IGN_GAZEBO_RESOURCE_PATH"] = f"{PKG_GZ}/models:{PKG_GZ}/worlds"
    env["GZ_SIM_RESOURCE_PATH"] = f"{PKG_GZ}/models:{PKG_GZ}/worlds"
    env["DISPLAY"] = ":0"
    env["QT_QPA_PLATFORM"] = "xcb"
    return env


def ign_service(args, env, timeout=5):
    subprocess.run(["ign", "service", *args], env=env,
                   capture_output=True, timeout=timeout, check=False)


def screenshot_world(world_file, center_x, center_y, out_png):
    env = run_env()
    world_path = os.path.join(WORLDS_DIR, world_file)
    proc = subprocess.Popen(
        ["ruby", IGN, "gazebo", "-r", "-v", "1", world_path,
         "--force-version", "6"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        time.sleep(SETTLE_SECONDS)
        pose = (f"pose: {{position: {{x: {center_x}, y: {center_y}, "
                f"z: {CAMERA_Z}}} orientation: {{x: 0, y: 0.7071, z: 0, "
                f"w: 0.7071}}}}")
        ign_service(
            ["-s", "/gui/move_to/pose", "--reqtype", "ignition.msgs.GUICamera",
             "--reptype", "ignition.msgs.Boolean", "--timeout",
             str(SERVICE_TIMEOUT_MS), "--req", pose], env, timeout=8)
        time.sleep(2.0)
        os.makedirs(os.path.dirname(out_png), exist_ok=True)
        tmp_dir = os.path.dirname(out_png)
        before = set(os.listdir(tmp_dir)) if os.path.isdir(tmp_dir) else set()
        ign_service(
            ["-s", "/gui/screenshot", "--reqtype", "ignition.msgs.StringMsg",
             "--reptype", "ignition.msgs.Boolean", "--timeout",
             str(SERVICE_TIMEOUT_MS), "--req", f'data: "{tmp_dir}"'],
            env, timeout=8)
        time.sleep(1.5)
        after = set(os.listdir(tmp_dir))
        new_files = [f for f in (after - before) if f.endswith(".png")]
        if not new_files:
            print(f"  WARNING: no screenshot produced for {world_file}")
            return False
        os.replace(os.path.join(tmp_dir, new_files[0]), out_png)
        return True
    finally:
        ign_service(
            ["-s", "/server_control", "--reqtype",
             "ignition.msgs.ServerControl", "--reptype",
             "ignition.msgs.Boolean", "--timeout",
             str(SERVICE_TIMEOUT_MS), "--req", "stop: true"], env, timeout=8)
        time.sleep(2.0)
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()


def convert_occupancy_map(pgm_path, out_png):
    os.makedirs(os.path.dirname(out_png), exist_ok=True)
    subprocess.run(
        ["convert", pgm_path, "-filter", "point", "-resize", "600%",
         "-rotate", "-90", "-bordercolor", "#999999", "-border", "2",
         out_png],
        check=True)


def main():
    if IGN is None:
        sys.exit("'ign' executable not found")

    yaml_files = sorted(
        f for f in os.listdir(MAPS_DIR) if f.endswith(".yaml"))
    for yaml_name in yaml_files:
        map_name = os.path.splitext(yaml_name)[0]
        if map_name in SKIP_MAPS:
            print(f"== {map_name} == skipped (see SKIP_MAPS)")
            continue

        world_name = WORLD_NAME_OVERRIDES.get(map_name, map_name)
        yaml_path = os.path.join(MAPS_DIR, yaml_name)
        print(f"== {map_name} ==")

        center_x, center_y = bounding_box_center(yaml_path)
        print(f"  bounding-box center: ({center_x:.3f}, {center_y:.3f})")

        pgm_path = os.path.join(MAPS_DIR, yaml.safe_load(
            open(yaml_path, encoding="utf-8"))["image"])
        convert_occupancy_map(
            pgm_path, os.path.join(OUT_DIR, f"{map_name}_occupancy.png"))
        print("  occupancy preview: done")

        base_world_file = f"{world_name}.world"
        if os.path.exists(os.path.join(WORLDS_DIR, base_world_file)):
            ok = screenshot_world(
                base_world_file, center_x, center_y,
                os.path.join(OUT_DIR, f"{world_name}.png"))
            print(f"  base world screenshot: {'done' if ok else 'FAILED'}")
        else:
            print(f"  WARNING: {base_world_file} not found, skipping")

        victimas_world_file = f"{world_name}_victimas.world"
        if os.path.exists(os.path.join(WORLDS_DIR, victimas_world_file)):
            ok = screenshot_world(
                victimas_world_file, center_x, center_y,
                os.path.join(OUT_DIR, f"{world_name}_victimas.png"))
            print(f"  victimas world screenshot: {'done' if ok else 'FAILED'}")
        else:
            print(f"  no victimas variant for {world_name}")


if __name__ == "__main__":
    main()
