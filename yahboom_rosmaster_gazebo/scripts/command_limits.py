"""Load the command limits and saturate a velocity command to them."""

import math
from pathlib import Path

import yaml


PACKAGE_NAME = "yahboom_rosmaster_gazebo"
FILE_NAME = "command_limits.yaml"
SOURCE_PATH = Path(__file__).resolve().parents[1] / "config" / FILE_NAME
LIMIT_KEYS = ("linear_x_mps", "linear_y_mps", "angular_z_rad_s")


def default_path():
    """Return the installed limits file, or the source tree's without a workspace."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory(PACKAGE_NAME))
    except (ImportError, LookupError):
        return SOURCE_PATH
    return share / "config" / FILE_NAME


def load_command_limits(config_path):
    """
    Return (linear x, linear y, angular z) limits from the YAML file.

    Each value must carry a non-empty ``source``, and the file must hold exactly
    the keys in ``LIMIT_KEYS``.
    """
    with open(config_path, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    limits = document.get("limits") if isinstance(document, dict) else None
    if not isinstance(limits, dict) or set(limits) != set(LIMIT_KEYS):
        raise RuntimeError(
            f"{config_path}: `limits` must hold exactly {list(LIMIT_KEYS)}")
    values = []
    for key in LIMIT_KEYS:
        entry = limits[key]
        if not isinstance(entry, dict) or not str(entry.get("source", "")).strip():
            raise RuntimeError(f"command limit {key} needs a value and a source")
        value = entry.get("value")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RuntimeError(f"command limit {key} must be numeric, got {value!r}")
        if not math.isfinite(value) or value < 0.0:
            raise RuntimeError(
                f"command limit {key} must be finite and non-negative, got {value}")
        values.append(float(value))
    return tuple(values)


def clamp_command(vx, vy, wz, x_limit, y_limit, angular_limit):
    """
    Saturate each component of a command to its absolute limit.

    The physical driver clamps component by component, without scaling the
    vector, and answers any non-finite input with a full stop.
    """
    values = (vx, vy, wz, x_limit, y_limit, angular_limit)
    if not all(math.isfinite(value) for value in values):
        return 0.0, 0.0, 0.0
    return tuple(
        max(-limit, min(command, limit))
        for command, limit in zip((vx, vy, wz), (x_limit, y_limit, angular_limit)))
