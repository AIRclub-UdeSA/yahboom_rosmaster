"""Load and validate the sensor profiles in config/sensor_profiles.yaml."""

import math
from pathlib import Path

import yaml


PACKAGE_NAME = "yahboom_rosmaster_gazebo"
FILE_NAME = "sensor_profiles.yaml"
SOURCE_PATH = Path(__file__).resolve().parents[1] / "config" / FILE_NAME
PROFILE_NAMES = ("ideal", "physical")
# The keys every profile of a sensor must carry. Step 8 of #43 adds the LiDAR
# (8b) and the IMU (8c) here.
SENSOR_KEYS = {
    "point_cloud": ("frame_period_s", "latency_s", "gap_frames"),
    "depth": (
        "scale_error", "noise_sigma_floor_m", "noise_sigma_coefficient",
        "noise_sigma_exponent", "min_range_m"),
    "lidar": ("hold_scans", "scan_time_s"),
    "imu": ("gyro_noise_stddev_rad_s", "accel_noise_stddev_mps2"),
}
# The published gap probabilities carry four decimals and sum to 0.9998.
PROBABILITY_SUM_TOLERANCE = 1e-3


def default_path():
    """Return the installed profile file, or the source tree's without a workspace."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory(PACKAGE_NAME))
    except (ImportError, LookupError):
        return SOURCE_PATH
    return share / "config" / FILE_NAME


def _number(sensor, profile, key, value, minimum, exclusive=False):
    """Return a finite number no lower than ``minimum``."""
    where = f"sensor profile {sensor}.{profile}.{key}"
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RuntimeError(f"{where} must be numeric, got {value!r}")
    value = float(value)
    if not math.isfinite(value) or value < minimum or (exclusive and value == minimum):
        bound = "greater than" if exclusive else "at least"
        raise RuntimeError(f"{where} must be {bound} {minimum}, got {value}")
    return value


def _gap_frames(sensor, profile, key, value):
    """Return a {whole frames: probability} mapping that sums to about one."""
    where = f"sensor profile {sensor}.{profile}.{key}"
    if not isinstance(value, dict) or not value:
        raise RuntimeError(f"{where} must be a mapping of frames to probability")
    gaps = {}
    for frames, probability in value.items():
        if isinstance(frames, bool) or not isinstance(frames, int) or frames < 1:
            raise RuntimeError(f"{where} key {frames!r} must be a whole number >= 1")
        gaps[frames] = _number(sensor, profile, f"{key}.{frames}", probability, 0.0, True)
    total = sum(gaps.values())
    if abs(total - 1.0) > PROBABILITY_SUM_TOLERANCE:
        raise RuntimeError(f"{where} sums to {total}, not 1")
    return dict(sorted(gaps.items()))


def _hold_scans(sensor, profile, key, value):
    """Return how many scans the relay holds back: none, or one."""
    where = f"sensor profile {sensor}.{profile}.{key}"
    if isinstance(value, bool) or not isinstance(value, int) or value not in (0, 1):
        raise RuntimeError(f"{where} must be 0 or 1, got {value!r}")
    return value


def _per_axis(sensor, profile, key, value):
    """Return three finite, non-negative standard deviations, for x, y and z."""
    where = f"sensor profile {sensor}.{profile}.{key}"
    if not isinstance(value, list) or len(value) != 3:
        raise RuntimeError(f"{where} must be a list of three numbers (x, y, z)")
    return [
        _number(sensor, profile, f"{key}[{index}]", item, 0.0)
        for index, item in enumerate(value)]


def _validated_value(sensor, profile, key, value):
    if key in ("gyro_noise_stddev_rad_s", "accel_noise_stddev_mps2"):
        return _per_axis(sensor, profile, key, value)
    if key == "hold_scans":
        return _hold_scans(sensor, profile, key, value)
    if key == "scan_time_s":
        return _number(sensor, profile, key, value, 0.0, exclusive=True)
    if key == "gap_frames":
        return _gap_frames(sensor, profile, key, value)
    if key in ("frame_period_s", "noise_sigma_exponent"):
        return _number(sensor, profile, key, value, 0.0, exclusive=True)
    if key == "scale_error":
        # A fraction of the depth: -1 would read every distance as zero.
        return _number(sensor, profile, key, value, -1.0, exclusive=True)
    return _number(sensor, profile, key, value, 0.0)


def load_sensor_profile(config_path, sensor, profile_name):
    """
    Load and validate one profile of one sensor.

    Every value must carry a non-empty ``source`` next to its ``value``, and a
    profile must hold exactly its sensor's keys, no more and no fewer.
    Returns the plain values, without their sources.
    """
    with open(config_path, encoding="utf-8") as profile_file:
        document = yaml.safe_load(profile_file)

    if not isinstance(document, dict):
        raise RuntimeError(f"{config_path} is not a sensor profile document")
    if sensor not in SENSOR_KEYS:
        available = ", ".join(sorted(SENSOR_KEYS))
        raise RuntimeError(f"Unknown sensor '{sensor}'; available sensors: {available}")
    profiles = document.get(sensor)
    if not isinstance(profiles, dict):
        raise RuntimeError(f"{config_path} has no '{sensor}' profiles")
    if profile_name not in profiles:
        available = ", ".join(sorted(profiles)) or "none"
        raise RuntimeError(
            f"Unknown sensor profile '{profile_name}' for {sensor}; "
            f"available profiles: {available}")

    profile = profiles[profile_name]
    if not isinstance(profile, dict):
        raise RuntimeError(f"Invalid sensor profile '{sensor}.{profile_name}'")
    expected = SENSOR_KEYS[sensor]
    missing = [key for key in expected if key not in profile]
    extra = [key for key in profile if key not in expected]
    if missing or extra:
        raise RuntimeError(
            f"Invalid sensor profile '{sensor}.{profile_name}': "
            f"missing={missing}, extra={extra}")

    values = {}
    for key in expected:
        record = profile[key]
        if not isinstance(record, dict) or set(record) != {"value", "source"}:
            raise RuntimeError(
                f"Sensor profile '{sensor}.{profile_name}' value '{key}' must be "
                "a mapping with exactly 'value' and 'source'")
        source = record["source"]
        if not isinstance(source, str) or not source.strip():
            raise RuntimeError(
                f"Sensor profile '{sensor}.{profile_name}' value '{key}' "
                "needs a source")
        values[key] = _validated_value(sensor, profile_name, key, record["value"])
    return values


def point_cloud_parameters(profile):
    """Return the camera adapter's node parameters for a point_cloud profile."""
    gaps = profile["gap_frames"]
    return {
        "frame_period_s": profile["frame_period_s"],
        "latency_s": profile["latency_s"],
        "gap_frames": list(gaps),
        "gap_probabilities": list(gaps.values()),
    }


def depth_parameters(profile):
    """Return the camera adapter's node parameters for a depth profile."""
    return {
        "depth_scale_error": profile["scale_error"],
        "depth_noise_floor_m": profile["noise_sigma_floor_m"],
        "depth_noise_coefficient": profile["noise_sigma_coefficient"],
        "depth_noise_exponent": profile["noise_sigma_exponent"],
        "depth_min_range_m": profile["min_range_m"],
    }


def lidar_parameters(profile):
    """Return the scan relay's node parameters for a lidar profile."""
    return {
        "hold_scans": profile["hold_scans"],
        "scan_time": profile["scan_time_s"],
    }


def imu_xacro_arguments(profile):
    """
    Return the xacro arguments that set the IMU's per-axis noise, as name:=value.

    The launch file passes them when it expands the description, the way it passes
    the wheel-contact profile, so the description package reads no gazebo config.
    """
    arguments = {}
    for prefix, key in (
            ("imu_gyro_noise_stddev", "gyro_noise_stddev_rad_s"),
            ("imu_accel_noise_stddev", "accel_noise_stddev_mps2")):
        for axis, value in zip("xyz", profile[key]):
            arguments[f"{prefix}_{axis}"] = value
    return arguments
