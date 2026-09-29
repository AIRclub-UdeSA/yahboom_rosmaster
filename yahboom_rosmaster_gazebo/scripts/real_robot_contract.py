"""Read the simulator/physical parity ledger in config/real_robot_contract.yaml."""

from pathlib import Path

import yaml


PACKAGE_NAME = "yahboom_rosmaster_gazebo"
FILE_NAME = "real_robot_contract.yaml"
SOURCE_PATH = Path(__file__).resolve().parents[1] / "config" / FILE_NAME
ENTRY_FIELDS = {
    "nominal",
    "measured",
    "contract",
    "matches_physical",
    "closes_in_step",
    "tolerance",
    "note",
}
# Steps of yahboom_rosmaster#43 that can still close a gap.
OPEN_STEPS = range(4, 10)
OUT_OF_SCOPE = "out_of_scope"
# Absorbs binary rounding, so a difference equal to the tolerance agrees.
ROUNDING_SLACK = 1e-12
# Simulator-only settings with no physical counterpart, so no parity entry.
SETTINGS_GROUP = "render_settings"
SETTING_FIELDS = {"nominal", "note"}


def default_path():
    """Return the installed ledger, or the source tree's without a workspace."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory(PACKAGE_NAME))
    except (ImportError, LookupError):
        return SOURCE_PATH
    return share / "config" / FILE_NAME


def is_entry(node):
    """Return whether a simulator node is a parity entry rather than a group."""
    return isinstance(node, dict) and "matches_physical" in node


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def is_range(value):
    """
    Return whether a physical value is a measured range, ``{min: low, max: high}``.

    A range is a mapping of exactly those two numbers, in order. A bare
    two-element list is not one: lists compare element by element.
    """
    return (
        isinstance(value, dict) and set(value) == {"min", "max"}
        and _is_number(value["min"]) and _is_number(value["max"])
        and value["min"] <= value["max"])


def values_agree(simulator, physical, tolerance=0.0):
    """
    Return whether two ledger values agree, or None if they are not comparable.

    Numbers agree within the absolute tolerance, strings and booleans when
    equal, and lists and mappings when every element agrees. A physical
    ``{min, max}`` range (see ``is_range``) agrees with a simulator number that
    lies inside it, widened by the tolerance at both ends.
    """
    if isinstance(simulator, bool) or isinstance(physical, bool):
        if type(simulator) is not type(physical):
            return None
        return simulator == physical
    if _is_number(simulator) and is_range(physical):
        slack = tolerance + ROUNDING_SLACK
        return physical["min"] - slack <= simulator <= physical["max"] + slack
    if isinstance(simulator, (int, float)) and isinstance(physical, (int, float)):
        return abs(simulator - physical) <= tolerance + ROUNDING_SLACK
    if isinstance(simulator, str) and isinstance(physical, str):
        return simulator == physical
    if isinstance(simulator, list) and isinstance(physical, list):
        if len(simulator) != len(physical):
            return False
        results = [
            values_agree(sim_value, physical_value, tolerance)
            for sim_value, physical_value in zip(simulator, physical)
        ]
        return None if None in results else all(results)
    if isinstance(simulator, dict) and isinstance(physical, dict):
        if set(simulator) != set(physical):
            return None
        results = [
            values_agree(simulator[key], physical[key], tolerance)
            for key in simulator
        ]
        return None if None in results else all(results)
    return None


class RealRobotContract:
    """Look up physical reference values and simulator parity entries."""

    def __init__(self, data):
        self.data = data

    @classmethod
    def load(cls, path=None):
        """Parse the ledger from a path, defaulting to the installed copy."""
        with open(path or default_path(), encoding="utf-8") as stream:
            return cls(yaml.safe_load(stream))

    def _lookup(self, section, key):
        node = self.data[section]
        for part in key.split("."):
            node = node[part]
        return node

    def physical(self, key):
        """Return the physical value at a dotted key such as 'camera.width'."""
        return self._lookup("physical", key)

    def entry(self, key):
        """Return the simulator parity entry at a dotted key."""
        node = self._lookup("simulator", key)
        if not is_entry(node):
            raise KeyError(f"simulator.{key} is not a parity entry")
        return node

    def nominal(self, key):
        """Return the simulator's configured value at a dotted key."""
        return self.entry(key)["nominal"]

    def setting(self, key):
        """
        Return a simulator-only setting at a dotted key of ``render_settings``.

        These are values the simulator is configured with that have no physical
        counterpart, such as the camera's clip planes, so they are not parity
        entries. ``settings_errors`` validates them.
        """
        return self._lookup("simulator", f"{SETTINGS_GROUP}.{key}")["nominal"]

    def rate_bounds(self, topic):
        """Return the (minimum, maximum) sim-time rate a probe grades."""
        minimum, maximum = self.entry(f"topics.{topic}.rate_hz")["contract"]
        return float(minimum), float(maximum)

    def entries(self, node=None, prefix=""):
        """Yield (dotted key, entry) for every simulator parity entry."""
        if node is None:
            node = {
                key: value for key, value in self.data["simulator"].items()
                if key not in ("provenance", SETTINGS_GROUP)
            }
        for key, value in node.items():
            dotted = f"{prefix}.{key}" if prefix else key
            if is_entry(value):
                yield dotted, value
            elif isinstance(value, dict):
                yield from self.entries(value, dotted)

    def parity_errors(self):
        """Return every inconsistency between the parity entries and physical."""
        errors = []
        for key, entry in self.entries():
            unknown = set(entry) - ENTRY_FIELDS
            if unknown:
                errors.append(f"{key}: unknown fields {sorted(unknown)}")
            if "nominal" not in entry and "measured" not in entry:
                errors.append(f"{key}: needs a nominal or measured value")
            matches = entry["matches_physical"]
            step = entry.get("closes_in_step")
            if not isinstance(matches, bool):
                errors.append(f"{key}: matches_physical must be true or false")
            elif matches and step is not None:
                errors.append(f"{key}: matches physical but names a step")
            elif not matches and step not in (*OPEN_STEPS, OUT_OF_SCOPE):
                errors.append(
                    f"{key}: a gap needs closes_in_step 4-9 or "
                    f"{OUT_OF_SCOPE}, got {step!r}")
            if step == OUT_OF_SCOPE and not entry.get("note"):
                errors.append(f"{key}: {OUT_OF_SCOPE} needs a note")
            try:
                physical = self.physical(key)
            except (KeyError, TypeError):
                errors.append(f"{key}: no physical value at the same key")
                continue
            simulator = entry.get("nominal", entry.get("measured"))
            agreement = values_agree(
                simulator, physical, float(entry.get("tolerance", 0.0)))
            if agreement is None:
                if matches is True:
                    errors.append(
                        f"{key}: matches_physical is true but simulator "
                        f"{simulator!r} and physical {physical!r} are not "
                        f"comparable, so nothing verifies it; make the "
                        f"values comparable, or set matches_physical to "
                        f"false with a closes_in_step")
            elif agreement != matches:
                errors.append(
                    f"{key}: matches_physical is {matches} but simulator "
                    f"{simulator!r} and physical {physical!r} "
                    f"{'agree' if agreement else 'differ'}")
        return errors

    def settings_errors(self):
        """Return every problem in the simulator's ``render_settings`` group."""
        settings = self.data["simulator"].get(SETTINGS_GROUP)
        if not isinstance(settings, dict) or not settings:
            return [f"simulator.{SETTINGS_GROUP} must be a non-empty mapping"]
        errors = []
        for name, setting in settings.items():
            where = f"{SETTINGS_GROUP}.{name}"
            if not isinstance(setting, dict) or "nominal" not in setting:
                errors.append(f"{where}: needs a mapping with a nominal value")
                continue
            unknown = set(setting) - SETTING_FIELDS
            if unknown:
                errors.append(f"{where}: unknown fields {sorted(unknown)}")
            if not _is_number(setting["nominal"]) or not setting["nominal"] > 0.0:
                errors.append(f"{where}: nominal must be a positive number")
            if "matches_physical" in setting:
                errors.append(f"{where}: a setting is not a parity entry")
            if self._has_physical(name):
                errors.append(
                    f"{where}: has a physical counterpart, so it belongs in "
                    "the parity entries")
        return errors

    def _has_physical(self, name):
        """Return whether the physical section has a key of this name."""
        return any(
            name in group for group in self.data["physical"].values()
            if isinstance(group, dict))
