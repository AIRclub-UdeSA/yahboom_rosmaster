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


def values_agree(simulator, physical, tolerance=0.0):
    """
    Return whether two ledger values agree, or None if they are not comparable.

    Numbers agree within the absolute tolerance, strings and booleans when
    equal, and lists and mappings when every element agrees.
    """
    if isinstance(simulator, bool) or isinstance(physical, bool):
        if type(simulator) is not type(physical):
            return None
        return simulator == physical
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

    def rate_bounds(self, topic):
        """Return the (minimum, maximum) sim-time rate a probe grades."""
        minimum, maximum = self.entry(f"topics.{topic}.rate_hz")["contract"]
        return float(minimum), float(maximum)

    def entries(self, node=None, prefix=""):
        """Yield (dotted key, entry) for every simulator parity entry."""
        if node is None:
            node = {
                key: value for key, value in self.data["simulator"].items()
                if key != "provenance"
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
            if agreement is not None and agreement != matches:
                errors.append(
                    f"{key}: matches_physical is {matches} but simulator "
                    f"{simulator!r} and physical {physical!r} "
                    f"{'agree' if agreement else 'differ'}")
        return errors
