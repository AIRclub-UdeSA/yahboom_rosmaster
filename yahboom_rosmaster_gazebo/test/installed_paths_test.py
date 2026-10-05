#!/usr/bin/env python3
"""
Keep installed code from locating its data through the source tree.

A ``--symlink-install`` build resolves every installed script back into the
source tree, so ``Path(__file__).parents[1] / "config"`` works there and nowhere
else (a copy install, an apt package). This test reads the installed scripts and
launch files without running them and rejects the patterns that only work in a
symlink build. Data is found through ``default_path()`` (the ament share
directory, with the source tree as a fallback for a bare checkout).
"""

import ast
from pathlib import Path
import unittest

PACKAGE_DIR = Path(__file__).resolve().parents[1]

# Modules that define a data file's SOURCE_PATH next to its default_path().
DEFINING_MODULES = {
    "command_limits.py",
    "real_robot_contract.py",
    "sensor_profiles.py",
}

# Maintainer tools that install with the scripts directory but are only
# meaningful in a source checkout: generate_map_previews.py writes into the
# repository's docs/ tree.
SOURCE_TREE_TOOLS = {"generate_map_previews.py"}


def installed_files():
    """Return the python files the package installs, minus source-tree tools."""
    files = sorted((PACKAGE_DIR / "scripts").glob("*.py"))
    files += sorted((PACKAGE_DIR / "launch").glob("*.py"))
    return [path for path in files if path.name not in SOURCE_TREE_TOOLS]


def is_sys_path_insert(node):
    """Return whether ``node`` is a ``sys.path.insert(...)`` call."""
    return (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "insert"
        and ast.unparse(node.func.value) == "sys.path")


def is_source_path_assignment(node):
    """Return whether ``node`` assigns ``SOURCE_PATH``."""
    return isinstance(node, ast.Assign) and any(
        isinstance(target, ast.Name) and target.id == "SOURCE_PATH"
        for target in node.targets)


def violations(source, filename, defines_source_path=False):
    """Return the source-tree-relative path patterns in ``source``."""
    tree = ast.parse(source, filename)
    allowed = set()
    for node in ast.walk(tree):
        # Importing a sibling module from the script's own directory is how
        # installed scripts share code, and it holds in every layout.
        if is_sys_path_insert(node):
            allowed.update(id(child) for child in ast.walk(node))
        if defines_source_path and is_source_path_assignment(node):
            allowed.update(id(child) for child in ast.walk(node))
    found = []
    for node in ast.walk(tree):
        if id(node) in allowed:
            continue
        where = f"{filename}:{getattr(node, 'lineno', 0)}"
        if isinstance(node, ast.Name) and node.id == "__file__":
            found.append(f"{where} derives a path from __file__")
        elif isinstance(node, ast.Attribute) and node.attr == "parents":
            found.append(f"{where} walks up with .parents")
        elif isinstance(node, ast.alias) and node.name == "SOURCE_PATH":
            found.append(f"{where} imports SOURCE_PATH; use default_path()")
        elif isinstance(node, ast.Name) and node.id == "SOURCE_PATH":
            if not defines_source_path:
                found.append(f"{where} uses SOURCE_PATH; use default_path()")
        elif isinstance(node, ast.Constant) and isinstance(node.value, str):
            if node.value.startswith(("src/", "/home/")) or "/src/" in node.value:
                found.append(f"{where} hard-codes a source path {node.value!r}")
    return found


class TestInstalledPaths(unittest.TestCase):
    """Installed code finds its data without the source tree."""

    def test_the_package_has_installed_files_to_check(self):
        names = {path.name for path in installed_files()}
        self.assertIn("imu_motion_probe.py", names)
        self.assertIn("rosmaster_gazebo_fortress.launch.py", names)
        self.assertTrue(DEFINING_MODULES <= names)

    def test_no_installed_file_depends_on_the_source_tree(self):
        found = []
        for path in installed_files():
            found += violations(
                path.read_text(encoding="utf-8"), str(path.relative_to(PACKAGE_DIR)),
                defines_source_path=path.name in DEFINING_MODULES)
        self.assertEqual(found, [])

    def test_the_defining_modules_fall_back_to_the_share_directory_first(self):
        for name in sorted(DEFINING_MODULES):
            text = (PACKAGE_DIR / "scripts" / name).read_text(encoding="utf-8")
            self.assertIn("def default_path():", text, name)
            self.assertIn("get_package_share_directory", text, name)

    def test_the_check_catches_what_a_copy_install_breaks(self):
        samples = {
            "from sensor_profiles import SOURCE_PATH\n": "imports SOURCE_PATH",
            "x = SOURCE_PATH\n": "uses SOURCE_PATH",
            "from pathlib import Path\nx = Path(__file__).resolve().parents[1]\n":
                "derives a path from __file__",
            "p = '/home/someone/ws/src/pkg/config/a.yaml'\n": "hard-codes",
        }
        for source, expected in samples.items():
            with self.subTest(source=source):
                self.assertTrue(
                    any(expected in line for line in violations(source, "sample.py")))

    def test_the_check_allows_the_sibling_import_idiom_and_the_definition(self):
        idiom = (
            "import os, sys\n"
            "sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))\n")
        self.assertEqual(violations(idiom, "sample.py"), [])
        definition = (
            "from pathlib import Path\n"
            "SOURCE_PATH = Path(__file__).resolve().parents[1] / 'config' / 'a.yaml'\n"
            "def default_path():\n    return SOURCE_PATH\n")
        self.assertEqual(violations(definition, "sample.py", True), [])


if __name__ == "__main__":
    unittest.main()
