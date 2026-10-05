#!/usr/bin/env python3
"""
Print the reports of measure_python_node_cpu.py as one table per file.

    python3 tools/report_python_node_cpu.py REPORT.json [REPORT.json ...]

One row per figure, one column per variant, each cell "median (min-max) n=N".
Rows are the CPU of the Python nodes and the camera adapter's threads, the
cloud latency, the real-time factor and the message counts; the full report
stays in the JSON.
"""

import json
from pathlib import Path
import sys

PREFIXES = (
    "processes_cpu_percent.", "threads_cpu_percent.",
    "processes_cpu_percent_per_sim_s.camera_adapter", "cloud_", "real_time_factor",
    "session_cpu_percent", "topic_count.", "window_", "stream.")
MINIMUM_CPU = 0.5


def rows(summary):
    """Return the figure names worth printing, in a stable order."""
    names = set()
    for figures in summary.values():
        for name, value in figures.items():
            if not isinstance(value, dict) or not name.startswith(PREFIXES):
                continue
            if "cpu_percent" in name and value["max"] < MINIMUM_CPU:
                continue
            names.add(name)
    return sorted(names)


def main(paths):
    """Print each report."""
    for path in paths:
        report = json.loads(Path(path).read_text(encoding="utf-8"))
        print(f"\n## {path}: {report['render']}, {report['world']}, {report['profile']}, "
              f"numpy {report['numpy']}, {report['cpus']} CPUs")
        variants = list(report["summary"])
        print("figure | " + " | ".join(variants))
        for name in rows(report["summary"]):
            cells = []
            for variant in variants:
                cell = report["summary"][variant].get(name)
                cells.append("-" if cell is None else (
                    f"{cell['median']:g} ({cell['min']:g}-{cell['max']:g}) n={cell['n']}"))
            print(f"{name} | " + " | ".join(cells))
        for variant in variants:
            print(f"/clock subscribers [{variant}]: "
                  f"{report['summary'][variant].get('clock_subscribers')}")


if __name__ == "__main__":
    main(sys.argv[1:])
