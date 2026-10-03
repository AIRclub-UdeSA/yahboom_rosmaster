"""
Fetch physical_rosmaster's contract probe at the commit the parity ledger pins.

``config/real_robot_contract.yaml`` names one physical_rosmaster commit,
``physical.provenance.commit``. It is the commit every physical value was read
at, and it is the commit whose ``tools/physical_contract_probe.py`` CI runs
against the simulator (#43 step 9), so there is one pin to move.

Nothing of physical_rosmaster is vendored. The probe is read with ``git show``
from a checkout of that repository, which is never touched, into a work
directory; physical_rosmaster's Apache-2.0 header stays on the copy. A missing
checkout, a commit that is not in it, or a pin that is not a full 40-character
SHA is an error, never a skip: a gate that quietly stops running is worse than
one that fails.

The checkout is ``$PHYSICAL_ROSMASTER_REPO`` when set (CI sets it to its sparse
checkout at the pin), and ``~/Documents/air-club/physical_rosmaster`` otherwise.
"""

import os
from pathlib import Path
import re
import subprocess

import yaml

LEDGER = (
    Path(__file__).resolve().parents[1] / "config" / "real_robot_contract.yaml")
PROBE = "tools/physical_contract_probe.py"
ENVIRONMENT_VARIABLE = "PHYSICAL_ROSMASTER_REPO"
DEFAULT_REPOSITORY = "~/Documents/air-club/physical_rosmaster"
FULL_SHA = re.compile(r"[0-9a-f]{40}")


class ProbeUnavailable(RuntimeError):
    """The pinned probe could not be read; the caller must fail, not skip."""


def require_full_sha(commit):
    """Return ``commit`` if it is a full 40-character SHA, and reject anything else."""
    if not isinstance(commit, str) or not FULL_SHA.fullmatch(commit):
        raise ValueError(f"commit must be a full 40-character SHA, got {commit!r}")
    return commit


def ledger_pin(ledger=LEDGER):
    """Return the physical_rosmaster commit the ledger pins, as a full SHA."""
    with open(ledger, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    return require_full_sha(data["physical"]["provenance"]["commit"])


def repository():
    """Return the physical_rosmaster checkout to read the probe from."""
    return Path(
        os.environ.get(ENVIRONMENT_VARIABLE) or DEFAULT_REPOSITORY).expanduser()


def fetch_contract_probe(checkout, commit, work_dir, patches=()):
    """
    Write the contract probe of ``commit`` into ``work_dir`` and return (path, blob).

    ``blob`` is the git blob id of the file as read, before any ``patches``
    (each applied in order with ``patch -p1`` and required to apply cleanly).
    Raises ``ProbeUnavailable`` when the checkout or the commit is missing.
    """
    require_full_sha(commit)
    checkout = Path(checkout)
    work_dir = Path(work_dir)
    work_dir.mkdir(parents=True, exist_ok=True)
    shown = subprocess.run(
        ["git", "-C", str(checkout), "show", f"{commit}:{PROBE}"],
        capture_output=True, text=True)
    if shown.returncode != 0:
        raise ProbeUnavailable(
            f"cannot read {PROBE} at physical_rosmaster {commit} from {checkout}: "
            f"{shown.stderr.strip() or 'git show failed'}. Set "
            f"{ENVIRONMENT_VARIABLE} to a checkout that has the commit.")
    probe = work_dir / "physical_contract_probe.py"
    probe.write_text(shown.stdout, encoding="utf-8")
    blob = subprocess.run(
        ["git", "hash-object", str(probe)], check=True, capture_output=True,
        text=True).stdout.strip()
    for patch in patches:
        subprocess.run(
            ["patch", "-p1", "--forward", "--no-backup-if-mismatch", "-i",
             str(Path(patch).resolve())],
            cwd=work_dir, check=True, capture_output=True, text=True)
    return probe, blob
