"""
Grade the published depth image against the rendered one, in plain numpy.

The camera adapter turns each rendered depth image into the one it publishes
(#43 step 7). This module says whether a set of (rendered, published) frame
pairs, matched by stamp, looks like the physical camera under the physical
profile, or like the render itself under the ideal one. It needs no ROS graph
and shares no code with the adapter: the expected scale, noise and range come
from the parity ledger's physical section, and the model is written out again
here from the calibration doc, so a mistake in one cannot pass the other.

Under the physical profile, with ``s`` the scale error, ``z`` the rendered
depth and ``d = z * (1 + s)`` the depth the camera reports:

* nothing below the minimum range is ever published, and no infinity, exactly;
* a pixel is NaN wherever ``d`` is more than six standard deviations under the
  minimum range, and finite wherever it is more than six over it (a pixel that
  close to the cutoff may fall either side by noise, and about one in a billion
  crosses six deviations);
* the mean of ``published / rendered`` over the pixels rendered 1.5 m or more
  away lies inside the physical range of scale errors, within its standard error;
* in each band of distances the residual ``published - z * (1 + s)`` has the
  standard deviation of ``sigma(d) = max(floor, coefficient * d ** exponent)``
  to within five standard errors and a percent, and a mean of zero to within
  five standard errors of the band and of the scale estimate.

Under the ideal profile every finite rendered pixel is published bit for bit.
Under both, a pixel with no render return is NaN, never an infinity.
"""

from collections import OrderedDict
import math
import re

import numpy as np


NOISE_MODEL = re.compile(
    r"^sigma = max\((?P<floor>[0-9.]+), (?P<coefficient>[0-9.]+) \* "
    r"d\^(?P<exponent>[0-9.]+)\) m$")
# Standard deviations from the cutoff beyond which a pixel's side is certain.
CUTOFF_DEVIATIONS = 6.0
# Only pixels rendered this far away estimate the scale: their ratio's spread
# is small, and none is near the cutoff.
SCALE_MIN_DEPTH_M = 1.5
# Bands of reported distance, in metres, that the noise is graded in.
NOISE_BANDS = (
    (0.65, 0.8), (0.8, 1.0), (1.0, 1.3), (1.3, 1.8),
    (1.8, 2.5), (2.5, 3.6), (3.6, 5.0), (5.0, 8.1))
MIN_BAND_PIXELS = 300
MIN_GRADED_BANDS = 2
NOISE_DEVIATIONS = 5.0
# Room for the sigma varying inside a band and for the scale's own estimate.
NOISE_RELATIVE_SLACK = 0.01


class FramePairs:
    """
    Pair rendered and published frames that carry exactly the same stamp.

    A few frames of each kind are held while their partner is awaited, so the
    holding stays small however long the window runs. ``pairs`` collects
    (rendered, published) in the order the second of each pair arrived.
    """

    def __init__(self, hold=5):
        self._hold = hold
        self._waiting = {"rendered": OrderedDict(), "published": OrderedDict()}
        self.pairs = []

    def add(self, kind, stamp, pixels):
        """Take one frame of ``kind`` ("rendered" or "published") stamped ``stamp``."""
        other = "published" if kind == "rendered" else "rendered"
        partner = self._waiting[other].pop(stamp, None)
        if partner is None:
            self._waiting[kind][stamp] = pixels
            while len(self._waiting[kind]) > self._hold:
                self._waiting[kind].popitem(last=False)
        elif kind == "rendered":
            self.pairs.append((pixels, partner))
        else:
            self.pairs.append((partner, pixels))


def expectations_from_ledger(contract):
    """Return the physical expectations of ``contract``'s physical depth section."""
    scale = contract.physical("depth.scale_error")
    parsed = NOISE_MODEL.match(contract.physical("depth.noise_model"))
    if parsed is None:
        raise ValueError("physical depth.noise_model is not 'sigma = max(f, c * d^e) m'")
    return {
        "scale_min": float(scale["min"]),
        "scale_max": float(scale["max"]),
        "min_range_m": float(contract.physical("depth.min_range_m")),
        "sigma_floor_m": float(parsed["floor"]),
        "sigma_coefficient": float(parsed["coefficient"]),
        "sigma_exponent": float(parsed["exponent"]),
    }


def _sigma(distance, expected):
    return np.maximum(
        expected["sigma_floor_m"],
        expected["sigma_coefficient"] * np.power(distance, expected["sigma_exponent"]))


def _pooled(pairs):
    """Return the rendered and published pixels of every pair, as float64 vectors."""
    rendered = np.concatenate([np.asarray(r, dtype=np.float64).ravel() for r, _ in pairs])
    published = np.concatenate([np.asarray(p, dtype=np.float64).ravel() for _, p in pairs])
    return rendered, published


def shared_errors(pairs):
    """Return what every profile promises: same shape, NaN and never infinity."""
    errors = []
    for index, (rendered, published) in enumerate(pairs):
        rendered, published = np.asarray(rendered), np.asarray(published)
        if rendered.shape != published.shape:
            errors.append(
                f"frame {index}: published {published.shape} but rendered {rendered.shape}")
            continue
        infinite = int(np.count_nonzero(np.isinf(published)))
        if infinite:
            errors.append(f"frame {index}: {infinite} published pixels are infinite, not NaN")
        no_return = ~np.isfinite(rendered)
        leaked = int(np.count_nonzero(no_return & np.isfinite(published)))
        if leaked:
            errors.append(
                f"frame {index}: {leaked} pixels with no render return are finite")
    return errors


def ideal_errors(pairs):
    """Return where a finite rendered pixel was not published bit for bit."""
    errors = shared_errors(pairs)
    for index, (rendered, published) in enumerate(pairs):
        rendered = np.ascontiguousarray(rendered, dtype=np.float32)
        published = np.ascontiguousarray(published, dtype=np.float32)
        if rendered.shape != published.shape:
            continue
        finite = np.isfinite(rendered)
        changed = int(np.count_nonzero(
            rendered.view(np.uint32)[finite] != published.view(np.uint32)[finite]))
        if changed:
            errors.append(
                f"frame {index}: {changed} finite rendered pixels were not published "
                "bit for bit")
    return errors


def physical_errors(pairs, expected, notes=None):
    """
    Return where the published depth departs from the physical camera's.

    ``notes``, if given, is a list that takes a line for what was measured.
    """
    notes = [] if notes is None else notes
    errors = shared_errors(pairs)
    if errors:
        return errors
    rendered, published = _pooled(pairs)
    minimum = expected["min_range_m"]

    below = int(np.count_nonzero(published < minimum))
    if below:
        errors.append(
            f"{below} published pixels are below the {minimum} m minimum range "
            f"(lowest {np.nanmin(published):.4f} m)")

    usable = np.isfinite(rendered) & np.isfinite(published) & (
        rendered >= SCALE_MIN_DEPTH_M)
    if int(np.count_nonzero(usable)) < 1000:
        return errors + [
            f"only {int(np.count_nonzero(usable))} pixels are rendered "
            f"{SCALE_MIN_DEPTH_M} m or more away and published, too few to grade the scale"]
    ratio = published[usable] / rendered[usable]
    scale = float(ratio.mean()) - 1.0
    standard_error = float(ratio.std()) / math.sqrt(ratio.size)
    low = expected["scale_min"] - NOISE_DEVIATIONS * standard_error
    high = expected["scale_max"] + NOISE_DEVIATIONS * standard_error
    if not low <= scale <= high:
        errors.append(
            f"scale error {scale:+.5f} is outside the physical range "
            f"{expected['scale_min']:+.4f}..{expected['scale_max']:+.4f} "
            f"(standard error {standard_error:.5f})")

    notes.append(
        f"scale error {scale:+.5f} (standard error {standard_error:.5f}, "
        f"{ratio.size} pixels; physical range "
        f"{expected['scale_min']:+.4f}..{expected['scale_max']:+.4f})")
    finite = np.isfinite(rendered)
    reported = np.where(finite, rendered * (1.0 + scale), np.nan)
    with np.errstate(invalid="ignore"):
        margin = CUTOFF_DEVIATIONS * _sigma(np.where(finite, reported, 1.0), expected)
        certainly_gone = finite & (reported < minimum - margin)
        certainly_there = finite & (reported > minimum + margin)
    kept = int(np.count_nonzero(certainly_gone & np.isfinite(published)))
    if kept:
        errors.append(
            f"{kept} pixels {CUTOFF_DEVIATIONS:g} deviations under the {minimum} m "
            "minimum range were published")
    dropped = int(np.count_nonzero(certainly_there & ~np.isfinite(published)))
    if dropped:
        errors.append(
            f"{dropped} pixels {CUTOFF_DEVIATIONS:g} deviations over the {minimum} m "
            "minimum range are NaN")

    graded = 0
    for low_m, high_m in NOISE_BANDS:
        band = np.isfinite(published) & finite & (reported >= low_m) & (reported < high_m)
        count = int(np.count_nonzero(band))
        if count < MIN_BAND_PIXELS:
            continue
        graded += 1
        residual = published[band] - rendered[band] * (1.0 + scale)
        expected_std = math.sqrt(float(np.mean(_sigma(reported[band], expected) ** 2)))
        measured_std = float(residual.std())
        notes.append(
            f"noise {low_m}-{high_m} m: {measured_std * 1000:.2f} mm against "
            f"{expected_std * 1000:.2f} mm ({count} pixels)")
        tolerance = (
            NOISE_DEVIATIONS / math.sqrt(2.0 * count) + NOISE_RELATIVE_SLACK)
        if abs(measured_std / expected_std - 1.0) > tolerance:
            errors.append(
                f"noise at {low_m}-{high_m} m: standard deviation {measured_std:.5f} m, "
                f"expected {expected_std:.5f} m within {100 * tolerance:.1f}% "
                f"({count} pixels)")
        # The band's mean residual scatters by its own standard error, and by
        # the scale estimate's error times the distance, which is not small.
        bias_limit = NOISE_DEVIATIONS * math.hypot(
            expected_std / math.sqrt(count),
            float(reported[band].mean()) * standard_error)
        if abs(float(residual.mean())) > bias_limit:
            errors.append(
                f"scale at {low_m}-{high_m} m: mean residual {residual.mean():+.5f} m "
                f"exceeds {bias_limit:.5f} m ({count} pixels)")
    if graded < MIN_GRADED_BANDS:
        errors.append(
            f"only {graded} distance bands hold {MIN_BAND_PIXELS} pixels or more; "
            f"the noise needs {MIN_GRADED_BANDS} to be graded")
    return errors


def depth_quality_errors(pairs, profile, expected, notes=None):
    """Return the errors of ``pairs`` under the named sensor profile."""
    if not pairs:
        return ["no rendered and published depth frames were paired"]
    if profile == "ideal":
        return ideal_errors(pairs)
    if profile == "physical":
        return physical_errors(pairs, expected, notes)
    raise ValueError(f"unknown sensor profile {profile!r}")
