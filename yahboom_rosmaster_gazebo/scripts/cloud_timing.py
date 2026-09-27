"""
Timing model of the simulated point cloud, in plain Python with no ROS.

The physical X3 delivers a cloud for only about a quarter of its camera
frames. Its stamp gaps are whole camera frames, drawn independently from a
measured distribution, and every cloud arrives a fixed time after the frame
it was built from (sensor_profiles.yaml records the measurement).

``GapSampler`` draws the gaps, ``FrameGate`` decides which camera frames
become clouds, and ``publish_delay`` says how long to hold a finished cloud so
that it leaves at its capture stamp plus the latency.
"""

import bisect
import itertools
import math
import random


MAX_SEED = 2 ** 31 - 1


def resolve_seed(seed, entropy=random.SystemRandom()):
    """Return ``seed``, or a fresh random one when it is negative."""
    seed = int(seed)
    return seed if seed >= 0 else entropy.randrange(MAX_SEED + 1)


def _validated_distribution(distribution):
    """Return the (gap, probability) pairs of a distribution, sorted by gap."""
    if not distribution:
        raise ValueError("a gap distribution needs at least one entry")
    pairs = []
    for gap, probability in distribution.items():
        if isinstance(gap, bool) or not isinstance(gap, int) or gap < 1:
            raise ValueError(f"gap {gap!r} is not a whole number of frames >= 1")
        probability = float(probability)
        if not math.isfinite(probability) or probability <= 0.0:
            raise ValueError(
                f"gap {gap} has probability {probability}; it must be positive")
        pairs.append((gap, probability))
    return sorted(pairs)


def normalized(distribution):
    """Return the distribution scaled to sum to one."""
    pairs = _validated_distribution(distribution)
    total = sum(probability for _, probability in pairs)
    return {gap: probability / total for gap, probability in pairs}


def quantile(distribution, fraction):
    """
    Return the smallest gap whose cumulative probability reaches ``fraction``.

    A ``fraction`` at or below zero gives the shortest gap, and one above the
    total gives the longest.
    """
    pairs = list(normalized(distribution).items())
    cumulative = list(itertools.accumulate(probability for _, probability in pairs))
    # The epsilon absorbs the rounding of the running sum.
    index = bisect.bisect_left(cumulative, fraction - 1e-12)
    return pairs[min(index, len(pairs) - 1)][0]


def quantile_interval(distribution, fraction, samples, deviations=4.0):
    """
    Return the (lowest, highest) gap the sample quantile of ``fraction`` may take.

    The fraction of ``samples`` independent draws that fall at or below a gap
    has a standard deviation of sqrt(f * (1 - f) / samples) about its true
    value f, so the sample quantile stays within that many ``deviations`` of
    ``fraction`` when it is read back through the table. The larger the sample,
    the narrower the interval, which is how a test derives its tolerance from
    its sample size.
    """
    spread = deviations * math.sqrt(fraction * (1.0 - fraction) / samples)
    return quantile(distribution, fraction - spread), quantile(distribution, fraction + spread)


def gap_statistics(distribution):
    """Return the mean, median, p95 and longest gap (in frames) of a distribution."""
    pairs = list(normalized(distribution).items())
    return {
        "mean": sum(gap * probability for gap, probability in pairs),
        "median": quantile(distribution, 0.5),
        "p95": quantile(distribution, 0.95),
        "longest": pairs[-1][0],
    }


def cumulative_within(distribution, gap):
    """Return the probability of a gap of at most ``gap`` frames."""
    return sum(
        probability for frames, probability in normalized(distribution).items()
        if frames <= gap)


class GapSampler:
    """Draw the number of camera frames from one delivered cloud to the next."""

    def __init__(self, distribution, seed):
        pairs = list(normalized(distribution).items())
        self.gaps = [gap for gap, _ in pairs]
        self._cumulative = list(
            itertools.accumulate(probability for _, probability in pairs))
        self.seed = int(seed)
        self._random = random.Random(self.seed)

    def next_gap(self):
        """Return the next gap, in whole frames."""
        draw = self._random.random() * self._cumulative[-1]
        index = bisect.bisect_right(self._cumulative, draw)
        return self.gaps[min(index, len(self.gaps) - 1)]


class FrameGate:
    """
    Decide which camera frames become clouds.

    The first frame is always delivered. Each later delivery comes a drawn
    number of frames after the previous one, counted on the camera's frame
    clock, which ``frame_period_s`` gives. If the frame a gap lands on never
    arrives, the next one does instead, so the realized gap can only grow.

    A frame stamped less than half a period after the last one counted is not
    a new tick of that clock. Gazebo's camera emits such frames in a burst when
    it starts, catching up on the frames it owes since sim time zero, and they
    make no cloud.
    """

    def __init__(self, sampler, frame_period_s):
        if not math.isfinite(frame_period_s) or frame_period_s <= 0.0:
            raise ValueError("the frame period must be positive")
        self._sampler = sampler
        self._frame_period_s = float(frame_period_s)
        self._last_stamp = None
        self._index = 0
        self._next_delivery = 0

    def deliver(self, stamp_s):
        """Return whether the frame stamped ``stamp_s`` becomes a cloud."""
        if self._last_stamp is None:
            self._last_stamp = stamp_s
        else:
            # Rounding each step separately tolerates a millisecond of jitter
            # in the stamps, which summing them would let accumulate. A repeated
            # or reordered stamp rounds to zero or less, and so does a burst
            # frame, so neither counts.
            steps = round((stamp_s - self._last_stamp) / self._frame_period_s)
            if steps < 1:
                return False
            self._index += steps
            self._last_stamp = stamp_s
        if self._index < self._next_delivery:
            return False
        self._next_delivery = self._index + self._sampler.next_gap()
        return True


def publish_delay(stamp_s, now_s, latency_s):
    """
    Return how long to hold a cloud, and whether it was already late.

    A cloud leaves ``latency_s`` after its capture stamp. One that is ready
    after that goes out at once and counts as late. A zero latency asks for
    no delay at all, so nothing can be late.
    """
    delay = stamp_s + latency_s - now_s
    if delay >= 0.0:
        return delay, False
    return 0.0, latency_s > 0.0
