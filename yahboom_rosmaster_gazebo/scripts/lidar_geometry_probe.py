#!/usr/bin/env python3
"""Validate the simulated LiDAR against asymmetric known geometry."""

import math
import statistics
import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


EXPECTED_SAMPLES = 720
EXPECTED_RATE_HZ = 5.0
EXPECTED_SCAN_PERIOD = 1.0 / EXPECTED_RATE_HZ
EXPECTED_RANGE_MIN = 0.20
EXPECTED_RANGE_MAX = 30.0
EXPECTED_FRONT_RANGE = 1.90
LEFT_TARGET_X_FACE = 2.30
LEFT_TARGET_BEARING = math.atan2(1.20, 2.40)


class LidarGeometryProbe(Node):
    """Collect scans and enforce metadata, geometry, timing, and TF contracts."""

    def __init__(self):
        super().__init__("lidar_geometry_probe")
        self.declare_parameter("timeout", 30.0)
        self.declare_parameter("samples", 12)
        self.timeout = float(self.get_parameter("timeout").value)
        self.sample_count = max(10, int(self.get_parameter("samples").value))
        self.scans = []
        self.arrival_times = []
        self.started_at = time.monotonic()

        sensor_qos = QoSProfile(
            depth=max(20, self.sample_count),
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.capture, sensor_qos)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0), node=self)
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False)

        self.get_logger().info(
            f"Waiting up to {self.timeout:.1f}s for {self.sample_count} LiDAR scans")

    def capture(self, scan):
        """Keep the requested number of scans and their wall-clock arrival times."""
        if len(self.scans) < self.sample_count:
            self.scans.append(scan)
            self.arrival_times.append(time.monotonic())

    @staticmethod
    def stamp_seconds(scan):
        """Convert a LaserScan header stamp to floating-point seconds."""
        return float(scan.header.stamp.sec) + float(scan.header.stamp.nanosec) * 1e-9

    @staticmethod
    def angle_at(scan, index):
        """Return the declared angle of a sample index."""
        return scan.angle_min + index * scan.angle_increment

    @classmethod
    def sector_samples(cls, scan, center, half_width):
        """Return ``(angle, range)`` samples in an angular sector."""
        return [
            (cls.angle_at(scan, index), value)
            for index, value in enumerate(scan.ranges)
            if abs(cls.angle_at(scan, index) - center) <= half_width
        ]

    @staticmethod
    def finite_sector_values(samples):
        """Return finite range values from an angular-sector sample list."""
        return [value for _, value in samples if math.isfinite(float(value))]

    def validate_scan_structure(self, scan, scan_index, errors):
        """Validate one scan's frame, angular grid, ranges, and temporal fields."""
        label = f"scan[{scan_index}]"
        if scan.header.frame_id != "laser_frame":
            errors.append(
                f"{label}: expected frame laser_frame, got {scan.header.frame_id!r}")
        if len(scan.ranges) != EXPECTED_SAMPLES:
            errors.append(
                f"{label}: expected {EXPECTED_SAMPLES} ranges, got {len(scan.ranges)}")
            return

        metadata = (
            scan.angle_min, scan.angle_max, scan.angle_increment,
            scan.time_increment, scan.scan_time, scan.range_min, scan.range_max,
        )
        if not all(math.isfinite(float(value)) for value in metadata):
            errors.append(f"{label}: metadata contains non-finite values")
            return
        if scan.angle_increment <= 0.0:
            errors.append(f"{label}: angle_increment is not positive")
        if not scan.angle_min < 0.0 < scan.angle_max:
            errors.append(f"{label}: angular interval does not cross the front axis")
        if not math.isclose(scan.angle_min, -math.pi, abs_tol=2e-5):
            errors.append(f"{label}: angle_min {scan.angle_min:.8f} is not -pi")
        if not math.isclose(scan.angle_max, math.pi, abs_tol=2e-5):
            errors.append(f"{label}: angle_max {scan.angle_max:.8f} is not +pi")
        reconstructed_max = scan.angle_min + (len(scan.ranges) - 1) * scan.angle_increment
        if not math.isclose(
                reconstructed_max, scan.angle_max,
                abs_tol=max(1e-6, abs(scan.angle_increment) * 1e-3)):
            errors.append(
                f"{label}: sample order/grid ends at {reconstructed_max:.8f}, "
                f"not angle_max {scan.angle_max:.8f}")
        if not math.isclose(scan.range_min, EXPECTED_RANGE_MIN, abs_tol=1e-5):
            errors.append(f"{label}: unexpected range_min {scan.range_min:.6f}")
        if not math.isclose(scan.range_max, EXPECTED_RANGE_MAX, abs_tol=1e-5):
            errors.append(f"{label}: unexpected range_max {scan.range_max:.6f}")

        finite_count = 0
        invalid_count = 0
        for range_index, value in enumerate(scan.ranges):
            value = float(value)
            if math.isfinite(value):
                finite_count += 1
                if not scan.range_min <= value <= scan.range_max:
                    errors.append(
                        f"{label}: finite range[{range_index}]={value:.6f} "
                        "is outside declared bounds")
                    break
            elif math.isnan(value) or value == math.inf:
                invalid_count += 1
            else:
                errors.append(
                    f"{label}: range[{range_index}] uses invalid marker {value}")
                break
        if finite_count == 0:
            errors.append(f"{label}: no finite target returns")
        if invalid_count == 0:
            errors.append(f"{label}: no invalid/no-return markers in the empty sectors")

        # The current Fortress bridge leaves scan_time unspecified. The GPU
        # LiDAR renders one complete scan snapshot, so time_increment must also
        # stay zero rather than claiming a rolling per-ray acquisition.
        if not math.isclose(scan.scan_time, 0.0, abs_tol=1e-12):
            errors.append(
                f"{label}: current bridge contract expects unspecified "
                f"scan_time=0, got {scan.scan_time:.9f}")
        if not math.isclose(scan.time_increment, 0.0, abs_tol=1e-12):
            errors.append(
                f"{label}: snapshot acquisition requires time_increment=0, "
                f"got {scan.time_increment:.9f}")

    def validate_geometry(self, errors):
        """Validate front range plus an asymmetric positive-y target."""
        front_medians = []
        left_residual_medians = []
        mirror_invalid_fractions = []

        for scan in self.scans:
            front = self.finite_sector_values(
                self.sector_samples(scan, center=0.0, half_width=0.04))
            if front:
                front_medians.append(statistics.median(front))

            left_samples = self.sector_samples(
                scan, center=LEFT_TARGET_BEARING, half_width=0.025)
            left_residuals = [
                abs(float(value) - LEFT_TARGET_X_FACE / math.cos(angle))
                for angle, value in left_samples
                if math.isfinite(float(value))
            ]
            if left_residuals:
                left_residual_medians.append(statistics.median(left_residuals))

            mirror = self.sector_samples(
                scan, center=-LEFT_TARGET_BEARING, half_width=0.025)
            if mirror:
                mirror_invalid_fractions.append(
                    sum(not math.isfinite(float(value)) for _, value in mirror)
                    / len(mirror))

        required_good_scans = math.ceil(0.8 * len(self.scans))
        good_front = sum(
            abs(value - EXPECTED_FRONT_RANGE) <= 0.08
            for value in front_medians)
        if good_front < required_good_scans:
            detail = (
                f"median={statistics.median(front_medians):.3f}m"
                if front_medians else "no finite returns")
            errors.append(
                f"front target: {good_front}/{len(self.scans)} scans within "
                f"{EXPECTED_FRONT_RANGE:.2f}+/-0.08m ({detail})")

        good_left = sum(value <= 0.08 for value in left_residual_medians)
        if good_left < required_good_scans:
            detail = (
                f"median residual={statistics.median(left_residual_medians):.3f}m"
                if left_residual_medians else "no finite returns")
            errors.append(
                f"positive-y target: {good_left}/{len(self.scans)} scans match "
                f"the +{LEFT_TARGET_BEARING:.3f}rad geometry ({detail})")

        good_mirror = sum(value >= 0.8 for value in mirror_invalid_fractions)
        if good_mirror < required_good_scans:
            errors.append(
                f"handedness: only {good_mirror}/{len(self.scans)} scans keep "
                "the mirrored negative-y sector clear")

    def validate_tf(self, errors):
        """Require the LiDAR frame to resolve through TF at representative stamps."""
        # Avoid the two collection boundaries: the first scan may predate the
        # listener's dynamic-TF cache, while the newest scan can arrive before
        # its matching odometry transform. Interior samples still prove lookup
        # at real scan timestamps rather than only accepting the latest TF.
        representative = (
            self.scans[2], self.scans[len(self.scans) // 2], self.scans[-3])
        for scan in representative:
            stamp = Time.from_msg(scan.header.stamp)
            if not self.tf_buffer.can_transform(
                    "odom", scan.header.frame_id, stamp,
                    timeout=Duration(seconds=0.25)):
                errors.append(
                    "TF: cannot resolve odom -> laser_frame at scan stamp "
                    f"{self.stamp_seconds(scan):.9f}")

    def validate(self):
        """Return all contract errors after scan collection."""
        errors = []
        if len(self.scans) < self.sample_count:
            return [f"received {len(self.scans)}/{self.sample_count} scans"]

        stamps = [self.stamp_seconds(scan) for scan in self.scans]
        if any(stamp <= 0.0 for stamp in stamps):
            errors.append("header timestamps contain zero or negative simulation time")
        if any(current <= previous for previous, current in zip(stamps, stamps[1:])):
            errors.append("header timestamps are not strictly increasing")

        for scan_index, scan in enumerate(self.scans):
            self.validate_scan_structure(scan, scan_index, errors)

        stamp_deltas = [
            current - previous for previous, current in zip(stamps, stamps[1:])]
        if stamp_deltas:
            median_period = statistics.median(stamp_deltas)
            measured_rate = 1.0 / median_period if median_period > 0.0 else math.inf
            if not 4.5 <= measured_rate <= 5.5:
                errors.append(
                    f"scan rate: expected about {EXPECTED_RATE_HZ:.1f}Hz, "
                    f"measured {measured_rate:.3f}Hz from simulation stamps")
            if not math.isclose(
                    median_period, EXPECTED_SCAN_PERIOD, abs_tol=0.02):
                errors.append(
                    f"scan period: expected {EXPECTED_SCAN_PERIOD:.3f}s, "
                    f"measured {median_period:.6f}s from simulation stamps")
            if max(abs(delta - median_period) for delta in stamp_deltas) > 0.025:
                errors.append(
                    "scan rate: simulation-time period jitter exceeds 0.025s; "
                    f"periods={[round(delta, 6) for delta in stamp_deltas]}")
        first_arrival = self.arrival_times[0] - self.started_at
        if first_arrival > 5.0:
            errors.append(
                f"first scan arrived {first_arrival:.3f}s after probe startup; "
                "expected an already-running 5Hz sensor within 5s")

        self.validate_geometry(errors)
        self.validate_tf(errors)
        return errors

    def summary(self):
        """Return measured count, rate, latency, geometry, and timing diagnostics."""
        if not self.scans:
            return "no scans received"
        stamps = [self.stamp_seconds(scan) for scan in self.scans]
        periods = [
            current - previous for previous, current in zip(stamps, stamps[1:])]
        rate = 1.0 / statistics.median(periods) if periods else 0.0
        first_latency = self.arrival_times[0] - self.started_at
        scan = self.scans[-1]
        front = self.finite_sector_values(
            self.sector_samples(scan, center=0.0, half_width=0.04))
        front_text = f"{statistics.median(front):.3f}m" if front else "missing"
        left_samples = self.sector_samples(
            scan, center=LEFT_TARGET_BEARING, half_width=0.025)
        left_residuals = [
            abs(float(value) - LEFT_TARGET_X_FACE / math.cos(angle))
            for angle, value in left_samples
            if math.isfinite(float(value))
        ]
        left_text = (
            f"{statistics.median(left_residuals):.3f}m"
            if left_residuals else "missing")
        return (
            f"scans={len(self.scans)}, rate={rate:.3f}Hz(sim), "
            f"first_arrival={first_latency:.3f}s(wall), front={front_text}, "
            f"left_residual={left_text}, "
            f"scan_time={scan.scan_time:.9f}s, "
            f"time_increment={scan.time_increment:.9f}s")


def main():
    rclpy.init()
    node = LidarGeometryProbe()
    deadline = time.monotonic() + node.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if len(node.scans) >= node.sample_count:
                # Keep spinning briefly so TF can advance past the newest scan.
                tf_deadline = time.monotonic() + 0.5
                while rclpy.ok() and time.monotonic() < tf_deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                break

        errors = node.validate()
        summary = node.summary()
        if errors:
            node.get_logger().error("LiDAR geometry contract FAILED: " + "; ".join(errors))
            node.get_logger().error("Measurements: " + summary)
            return 1

        node.get_logger().info("LiDAR geometry contract PASSED: " + summary)
        if node.scans[-1].scan_time == 0.0:
            node.get_logger().warning(
                "Fortress leaves scan_time unspecified (zero); the 0.2s scan "
                "period is validated from consecutive headers. time_increment=0 "
                "correctly declares this simulator's snapshot acquisition")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
