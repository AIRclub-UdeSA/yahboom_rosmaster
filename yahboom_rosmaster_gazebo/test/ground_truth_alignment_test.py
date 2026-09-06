#!/usr/bin/env python3
"""Unit tests for fixed world/odom/map ground-truth alignment."""

import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from ground_truth_alignment import (  # noqa: E402
    RigidTransform,
    compose,
    inverse,
    yaw_quaternion,
)


def quaternion_distance(left, right):
    """Return sign-insensitive Euclidean quaternion distance."""
    direct = math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))
    negated = math.sqrt(sum((a + b) ** 2 for a, b in zip(left, right)))
    return min(direct, negated)


class GroundTruthAlignmentTest(unittest.TestCase):
    """Protect the alignment math independently of ROS runtime timing."""

    def assert_transform_close(self, actual, expected, tolerance=1e-10):
        for actual_value, expected_value in zip(
                actual.translation, expected.translation):
            self.assertAlmostEqual(actual_value, expected_value, delta=tolerance)
        self.assertLessEqual(
            quaternion_distance(actual.rotation, expected.rotation), tolerance)

    def test_inverse_round_trip(self):
        value = RigidTransform((3.0, -2.0, 0.4), yaw_quaternion(0.73))
        identity = compose(value, inverse(value))
        self.assert_transform_close(
            identity,
            RigidTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        )

    def test_nonzero_spawn_aligns_truth_with_initial_odometry(self):
        world_from_base = RigidTransform(
            (8.0, -4.0, 0.0325), yaw_quaternion(-0.6))
        odom_from_base = RigidTransform(
            (0.0, 0.0, 0.0), yaw_quaternion(0.0))
        odom_from_world = compose(
            odom_from_base, inverse(world_from_base))

        aligned_truth = compose(odom_from_world, world_from_base)
        self.assert_transform_close(aligned_truth, odom_from_base)

    def test_map_alignment_is_frozen_after_first_slam_transform(self):
        world_from_base_initial = RigidTransform(
            (5.0, 2.0, 0.1), yaw_quaternion(-0.4))
        odom_from_base_initial = RigidTransform(
            (0.3, -0.2, 0.0), yaw_quaternion(0.1))
        world_from_base_at_slam_start = compose(
            world_from_base_initial,
            RigidTransform((0.5, 0.1, 0.0), yaw_quaternion(0.15)),
        )
        odom_from_base_at_slam_start = compose(
            odom_from_base_initial,
            RigidTransform((0.54, 0.08, 0.0), yaw_quaternion(0.17)),
        )

        first_map_from_odom = RigidTransform(
            (2.0, -1.0, 0.0), yaw_quaternion(0.25))
        fixed_map_from_world = compose(
            compose(first_map_from_odom, odom_from_base_at_slam_start),
            inverse(world_from_base_at_slam_start),
        )

        world_from_base_later = compose(
            world_from_base_initial,
            RigidTransform((0.8, -0.1, 0.0), yaw_quaternion(0.2)),
        )
        expected_truth = compose(fixed_map_from_world, world_from_base_later)

        corrected_map_from_odom = RigidTransform(
            (20.0, 7.0, 0.0), yaw_quaternion(-1.0))
        incorrectly_recomputed = compose(
            compose(corrected_map_from_odom, odom_from_base_at_slam_start),
            compose(inverse(world_from_base_at_slam_start), world_from_base_later),
        )

        self.assert_transform_close(
            compose(fixed_map_from_world, world_from_base_later),
            expected_truth,
        )
        self.assertGreater(
            math.dist(
                incorrectly_recomputed.translation,
                expected_truth.translation,
            ),
            1.0,
        )


if __name__ == "__main__":
    unittest.main()
