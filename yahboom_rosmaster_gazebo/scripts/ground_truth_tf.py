#!/usr/bin/env python3
"""
Broadcast a display-only, frame-aligned simulator ground-truth pose.

The raw `/ground_truth/odom` measurement remains `world -> base_footprint` and
never enters the robot-facing TF tree. This node creates only the separate
`ground_truth_base` diagnostic frame. It captures one fixed `odom <- world`
alignment, then optionally one fixed `map <- world` alignment when SLAM appears.
Later `map -> odom` corrections must not move the ground-truth frame.
"""

from collections import deque
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener

from ground_truth_alignment import RigidTransform, compose, inverse


class GroundTruthTf(Node):
    """Publish ground truth in a fixed diagnostic display frame."""

    def __init__(self):
        super().__init__("ground_truth_tf")
        self.declare_parameter("input_topic", "/ground_truth/odom")
        self.declare_parameter("frame_id", "auto")
        self.declare_parameter("child_frame_id", "ground_truth_base")
        self.declare_parameter("world_frame_id", "world")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("alignment_timeout", 2.0)

        self.frame_mode = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.world_frame_id = self.get_parameter("world_frame_id").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.map_frame_id = self.get_parameter("map_frame_id").value
        self.alignment_timeout = float(
            self.get_parameter("alignment_timeout").value)
        input_topic = self.get_parameter("input_topic").value

        if not all((
                self.frame_mode,
                self.child_frame_id,
                self.world_frame_id,
                self.odom_frame_id,
                self.base_frame_id,
                self.map_frame_id)):
            raise ValueError("ground-truth frame parameters must not be empty")
        if self.alignment_timeout <= 0.0:
            raise ValueError("alignment_timeout must be positive")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.pending_ground_truth = deque(maxlen=250)
        self.recent_ground_truth = deque(maxlen=100)
        self.odom_from_world = None
        self.display_from_world = None
        self.display_frame_id = None
        self.last_input_error = 0.0
        self.create_subscription(Odometry, input_topic, self.odom_callback, 10)
        self.create_timer(0.05, self.process_pending)

        self.get_logger().info(
            f"Aligning {input_topic} into frame mode {self.frame_mode!r} as "
            f"{self.child_frame_id}")

    @staticmethod
    def normalized_frame(frame_id):
        """Normalize a frame ID for comparisons without changing output."""
        return frame_id.lstrip("/")

    @staticmethod
    def pose_transform(pose):
        """Convert a geometry pose into a transform value."""
        return RigidTransform(
            (pose.position.x, pose.position.y, pose.position.z),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )

    @staticmethod
    def message_transform(transform):
        """Convert a geometry transform into a transform value."""
        return RigidTransform(
            (
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ),
            (
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ),
        )

    def valid_input_frames(self, message):
        """Reject a source whose coordinates do not match the declared contract."""
        valid = (
            self.normalized_frame(message.header.frame_id)
            == self.normalized_frame(self.world_frame_id)
            and self.normalized_frame(message.child_frame_id)
            == self.normalized_frame(self.base_frame_id)
        )
        if not valid and time.monotonic() - self.last_input_error >= 5.0:
            self.last_input_error = time.monotonic()
            self.get_logger().error(
                "Ignoring ground truth with frames "
                f"{message.header.frame_id!r} -> {message.child_frame_id!r}; "
                f"expected {self.world_frame_id!r} -> {self.base_frame_id!r}")
        return valid

    def odom_callback(self, message):
        """Queue or publish one raw Gazebo ground-truth measurement."""
        if not self.valid_input_frames(message):
            return
        self.recent_ground_truth.append(message)
        if self.odom_from_world is None:
            self.pending_ground_truth.append((time.monotonic(), message))
            self.process_pending()
            return
        self.maybe_select_localization_frame(message)
        self.publish_ground_truth(message)

    def lookup(self, target_frame, source_frame, stamp=None):
        """Look up a transform, returning None while the TF data catches up."""
        query_time = Time() if stamp is None else Time.from_msg(stamp)
        try:
            result = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                query_time,
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
            return None
        return self.message_transform(result.transform)

    def process_pending(self):
        """Find a time-aligned odometry sample and establish the fixed origin."""
        if self.odom_from_world is not None:
            target_frame = self.requested_localization_frame()
            if target_frame is None or self.display_frame_id == target_frame:
                self.recent_ground_truth.clear()
                return
            if self.lookup(target_frame, self.odom_frame_id) is None:
                return
            for message in tuple(self.recent_ground_truth):
                if self.maybe_select_localization_frame(message):
                    self.recent_ground_truth.clear()
                    break
            return
        if not self.pending_ground_truth:
            return

        now = time.monotonic()
        for arrival, message in tuple(self.pending_ground_truth):
            odom_from_base = self.lookup(
                self.odom_frame_id,
                self.base_frame_id,
                message.header.stamp,
            )
            if odom_from_base is None:
                continue

            world_from_base = self.pose_transform(message.pose.pose)
            try:
                self.odom_from_world = compose(
                    odom_from_base, inverse(world_from_base))
            except ValueError as exception:
                self.get_logger().error(
                    f"Cannot align invalid ground-truth pose: {exception}")
                self.pending_ground_truth.clear()
                return

            if self.frame_mode in ("auto", self.odom_frame_id):
                self.display_frame_id = self.odom_frame_id
                self.display_from_world = self.odom_from_world
            self.get_logger().info(
                f"Captured fixed {self.odom_frame_id} <- {self.world_frame_id} "
                "ground-truth alignment")
            self.maybe_select_localization_frame(message)

            queued = [
                queued_message
                for queued_arrival, queued_message in self.pending_ground_truth
                if queued_arrival >= arrival
            ]
            self.pending_ground_truth.clear()
            for queued_message in queued:
                self.publish_ground_truth(queued_message)
            return

        while (
                self.pending_ground_truth
                and now - self.pending_ground_truth[0][0] > self.alignment_timeout):
            self.pending_ground_truth.popleft()
        if not self.pending_ground_truth:
            self.get_logger().warning(
                f"Could not align ground truth: no synchronized "
                f"{self.odom_frame_id} -> {self.base_frame_id} TF within "
                f"{self.alignment_timeout:.1f}s")

    def requested_localization_frame(self):
        """Return the final localization frame requested by the mode."""
        if self.frame_mode == "auto":
            return self.map_frame_id
        if self.frame_mode == self.odom_frame_id:
            return None
        return self.frame_mode

    def maybe_select_localization_frame(self, ground_truth):
        """Capture one fixed localization-to-world alignment when available."""
        target_frame = self.requested_localization_frame()
        if (
                self.odom_from_world is None
                or target_frame is None
                or self.display_frame_id == target_frame):
            return False

        target_from_odom = self.lookup(
            target_frame, self.odom_frame_id, ground_truth.header.stamp)
        odom_from_base = self.lookup(
            self.odom_frame_id,
            self.base_frame_id,
            ground_truth.header.stamp,
        )
        if target_from_odom is None or odom_from_base is None:
            return False
        world_from_base = self.pose_transform(ground_truth.pose.pose)
        self.display_from_world = compose(
            compose(target_from_odom, odom_from_base),
            inverse(world_from_base),
        )
        previous_frame = self.display_frame_id
        self.display_frame_id = target_frame
        if previous_frame is None:
            self.get_logger().info(
                f"Captured fixed {target_frame} <- {self.world_frame_id} "
                "ground-truth alignment")
        else:
            self.get_logger().info(
                f"Ground-truth display switched from {previous_frame} to "
                f"{target_frame}; later {target_frame} -> {self.odom_frame_id} "
                "corrections will not move truth")
        return True

    def publish_ground_truth(self, message):
        """Publish one aligned diagnostic transform without touching robot TF."""
        if self.display_from_world is None or self.display_frame_id is None:
            return
        try:
            display_from_base = compose(
                self.display_from_world,
                self.pose_transform(message.pose.pose),
            )
        except ValueError as exception:
            self.get_logger().error(
                f"Cannot publish invalid ground-truth pose: {exception}")
            return

        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = self.display_frame_id
        transform.child_frame_id = self.child_frame_id
        transform.transform.translation.x = display_from_base.translation[0]
        transform.transform.translation.y = display_from_base.translation[1]
        transform.transform.translation.z = display_from_base.translation[2]
        transform.transform.rotation.x = display_from_base.rotation[0]
        transform.transform.rotation.y = display_from_base.rotation[1]
        transform.transform.rotation.z = display_from_base.rotation[2]
        transform.transform.rotation.w = display_from_base.rotation[3]
        if not all(math.isfinite(value) for value in (
                *display_from_base.translation, *display_from_base.rotation)):
            self.get_logger().error("Aligned ground truth contains non-finite values")
            return
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = GroundTruthTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
