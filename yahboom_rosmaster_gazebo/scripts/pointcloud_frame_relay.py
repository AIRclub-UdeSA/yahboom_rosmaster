#!/usr/bin/env python3
"""
Republish the Fortress RGB-D point cloud in cam_1_depth_frame.

The rgbd_camera renders from ``cam_1_color_frame``: the physical Astra
registers depth to color, so its images are seen from the color aperture. The
physical robot still publishes its cloud in ``cam_1_depth_frame``
(yahboomcar_astra ``sensor_adapter.py``), 25.1 mm and 0.34 deg away.

Fortress 6.18 stamps the rgbd_camera's PointCloudPacked with the sensor's
``optical_frame_id`` even though the XYZ data is expressed in the camera's
regular +X-forward frame. Relabelling the header alone would therefore put the
cloud 25 mm off, so this relay transforms every finite point from
``source_frame`` into ``target_frame`` with the static TF between them, as the
physical adapter's ``transform_cloud()`` does, then labels it
``target_frame``. The layout (Gazebo's organized 24-byte points) is left alone.

The public output also switches to Best Effort here: the physical robot
publishes its point cloud as Best Effort (``qos_profile_sensor_data``), and
ros_gz_bridge cannot publish that directly, so this relay is also where the
QoS gets fixed to match the contract.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException


def rotation_matrix(quaternion):
    """Return the rotation matrix of a normalized geometry_msgs Quaternion."""
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return np.array((
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    ))


def transform_points(message, rotation, translation):
    """Transform the finite XYZ points of a cloud in place; keep non-finite ones."""
    fields = {field.name: field for field in message.fields}
    if not all(
            name in fields
            and fields[name].datatype == PointField.FLOAT32
            and fields[name].count == 1
            and fields[name].offset + 4 <= message.point_step
            for name in ("x", "y", "z")):
        raise ValueError("cloud has no scalar FLOAT32 x, y and z fields")
    if len(message.data) < message.row_step * message.height:
        raise ValueError("cloud data is shorter than row_step*height")
    endian = ">f4" if message.is_bigendian else "<f4"
    # Writable views into the message's own buffer, one per coordinate.
    coordinates = [
        np.ndarray(
            shape=(message.height, message.width),
            dtype=endian,
            buffer=message.data,
            offset=fields[name].offset,
            strides=(message.row_step, message.point_step),
        )
        for name in ("x", "y", "z")
    ]
    finite = np.logical_and.reduce([np.isfinite(axis) for axis in coordinates])
    x, y, z = (axis[finite].astype(np.float64) for axis in coordinates)
    # Element-wise rather than a matrix product: numpy hands `@` to a
    # multithreaded BLAS whose spinning threads starved the Gazebo server.
    for row, offset, axis in zip(rotation, translation, coordinates):
        axis[finite] = row[0] * x + row[1] * y + row[2] * z + offset


class PointCloudFrameRelay(Node):
    """Bridge-side transform: gz /cam_1/points -> public RGB-D cloud topic."""

    def __init__(self):
        super().__init__("pointcloud_frame_relay")
        self.declare_parameter("input_topic", "/internal/cam_1/points_raw")
        self.declare_parameter("output_topic", "cam_1/depth/color/points")
        # The frame the Gazebo cloud's XYZ is really expressed in: the
        # rgbd_camera's regular frame, whatever its header says.
        self.declare_parameter("source_frame", "cam_1_color_frame")
        self.declare_parameter("target_frame", "cam_1_depth_frame")
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.source_frame = self.get_parameter("source_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.rotation = None
        self.translation = None

        # Both frames hang off cam_1_link on fixed joints, so the transform is
        # static. Feed a buffer from /tf_static only rather than a full
        # listener that would also decode every /tf message.
        self.tf_buffer = Buffer()
        self.create_subscription(
            TFMessage, "/tf_static", self.store_static_transforms,
            QoSProfile(
                depth=100,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ))

        qos_in = QoSProfile(
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(PointCloud2, self.output_topic,
                                               qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.input_topic,
                                 self.republish, qos_in)
        self.get_logger().info(
            f"Transforming {self.input_topic} from {self.source_frame!r} to "
            f"{self.target_frame!r} on {self.output_topic}")

    def store_static_transforms(self, message):
        for transform in message.transforms:
            self.tf_buffer.set_transform_static(transform, "tf_static")

    def resolve_transform(self):
        """Cache the static target <- source transform once TF has it."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, Time()).transform
        except TransformException as exception:
            self.get_logger().warn(
                f"Dropping clouds until {self.target_frame} <- "
                f"{self.source_frame} is on /tf_static: {exception}",
                throttle_duration_sec=5.0)
            return False
        self.rotation = rotation_matrix(transform.rotation)
        self.translation = np.array((
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ))
        self.get_logger().info(
            f"{self.target_frame} <- {self.source_frame}: translation "
            f"{self.translation.tolist()}")
        return True

    def republish(self, message):
        if self.rotation is None and not self.resolve_transform():
            return
        try:
            transform_points(message, self.rotation, self.translation)
        except ValueError as exception:
            self.get_logger().error(
                f"Dropping cloud: {exception}", throttle_duration_sec=5.0)
            return
        message.header.frame_id = self.target_frame
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFrameRelay()
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
