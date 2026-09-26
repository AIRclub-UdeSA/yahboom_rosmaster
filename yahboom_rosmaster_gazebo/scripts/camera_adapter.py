#!/usr/bin/env python3
"""
Simulator twin of the physical X3's Astra sensor adapter.

Gazebo's RGB-D camera renders color and depth from one aperture, at
``cam_1_color_frame``. The physical Astra registers its depth to color, so
the same is true there: pixel (u, v) of the depth image and of the color image
is one ray. Each depth image is conditioned and published on its own, as
yahboomcar_astra's ``sensor_adapter.py`` republishes it, whether or not its
color partner or a ``camera_info`` ever arrives. The adapter also pairs it with
the color image of the same stamp, back-projects that same conditioned depth
with the intrinsics in ``camera_info``, attaches the color, transforms the
points into ``cam_1_depth_frame`` and packs them into the layout that
``sensor_adapter.py`` publishes: 16-byte x, y, z and rgb points, with the
non-finite ones stripped (``cloud_strip_nan``) and every ``cloud_decimation``-th
row and column kept.

The arithmetic and the frame handling need no running ROS graph, so they are
unit tested on their own. ``CameraAdapter``, at the end, wires them to topics.
"""

from collections import OrderedDict
import math
import os

# One thread, set before numpy loads. Should any call reach BLAS, its spinning
# threads would take cores from the Gazebo server (see _rotate_translate).
for _name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS"):
    os.environ.setdefault(_name, "1")

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402
from tf2_ros import Buffer, TransformException  # noqa: E402

from cloud_timing import (  # noqa: E402
    FrameGate,
    GapSampler,
    publish_delay,
    resolve_seed,
)


DEPTH_ENCODING = "32FC1"
# The source channel of each output channel, for the color encodings taken.
COLOR_CHANNELS = {"rgb8": (0, 1, 2), "bgr8": (2, 1, 0)}
POINT_STEP = 16
RGB_OFFSET = 12
# One point as it is packed on the wire. The rgb field is a FLOAT32 whose bits
# are the 0x00RRGGBB integer of the ROS and PCL convention, so it is stored as
# the unsigned integer with those bits.
POINT_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])


def _pixels(message, bytes_per_pixel, dtype, channels=None):
    """
    Return a view of an image's pixels, honouring the row stride.

    A row may be followed by padding: ``step`` can exceed ``width`` pixels.
    The view skips it rather than assuming the rows are packed.
    """
    if message.width == 0 or message.height == 0:
        raise ValueError("image dimensions must be non-zero")
    if message.step < message.width * bytes_per_pixel:
        raise ValueError("image step is shorter than one row of pixels")
    if len(message.data) < message.step * message.height:
        raise ValueError("truncated image")
    if channels is None:
        shape = (message.height, message.width)
        strides = (message.step, bytes_per_pixel)
    else:
        shape = (message.height, message.width, channels)
        strides = (message.step, bytes_per_pixel, 1)
    return np.ndarray(shape=shape, dtype=dtype, buffer=message.data, strides=strides)


def depth_array(message):
    """Return the depth image as a (height, width) view in metres."""
    if message.encoding != DEPTH_ENCODING:
        raise ValueError(f"unsupported depth encoding: {message.encoding}")
    return _pixels(message, 4, ">f4" if message.is_bigendian else "<f4")


def color_array(message):
    """Return the color image as a (height, width, 3) view in R, G, B order."""
    if message.encoding not in COLOR_CHANNELS:
        raise ValueError(f"unsupported color encoding: {message.encoding}")
    pixels = _pixels(message, 3, np.uint8, channels=3)
    if message.encoding == "rgb8":
        return pixels
    return pixels[:, :, COLOR_CHANNELS[message.encoding]]


def depth_image_message(header, depth):
    """Return a tightly packed 32FC1 image message of a (height, width) array."""
    depth = np.asarray(depth)
    message = Image()
    message.header = header
    message.height, message.width = depth.shape
    message.encoding = DEPTH_ENCODING
    message.is_bigendian = False
    message.step = message.width * 4
    # Extend the message's own array in place. Its data property would check
    # every byte on assignment first, which costs as much as the copy.
    message.data.frombytes(np.ascontiguousarray(depth, dtype="<f4").view(np.uint8))
    return message


def condition_depth(depth):
    """
    Return the depth image that the sensor profile publishes and builds clouds from.

    Step 7 of #43 fills this in with the physical camera's scale error, noise
    and NaN below its minimum range. It runs once on every depth image as it
    arrives, needing neither its color partner nor ``camera_info``, and the
    public depth image and the cloud are both built from its result, so that
    the two always come from the same degraded frame. It does nothing for now.
    """
    return depth


class CameraRays:
    """
    The direction of every pixel's ray in the camera's optical frame.

    A pixel (u, v) at depth z lies at ((u - cx) / fx * z, (v - cy) / fy * z, z).
    The factors do not depend on the frame, so they are computed once for an
    intrinsic matrix. Each is separable: one per column and one per row.
    """

    def __init__(self, info):
        if info.width == 0 or info.height == 0:
            raise ValueError("camera_info dimensions must be non-zero")
        if not all(math.isfinite(value) for value in info.k):
            raise ValueError("camera_info has no valid intrinsic matrix")
        fx, cx, fy, cy = info.k[0], info.k[2], info.k[4], info.k[5]
        if fx <= 0.0 or fy <= 0.0:
            raise ValueError("camera_info focal lengths must be positive")
        self.width = int(info.width)
        self.height = int(info.height)
        self.key = self.key_of(info)
        self.x = (np.arange(self.width, dtype=np.float64) - cx) / fx
        self.y = (np.arange(self.height, dtype=np.float64) - cy) / fy

    @staticmethod
    def key_of(info):
        """Return what the rays depend on."""
        return (int(info.width), int(info.height), tuple(info.k))

    @classmethod
    def for_info(cls, info, current=None):
        """Return ``current`` while it still fits ``info``, otherwise new rays."""
        if current is not None and current.key == cls.key_of(info):
            return current
        return cls(info)


def back_project(depth, rays, decimation=1):
    """
    Return the (N, 3) float64 points of every ``decimation``-th pixel, row by row.

    ``depth`` is a (height, width) array in metres and the points are in the
    optical frame the rays are in. Non-finite depths give non-finite points.
    """
    if depth.shape != (rays.height, rays.width):
        raise ValueError(
            f"depth image is {depth.shape[1]}x{depth.shape[0]} but camera_info "
            f"says {rays.width}x{rays.height}")
    z = np.asarray(depth[::decimation, ::decimation], dtype=np.float64)
    points = np.empty((z.size, 3), dtype=np.float64)
    # A ray component of zero times an infinite depth is NaN, which is as
    # non-finite as the infinity, so the warning says nothing useful.
    with np.errstate(invalid="ignore"):
        points[:, 0] = (rays.x[::decimation][np.newaxis, :] * z).ravel()
        points[:, 1] = (rays.y[::decimation][:, np.newaxis] * z).ravel()
    points[:, 2] = z.ravel()
    return points


def pack_colour(rgb, decimation=1):
    """
    Return the ``rgb`` field value of every ``decimation``-th pixel, row by row.

    Each is the integer 0x00RRGGBB, which stored little-endian is the bytes
    blue, green, red, zero: the ROS and PCL convention, and Gazebo's own.
    """
    kept = rgb[::decimation, ::decimation].astype(np.uint32)
    packed = (kept[:, :, 0] << 16) | (kept[:, :, 1] << 8) | kept[:, :, 2]
    return packed.ravel()


def rotation_matrix(quaternion):
    """Return the rotation matrix of an (x, y, z, w) quaternion, normalized first."""
    x, y, z, w = (float(value) for value in quaternion)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("invalid transform quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _rotate_translate(x, y, z, rotation, translation):
    """
    Return the x, y and z columns rotated then translated.

    Element-wise rather than a matrix product: numpy hands ``@`` to a
    multithreaded BLAS, and in #49 its spinning threads took 8.5 cores from the
    Gazebo server and cut the real-time factor to 0.77.
    """
    return [
        row[0] * x + row[1] * y + row[2] * z + offset
        for row, offset in zip(rotation, translation)
    ]


def pack_cloud(
        points, colour, rotation, translation, frame_id, stamp, height, width,
        strip_nan):
    """
    Return the points, moved into ``frame_id`` and packed to 16 bytes each.

    ``points`` is (N, 3) with N = ``height`` * ``width``, and ``colour`` the
    matching N ``rgb`` field values. As in the physical adapter's
    ``transform_cloud()``, a point is finite only if all of x, y and z are, and
    only finite points are transformed. A non-finite one is left exactly as it
    came: with ``strip_nan`` it is dropped and the cloud is unorganized
    (``height`` 1) and dense, and without it the cloud keeps the image's
    organized shape and is dense only if nothing was non-finite.
    """
    points = np.asarray(points, dtype=np.float64)
    colour = np.asarray(colour, dtype=np.uint32)
    rotation = np.asarray(rotation, dtype=np.float64)
    translation = np.asarray(translation, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] != height * width:
        raise ValueError("points do not match the cloud dimensions")
    if colour.shape != (len(points),):
        raise ValueError("colour does not match the points")
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("invalid transform rotation")
    if translation.shape != (3,) or not np.all(np.isfinite(translation)):
        raise ValueError("invalid transform translation")

    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    # A non-finite coordinate times a rotation entry of zero is NaN, which is
    # as non-finite as what it came from, so the warning says nothing useful.
    with np.errstate(invalid="ignore", over="ignore"):
        if strip_nan:
            x, y, z, colour = x[finite], y[finite], z[finite], colour[finite]
            moved = _rotate_translate(x, y, z, rotation, translation)
            height, width = 1, len(x)
            is_dense = True
        else:
            moved = _rotate_translate(x, y, z, rotation, translation)
            # A point that is not finite is left exactly as it came.
            moved = [np.where(finite, new, old) for new, old in zip(moved, (x, y, z))]
            is_dense = bool(finite.all())

    packed = np.empty(len(colour), dtype=POINT_DTYPE)
    packed["x"], packed["y"], packed["z"] = moved
    packed["rgb"] = colour

    cloud = PointCloud2()
    cloud.header.stamp = stamp
    cloud.header.frame_id = frame_id
    cloud.height = height
    cloud.width = width
    cloud.is_dense = is_dense
    cloud.is_bigendian = False
    cloud.point_step = POINT_STEP
    cloud.row_step = POINT_STEP * width
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=RGB_OFFSET, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.data.frombytes(packed.view(np.uint8))
    return cloud


def stamp_seconds(stamp):
    """Return a builtin_interfaces Time as float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


class StampPairer:
    """
    Match color and depth frames that carry exactly the same stamp.

    Gazebo renders both images from one camera in one step, so a frame's two
    images share a stamp but reach ROS as separate messages. Each is held until
    its partner arrives. Once a pair completes, anything older that is still
    waiting can never pair, and is discarded and counted. What is held is
    anything with a ``header``: the color ``Image`` and the ``ConditionedDepth``.
    """

    def __init__(self, capacity=8):
        self._capacity = capacity
        self._pending = {"color": OrderedDict(), "depth": OrderedDict()}
        self.discarded = 0

    @staticmethod
    def _key(item):
        return (item.header.stamp.sec, item.header.stamp.nanosec)

    def add(self, kind, item):
        """Hold a color or depth frame, and return (color, depth) once both have arrived."""
        key = self._key(item)
        other = "depth" if kind == "color" else "color"
        partner = self._pending[other].pop(key, None)
        if partner is None:
            self._pending[kind][key] = item
            while len(self._pending[kind]) > self._capacity:
                self._pending[kind].popitem(last=False)
                self.discarded += 1
            return None
        for waiting in self._pending.values():
            for stale in [held for held in waiting if held < key]:
                del waiting[stale]
                self.discarded += 1
        return (item, partner) if kind == "color" else (partner, item)


class ConditionedDepth:
    """A depth image after conditioning: its header, and its (height, width) metres."""

    def __init__(self, header, pixels):
        self.header = header
        self.pixels = pixels


class FramePipeline:
    """
    Everything the adapter does to depth images and clouds, apart from talking to ROS.

    ``condition`` takes each depth image on its own, so every depth image can
    be published. ``build_cloud`` takes a color image and the conditioned depth
    it paired with, and the gate says which of those become clouds, so a cloud
    is built only for the frames that are delivered.
    """

    def __init__(
            self, gate, decimation=1, strip_nan=True,
            target_frame="cam_1_depth_frame", condition=condition_depth):
        if isinstance(decimation, bool) or not isinstance(decimation, int) or decimation < 1:
            raise ValueError("point cloud decimation must be a positive integer")
        self._gate = gate
        self._decimation = decimation
        self._strip_nan = strip_nan
        self._target_frame = target_frame
        self._condition = condition
        self._rays = None
        self._rotation = None
        self._translation = None

    @property
    def has_transform(self):
        """Whether the depth-frame transform has been set."""
        return self._rotation is not None

    def set_transform(self, translation, quaternion):
        """Set the static transform from the image's optical frame to the cloud's."""
        self._rotation = rotation_matrix(quaternion)
        self._translation = np.asarray(tuple(translation), dtype=np.float64)
        if self._translation.shape != (3,) or not np.all(np.isfinite(self._translation)):
            self._rotation = None
            raise ValueError("invalid transform translation")

    def condition(self, depth):
        """
        Return the ConditionedDepth of one depth image message.

        It reads the image alone, needing no ``camera_info``. Raises ValueError
        if the image is malformed. The caller drops it.
        """
        return ConditionedDepth(depth.header, self._condition(depth_array(depth)))

    def build_cloud(self, color, depth, info):
        """
        Return the cloud of a color image and its conditioned depth, or None.

        The cloud is built from ``depth.pixels``, the array the depth image was
        published from. None means the gate does not deliver this frame, or the
        depth-frame transform has not been set. Raises ValueError if the color
        image is malformed or the frame does not match ``camera_info``. The
        caller drops the cloud, and a frame like that does not count for the
        gate, so a cloud the gate owes is not lost to it.
        """
        color_pixels = color_array(color)
        if color_pixels.shape[:2] != depth.pixels.shape:
            raise ValueError("color and depth images differ in size")
        self._rays = CameraRays.for_info(info, self._rays)
        if depth.pixels.shape != (self._rays.height, self._rays.width):
            raise ValueError("images do not match camera_info's size")

        stamp = depth.header.stamp
        if not self.has_transform or not self._gate.deliver(stamp_seconds(stamp)):
            return None

        points = back_project(depth.pixels, self._rays, self._decimation)
        colour = pack_colour(color_pixels, self._decimation)
        kept = depth.pixels[::self._decimation, ::self._decimation].shape
        return pack_cloud(
            points, colour, self._rotation, self._translation, self._target_frame,
            stamp, kept[0], kept[1], self._strip_nan)


class AdapterCore:
    """
    What the adapter does with each message, with its outputs passed in.

    ``publish_depth`` and ``publish_cloud`` take a finished message, and
    ``warn`` takes a line for a throttled log. The node supplies them, and the
    unit tests supply recorders, so none of this needs a running ROS graph.

    A depth image is conditioned and published the moment it arrives, before
    and whether or not its color partner or a ``camera_info`` comes, as on the
    robot. The conditioned array is then kept for its partner, and the cloud is
    built from that same array. A depth image that is malformed is dropped
    whole. Anything wrong with a frame's color or its size against
    ``camera_info`` drops only that frame's cloud, and warns.
    """

    def __init__(self, pipeline, publish_depth, publish_cloud, warn, pairer=None):
        self._pipeline = pipeline
        self._publish_depth = publish_depth
        self._publish_cloud = publish_cloud
        self._warn = warn
        self._pairer = pairer or StampPairer()
        self._reported_discards = 0
        self._info = None

    @property
    def discarded(self):
        """How many images never found a partner with their stamp."""
        return self._pairer.discarded

    def on_camera_info(self, info):
        """Keep the latest camera intrinsics."""
        self._info = info

    def on_color(self, message):
        """Take a color image."""
        self._pair("color", message)

    def on_depth(self, message):
        """Condition a depth image, publish it, and take it for pairing."""
        try:
            depth = self._pipeline.condition(message)
        except ValueError as error:
            self._warn(f"Dropping invalid depth image: {error}")
            return
        self._publish_depth(depth_image_message(depth.header, depth.pixels))
        self._pair("depth", depth)

    def _pair(self, kind, held):
        pair = self._pairer.add(kind, held)
        if self._pairer.discarded != self._reported_discards:
            self._reported_discards = self._pairer.discarded
            self._warn(
                f"{self._reported_discards} images so far never found a color or "
                "depth partner with their stamp")
        if pair is None:
            return
        if self._info is None:
            self._warn("No cloud for a frame: no camera_info has arrived yet")
            return
        color, depth = pair
        try:
            cloud = self._pipeline.build_cloud(color, depth, self._info)
        except ValueError as error:
            self._warn(f"Dropping the cloud of an invalid frame: {error}")
            return
        if cloud is not None:
            self._publish_cloud(cloud)


STATS_PERIOD_S = 30.0


class CameraAdapter(Node):
    """Publish the public depth image and point cloud from the bridged images."""

    def __init__(self):
        super().__init__("camera_adapter")
        self.declare_parameter("cloud_decimation", 1)
        self.declare_parameter("cloud_strip_nan", True)
        self.declare_parameter("target_cloud_frame", "cam_1_depth_frame")
        # The timing model. The defaults are the ideal profile; the launch file
        # passes the selected profile from config/sensor_profiles.yaml.
        self.declare_parameter("latency_s", 0.0)
        self.declare_parameter("frame_period_s", 0.033)
        self.declare_parameter("gap_frames", [1])
        self.declare_parameter("gap_probabilities", [1.0])
        self.declare_parameter("seed", -1)

        def parameter(name):
            return self.get_parameter(name).value

        gap_frames = list(parameter("gap_frames"))
        gap_probabilities = list(parameter("gap_probabilities"))
        if len(gap_frames) != len(gap_probabilities):
            raise ValueError("gap_frames and gap_probabilities differ in length")
        self.latency_s = float(parameter("latency_s"))
        if not math.isfinite(self.latency_s) or self.latency_s < 0.0:
            raise ValueError("latency_s must be zero or positive")
        seed = resolve_seed(parameter("seed"))
        self.get_logger().info(
            f"Point cloud gaps drawn from seed {seed}, latency {self.latency_s:.3f} s")
        gate = FrameGate(
            GapSampler(dict(zip(gap_frames, gap_probabilities)), seed),
            float(parameter("frame_period_s")))
        self.target_frame = str(parameter("target_cloud_frame"))
        self.pipeline = FramePipeline(
            gate,
            decimation=int(parameter("cloud_decimation")),
            strip_nan=bool(parameter("cloud_strip_nan")),
            target_frame=self.target_frame)
        self.core = AdapterCore(
            self.pipeline,
            self._publish_depth,
            self._schedule_cloud,
            lambda text: self.get_logger().warning(text, throttle_duration_sec=5.0))

        self.depth_publisher = self.create_publisher(
            Image, "/cam_1/depth/image_raw", qos_profile_sensor_data)
        self.cloud_publisher = self.create_publisher(
            PointCloud2, "/cam_1/depth/color/points", qos_profile_sensor_data)

        # Both frames hang off cam_1_link on fixed joints, so the transform is
        # static. Feed a buffer from /tf_static only rather than a full
        # listener that would also decode every /tf message.
        self.tf_buffer = Buffer()
        self.create_subscription(
            TFMessage, "/tf_static", self._store_static_transforms,
            QoSProfile(
                depth=100,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ))

        # ros_gz_image cannot publish Best Effort, so the images arrive
        # Reliable on private /internal/ names.
        reliable = QoSProfile(
            depth=10, durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            CameraInfo, "/internal/cam_1/color/camera_info",
            self.core.on_camera_info, reliable)
        self.create_subscription(
            Image, "/internal/cam_1/color/image_raw", self.core.on_color, reliable)
        self.create_subscription(
            Image, "/internal/cam_1/depth/image_raw", self._on_depth, reliable)

        self.frames = 0
        self.clouds = 0
        self.late_clouds = 0
        self.create_timer(STATS_PERIOD_S, self.log_summary)
        self.get_logger().info(
            f"Building {self.target_frame} clouds from the color and depth images")

    def _store_static_transforms(self, message):
        for transform in message.transforms:
            self.tf_buffer.set_transform_static(transform, "tf_static")

    def _resolve_transform(self, image_frame):
        """Cache the static target <- image frame transform once TF has it."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, image_frame, Time()).transform
        except TransformException as exception:
            self.get_logger().warning(
                f"No cloud until {self.target_frame} <- {image_frame} is on "
                f"/tf_static: {exception}", throttle_duration_sec=5.0)
            return
        self.pipeline.set_transform(
            (transform.translation.x, transform.translation.y, transform.translation.z),
            (transform.rotation.x, transform.rotation.y, transform.rotation.z,
             transform.rotation.w))
        self.get_logger().info(
            f"{self.target_frame} <- {image_frame}: translation "
            f"({transform.translation.x:.6f}, {transform.translation.y:.6f}, "
            f"{transform.translation.z:.6f})")

    def _on_depth(self, message):
        if not self.pipeline.has_transform:
            self._resolve_transform(message.header.frame_id)
        self.core.on_depth(message)

    def _publish_depth(self, message):
        self.frames += 1
        self.depth_publisher.publish(message)

    def _schedule_cloud(self, cloud):
        """Publish a cloud at its capture stamp plus the latency, or now if late."""
        now = self.get_clock().now().nanoseconds * 1e-9
        delay, late = publish_delay(
            stamp_seconds(cloud.header.stamp), now, self.latency_s)
        self.clouds += 1
        self.late_clouds += late
        if delay <= 0.0:
            self.cloud_publisher.publish(cloud)
            return
        timers = []

        def publish():
            timer = timers.pop()
            timer.cancel()
            self.destroy_timer(timer)
            self.cloud_publisher.publish(cloud)

        timers.append(self.create_timer(delay, publish))

    def summary(self):
        """Return how many frames and clouds were published, and how many clouds were late."""
        if not self.frames:
            return None
        late = ""
        if self.latency_s > 0.0 and self.clouds:
            late = (
                f", {self.late_clouds} of them ready after stamp + "
                f"{self.latency_s * 1000:.0f} ms ({100.0 * self.late_clouds / self.clouds:.1f}%) "
                "and published at once")
        return (
            f"{self.frames} frames and {self.clouds} clouds published so far{late}; "
            f"{self.core.discarded} images never found a partner with their stamp")

    def log_summary(self):
        """Log the summary."""
        text = self.summary()
        if text:
            self.get_logger().info(text)


def main(args=None):
    """Run the adapter."""
    rclpy.init(args=args)
    node = CameraAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # Plain text: the signal that ended the spin has already shut the ROS
        # context down, so the logger could not publish it.
        text = node.summary()
        if text:
            print(f"[camera_adapter] {text}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
