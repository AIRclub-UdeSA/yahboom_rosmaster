#!/usr/bin/env python3
"""Static compatibility contract for the ROSMASTER X3 robot description."""

from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ElementTree


PACKAGE_DIR = Path(__file__).resolve().parents[1]
ROBOT_XACRO = (
    PACKAGE_DIR / "urdf" / "robots" / "rosmaster_x3.urdf.xacro"
)
WHEEL_ORIGINS = {
    "front_left": (0.08, 0.0845, -0.0325),
    "front_right": (0.08, -0.0845, -0.0325),
    "back_left": (-0.08, 0.0845, -0.0325),
    "back_right": (-0.08, -0.0845, -0.0325),
}
MESHES = {
    "base_link": "rosmaster_base.obj",
    "front_left_wheel_link": "front_left_wheel.obj",
    "front_right_wheel_link": "front_right_wheel.obj",
    "back_left_wheel_link": "back_left_wheel.obj",
    "back_right_wheel_link": "back_right_wheel.obj",
    "cam_1_link": "camera.obj",
    "laser_link": "lidar.obj",
}
VISUAL_ORIGINS = {
    "base_link": (0.0, 0.0, 0.0),
    "front_left_wheel_link": (0.0, 0.0, 0.0),
    "front_right_wheel_link": (0.0, 0.0, 0.0),
    "back_left_wheel_link": (0.0, 0.0, 0.0),
    "back_right_wheel_link": (0.0, 0.0, 0.0),
    "cam_1_link": (0.0, 0.0, 0.0),
    "laser_link": (0.0, 0.0, 0.0),
}
CAD_VISUAL_DIR = (
    PACKAGE_DIR / "meshes" / "rosmaster_x3" / "cad_visual"
)
LINK_NAMES = {
    "base_footprint",
    "base_link",
    "front_left_wheel_link",
    "front_right_wheel_link",
    "back_left_wheel_link",
    "back_right_wheel_link",
    "cam_1_link",
    "cam_1_depth_frame",
    "cam_1_depth_optical_frame",
    "cam_1_infra1_frame",
    "cam_1_infra1_optical_frame",
    "cam_1_infra2_frame",
    "cam_1_infra2_optical_frame",
    "cam_1_color_frame",
    "cam_1_color_optical_frame",
    "laser_link",
    "imu_link",
}
JOINT_NAMES = {
    "base_joint",
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "back_left_wheel_joint",
    "back_right_wheel_joint",
    "cam_1_joint",
    "cam_1_depth_joint",
    "cam_1_depth_optical_joint",
    "cam_1_infra1_joint",
    "cam_1_infra1_optical_joint",
    "cam_1_infra2_joint",
    "cam_1_infra2_optical_joint",
    "cam_1_color_joint",
    "cam_1_color_optical_joint",
    "laser_link_joint",
    "imu_joint",
}
SENSOR_JOINT_POSES = {
    "cam_1_joint": ((0.105, 0.0, 0.05), (0.0, 0.0, 0.0)),
    "cam_1_depth_joint": ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    "cam_1_depth_optical_joint": (
        (0.0, 0.0, 0.0),
        (-1.5707963267948966, 0.0, -1.5707963267948966),
    ),
    "cam_1_infra1_joint": ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    "cam_1_infra1_optical_joint": (
        (0.0, 0.0, 0.0),
        (-1.5707963267948966, 0.0, -1.5707963267948966),
    ),
    "cam_1_infra2_joint": ((0.0, -0.05, 0.0), (0.0, 0.0, 0.0)),
    "cam_1_infra2_optical_joint": (
        (0.0, 0.0, 0.0),
        (-1.5707963267948966, 0.0, -1.5707963267948966),
    ),
    "cam_1_color_joint": ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    "cam_1_color_optical_joint": (
        (0.0, 0.0, 0.0),
        (-1.5707963267948966, 0.0, -1.5707963267948966),
    ),
    "laser_link_joint": (
        (0.043, 0.0, 0.110),
        (0.0, 0.0, 3.141592653589793),
    ),
    "imu_joint": ((-0.06, 0.01, 0.01), (0.0, 3.1415, 1.5707)),
}


def vector(value):
    """Convert a three-component URDF attribute to floats."""
    return tuple(float(component) for component in value.split())


def expand(backend):
    """Expand one simulator variant to an ElementTree root."""
    result = subprocess.run(
        [
            "xacro",
            str(ROBOT_XACRO),
            "use_gazebo:=true",
            "robot_name:=rosmaster_x3",
            "prefix:=",
            f"sim_backend:={backend}",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return ElementTree.fromstring(result.stdout)


class TestRobotDescriptionContract(unittest.TestCase):
    """Protect interfaces while allowing the render geometry to evolve."""

    @classmethod
    def setUpClass(cls):
        cls.robots = {
            backend: expand(backend)
            for backend in ("fortress", "classic")
        }

    def test_link_joint_and_backend_contracts_are_unchanged(self):
        for backend, robot in self.robots.items():
            with self.subTest(backend=backend):
                self.assertEqual(
                    {link.get("name") for link in robot.findall("link")},
                    LINK_NAMES,
                )
                joints = {
                    joint.get("name"): joint
                    for joint in robot.findall("joint")
                }
                self.assertEqual(set(joints), JOINT_NAMES)
                wheel_type = (
                    "continuous" if backend == "fortress" else "fixed"
                )
                for side in WHEEL_ORIGINS:
                    joint = joints[f"{side}_wheel_joint"]
                    self.assertEqual(joint.get("type"), wheel_type)
                    self.assertEqual(joint.find("axis").get("xyz"), "0 1 0")

    def test_wheel_joints_match_real_robot_geometry(self):
        for backend, robot in self.robots.items():
            for side, expected in WHEEL_ORIGINS.items():
                with self.subTest(backend=backend, wheel=side):
                    joint = robot.find(
                        f"./joint[@name='{side}_wheel_joint']"
                    )
                    actual = vector(joint.find("origin").get("xyz"))
                    for actual_value, expected_value in zip(actual, expected):
                        self.assertAlmostEqual(actual_value, expected_value)
                    self.assertEqual(
                        vector(joint.find("origin").get("rpy")),
                        (0.0, 0.0, 0.0),
                    )
                    wheel_link = robot.find(
                        f"./link[@name='{side}_wheel_link']"
                    )
                    for element_name in ("collision", "inertial"):
                        origin = wheel_link.find(f"./{element_name}/origin")
                        self.assertEqual(
                            vector(origin.get("xyz")), (0.0, 0.0, 0.0)
                        )
                        self.assertEqual(
                            vector(origin.get("rpy")), (0.0, 0.0, 0.0)
                        )
                    self.assertAlmostEqual(
                        float(
                            wheel_link.find(
                                "./collision/geometry/sphere"
                            ).get("radius")
                        ),
                        abs(expected[2]),
                    )

    def test_fortress_drive_geometry_matches_physical_wheels(self):
        robot = self.robots["fortress"]
        drive = robot.find(
            ".//plugin[@name='ignition::gazebo::systems::MecanumDrive']"
        )
        self.assertIsNotNone(drive)
        front_left = WHEEL_ORIGINS["front_left"]
        front_right = WHEEL_ORIGINS["front_right"]
        back_left = WHEEL_ORIGINS["back_left"]
        expected = {
            "wheelbase": abs(front_left[0] - back_left[0]),
            "wheel_separation": abs(front_left[1] - front_right[1]),
            "wheel_radius": abs(front_left[2]),
        }
        for element_name, expected_value in expected.items():
            with self.subTest(element=element_name):
                self.assertAlmostEqual(
                    float(drive.findtext(element_name)), expected_value
                )

    def test_sensor_frames_and_mounts_are_unchanged(self):
        for backend, robot in self.robots.items():
            for name, (expected_xyz, expected_rpy) in (
                SENSOR_JOINT_POSES.items()
            ):
                with self.subTest(backend=backend, joint=name):
                    origin = robot.find(
                        f"./joint[@name='{name}']/origin"
                    )
                    self.assertEqual(vector(origin.get("xyz")), expected_xyz)
                    self.assertEqual(vector(origin.get("rpy")), expected_rpy)

    def test_fortress_camera_rate_is_five_hertz(self):
        camera = self.robots["fortress"].find(
            "./gazebo[@reference='cam_1_link']/sensor[@name='cam_1']"
        )
        self.assertIsNotNone(camera)
        self.assertEqual(float(camera.findtext("update_rate")), 5.0)

    def test_only_generated_installed_visuals_are_referenced(self):
        for backend, robot in self.robots.items():
            for link_name, filename in MESHES.items():
                with self.subTest(backend=backend, link=link_name):
                    visual = robot.find(
                        f"./link[@name='{link_name}']/visual"
                    )
                    origin = visual.find("origin")
                    self.assertEqual(
                        vector(origin.get("xyz")), VISUAL_ORIGINS[link_name]
                    )
                    self.assertEqual(
                        vector(origin.get("rpy")), (0.0, 0.0, 0.0)
                    )
                    self.assertIsNone(visual.find("material"))
                    mesh = visual.find("./geometry/mesh")
                    mesh_path = Path(
                        mesh.get("filename").removeprefix("file://")
                    )
                    self.assertEqual(mesh_path.name, filename)
                    self.assertTrue(
                        (CAD_VISUAL_DIR / filename).is_file(), filename
                    )
                    self.assertEqual(
                        vector(mesh.get("scale", "1 1 1")),
                        (1.0, 1.0, 1.0),
                    )
            for link_name in MESHES:
                override = robot.find(
                    f"./gazebo[@reference='{link_name}']/visual/material"
                )
                self.assertIsNone(override, link_name)

        cmake = (PACKAGE_DIR / "CMakeLists.txt").read_text(
            encoding="utf-8"
        )
        install_block = cmake.split("install(", 1)[1].split(")", 1)[0]
        self.assertIn("meshes", install_block)
        self.assertNotIn("cad", install_block)

    def test_base_collision_and_inertia_are_unchanged(self):
        for backend, robot in self.robots.items():
            with self.subTest(backend=backend):
                base = robot.find("./link[@name='base_link']")
                collision = base.find("collision")
                self.assertEqual(
                    vector(collision.find("./geometry/box").get("size")),
                    (0.3, 0.1386, 0.19724999999999998),
                )
                self.assertEqual(
                    vector(collision.find("origin").get("xyz")),
                    (-0.031, 0.0, 0.10674999999999998),
                )
                inertial = base.find("inertial")
                self.assertEqual(float(inertial.find("mass").get("value")), 1.5)
                self.assertEqual(
                    vector(inertial.find("origin").get("xyz")),
                    (-0.031, 0.0, 0.10674999999999998),
                )
                inertia = inertial.find("inertia")
                expected = {
                    "ixx": 0.007264690312499999,
                    "ixy": 0.0,
                    "ixz": 0.0,
                    "iyy": 0.0161134453125,
                    "iyz": 0.0,
                    "izz": 0.013651245,
                }
                self.assertEqual(
                    {
                        name: float(inertia.get(name))
                        for name in expected
                    },
                    expected,
                )

    def test_lidar_ignores_only_its_own_cad_housing(self):
        robot = self.robots["fortress"]
        laser_visual = robot.find("./gazebo[@reference='laser_link']/visual")
        self.assertEqual(laser_visual.findtext("visibility_flags"), "1")
        lidar = robot.find("./gazebo/sensor[@type='gpu_lidar']/lidar")
        self.assertEqual(lidar.findtext("visibility_mask"), "4294967294")

        for link_name in MESHES:
            if link_name == "laser_link":
                continue
            visual = robot.find(f"./gazebo[@reference='{link_name}']/visual")
            if visual is not None:
                self.assertIsNone(visual.find("visibility_flags"), link_name)

    def test_default_fortress_state_plugin_contract_is_unchanged(self):
        plugins = {
            plugin.get("name"): plugin.get("filename")
            for plugin in self.robots["fortress"].findall(".//plugin")
        }
        self.assertEqual(
            plugins.get("gz_ros2_control::GazeboSimROS2ControlPlugin"),
            "gz_ros2_control-system",
        )
        self.assertNotIn(
            "ignition::gazebo::systems::JointStatePublisher", plugins
        )


if __name__ == "__main__":
    unittest.main()
