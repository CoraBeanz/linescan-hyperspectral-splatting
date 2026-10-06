"""Bring up the SO-101 scan arm.

    # no hardware at all: mock arm, simulated mirror ESP32
    ros2 launch so101_scan_bringup scan_arm.launch.py use_mock_hardware:=true mirror:=fake

    # the real arm with its motors off, to check the calibration by moving it by hand
    ros2 launch so101_scan_bringup scan_arm.launch.py torque:=false mirror:=fake

    # the real arm and the real mirror
    ros2 launch so101_scan_bringup scan_arm.launch.py

What starts:
  robot_state_publisher   the URDF (so101_scan_description) and TF for every frame
  ros2_control_node       the STS3215 driver (or mock hardware) at 100 Hz, with
                          joint_state_broadcaster -> /joint_states and arm_controller,
                          a trajectory controller (not started with torque:=false)
  scan_mirror_bridge      talks to the mirror ESP32; publishes the mirror angle and scan lines
  fake_scan_mirror        with mirror:=fake, a simulated ESP32 on a pseudo-terminal
  foxglove_bridge, rviz2  optional viewers

Arguments: see DeclareLaunchArgument below, or `ros2 launch so101_scan_bringup
scan_arm.launch.py --show-args`.
"""

import os

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

DATA_DIR = os.environ.get("SO101_SCAN_DATA", os.path.expanduser("~/so101_scan"))
FAKE_MIRROR_LINK = "/tmp/scan_mirror_fake"


def _true(text):
    return text.strip().lower() in ("1", "true", "yes", "on")


def _setup(context):
    arg = {name: LaunchConfiguration(name).perform(context) for name in (
        "use_mock_hardware", "port", "calibration_file", "torque", "mirror", "mirror_port",
        "mirror_config", "foxglove", "rviz")}
    description = get_package_share_directory("so101_scan_description")
    bringup = get_package_share_directory("so101_scan_bringup")
    mock = _true(arg["use_mock_hardware"])
    torque = _true(arg["torque"])
    calibration = os.path.expanduser(arg["calibration_file"])
    if not mock and not os.path.exists(calibration):
        raise RuntimeError(
            "No calibration file at %s. Run `ros2 run so101_scan_hardware sts_calibrate` first, "
            "pass calibration_file:=..., or use use_mock_hardware:=true." % calibration)

    mappings = {"use_mock_hardware": str(mock).lower(), "port": arg["port"], "torque": str(torque).lower()}
    if not mock:
        mappings["calibration_file"] = calibration
    urdf = xacro.process_file(os.path.join(description, "urdf", "so101_scan.urdf.xacro"),
                              mappings=mappings).toxml()
    robot_description = ParameterValue(urdf, value_type=str)

    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_description,
                          # TF for moving joints at the 100 Hz of /joint_states, not the default 20 Hz,
                          # so poses looked up at a scan line's time are interpolated from close samples
                          "publish_frequency": 100.0}]),
        Node(package="controller_manager", executable="ros2_control_node", output="screen",
             parameters=[os.path.join(bringup, "config", "controllers.yaml")],
             remappings=[("~/robot_description", "/robot_description")]),
        Node(package="controller_manager", executable="spawner", output="screen",
             arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]),
    ]
    if torque or mock:
        nodes.append(Node(package="controller_manager", executable="spawner", output="screen",
                          arguments=["arm_controller", "--controller-manager", "/controller_manager"]))

    mirror = arg["mirror"]
    if mirror not in ("esp32", "fake"):
        raise RuntimeError("mirror:=%s; use esp32 or fake" % mirror)
    mirror_port = arg["mirror_port"]
    if mirror == "fake":
        mirror_port = FAKE_MIRROR_LINK
        nodes.append(Node(package="so101_scan_sweep", executable="fake_scan_mirror", output="screen",
                          arguments=["--link", FAKE_MIRROR_LINK]))
    nodes.append(Node(package="so101_scan_sweep", executable="scan_mirror_bridge", output="screen",
                      parameters=[arg["mirror_config"], {"port": mirror_port}]))

    if _true(arg["foxglove"]):
        nodes.append(Node(package="foxglove_bridge", executable="foxglove_bridge", output="screen",
                          parameters=[{"port": 8765}]))
    if _true(arg["rviz"]):
        nodes.append(Node(package="rviz2", executable="rviz2", output="log",
                          arguments=["-d", os.path.join(description, "rviz", "so101_scan.rviz")]))
    return nodes


def generate_launch_description():
    bringup = get_package_share_directory("so101_scan_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="true: no servos, commands become positions instantly"),
        DeclareLaunchArgument("port", default_value="/dev/so101",
                              description="servo bus adapter (see ros2/udev)"),
        DeclareLaunchArgument("calibration_file", default_value=os.path.join(DATA_DIR, "calibration.yaml"),
                              description="from sts_calibrate; $SO101_SCAN_DATA is ~/so101_scan by default"),
        DeclareLaunchArgument("torque", default_value="true",
                              description="false: motors off, positions only read; move the arm by hand"),
        DeclareLaunchArgument("mirror", default_value="esp32", description="esp32 or fake"),
        DeclareLaunchArgument("mirror_port", default_value="/dev/scan_mirror",
                              description="the mirror ESP32's USB serial port"),
        DeclareLaunchArgument("mirror_config", default_value=os.path.join(bringup, "config", "scan_mirror.yaml")),
        DeclareLaunchArgument("foxglove", default_value="false",
                              description="start foxglove_bridge on port 8765, to view from another computer"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_setup),
    ])
