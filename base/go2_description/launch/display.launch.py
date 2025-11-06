from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

# 默认情况下，joint_state_publisher 节点会启动：
# ros2 launch go2_description display.launch.py

# 如果你想禁用 joint_state_publisher 节点，可以在启动时传入 use_joint_state_publisher 参数：
# ros2 launch go2_description display.launch.py use_joint_state_publisher:=false

def generate_launch_description():
    go2_description_dir = get_package_share_directory("go2_description")
    default_model_path = os.path.join(go2_description_dir, "urdf", "go2_description.urdf")

    # 声明一个布尔参数，用于控制是否启动 joint_state_publisher
    use_joint_state_publisher = DeclareLaunchArgument(
        name="use_joint_state_publisher",
        default_value="true",  # 默认值为 true，表示默认启动 joint_state_publisher
        description="Whether to launch the joint_state_publisher node"
    )

    model = DeclareLaunchArgument(name="model", default_value=default_model_path)

    # 加载机器人模型
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # 根据传入的布尔参数决定是否启动 joint_state_publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition=IfCondition(LaunchConfiguration("use_joint_state_publisher"))
    )

    return LaunchDescription([
        model,
        use_joint_state_publisher,
        robot_state_publisher,
        joint_state_publisher,
    ])