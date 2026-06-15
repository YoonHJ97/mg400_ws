"""MG400 bringup launch.

로봇과 통신하는 유일한 노드 mg400_driver 를 config/mg400_driver.yaml 파라미터로
실행한다. 상태(토픽) / 모션(액션) / 모드(서비스) 인터페이스를 모두 이 노드가 제공하며,
다른 노드(AMR 포함)는 이 인터페이스로만 대화한다.

    ros2 launch mg400_bringup mg400.launch.py
    ros2 launch mg400_bringup mg400.launch.py robot_ip:=192.168.1.10
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare('mg400_bringup'), 'config', 'mg400_driver.yaml']
    )
    robot_ip = LaunchConfiguration('robot_ip')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip', default_value='192.168.1.6',
            description='MG400 IP 주소 (config 값을 덮어씀)'),

        Node(
            package='mg400_driver',
            executable='mg400_driver',
            name='mg400_driver',
            output='screen',
            parameters=[params_file, {'robot_ip': robot_ip}],
        ),
    ])
