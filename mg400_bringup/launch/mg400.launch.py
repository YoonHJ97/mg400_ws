"""MG400 bringup launch.

상태 발행 노드(mg400_status_node)를 config/mg400_params.yaml 파라미터로 실행한다.
명령 서버(move_service_server)는 enable_service 인자로 켤 수 있다.

    ros2 launch mg400_bringup mg400.launch.py
    ros2 launch mg400_bringup mg400.launch.py robot_ip:=192.168.1.10
    ros2 launch mg400_bringup mg400.launch.py enable_service:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare('mg400_bringup'), 'config', 'mg400_params.yaml']
    )

    robot_ip = LaunchConfiguration('robot_ip')
    enable_service = LaunchConfiguration('enable_service')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip', default_value='192.168.1.6',
            description='MG400 IP 주소 (config 값을 덮어씀)'),
        DeclareLaunchArgument(
            'enable_service', default_value='false',
            description='move_service_server(MoveTo) 함께 실행 여부'),

        # 상태 발행 노드: yaml 파라미터 + robot_ip 오버라이드
        Node(
            package='mg400_pkg',
            executable='mg400_status_node',
            name='mg400_status_node',
            output='screen',
            parameters=[params_file, {'robot_ip': robot_ip}],
        ),

        # 명령 서버 (옵션)
        Node(
            package='mg400_pkg',
            executable='move_service_server',
            name='move_service_server',
            output='screen',
            condition=IfCondition(enable_service),
        ),
    ])
