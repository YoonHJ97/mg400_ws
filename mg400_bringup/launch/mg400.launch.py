"""MG400 bringup launch.

상태 발행 노드(mg400_status_node)를 config/mg400_params.yaml 파라미터로 실행한다.
bringup 연동 모션 제어 노드(mg400_move_node)는 enable_move 인자로 켤 수 있다.
이 노드는 feed(30004)를 직접 열지 않고 status_node 의 tcp_pose 를 구독해 도착을
판정하므로 status_node 와 함께 떠야 한다 (포트 충돌 없음).

기존 move_node / move_service_server 는 자체적으로 feed 를 열고 로봇을 점유하는
독립 노드이므로 bringup 에 포함하지 않는다 (별도로 단독 실행).

    ros2 launch mg400_bringup mg400.launch.py
    ros2 launch mg400_bringup mg400.launch.py robot_ip:=192.168.1.10
    ros2 launch mg400_bringup mg400.launch.py enable_move:=true
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
    enable_move = LaunchConfiguration('enable_move')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip', default_value='192.168.1.6',
            description='MG400 IP 주소 (config 값을 덮어씀)'),
        DeclareLaunchArgument(
            'enable_move', default_value='false',
            description='bringup 연동 모션 제어 노드(mg400_move_node) 실행 여부'),

        # 상태 발행 노드: yaml 파라미터 + robot_ip 오버라이드 (feed 단일 소유자)
        Node(
            package='mg400_pkg',
            executable='mg400_status_node',
            name='mg400_status_node',
            output='screen',
            parameters=[params_file, {'robot_ip': robot_ip}],
        ),

        # 모션 제어 노드 (옵션): tcp_pose 구독으로 도착 판정, 명령만 전송
        Node(
            package='mg400_pkg',
            executable='mg400_move_node',
            name='mg400_move_node',
            output='screen',
            parameters=[params_file, {'robot_ip': robot_ip}],
            condition=IfCondition(enable_move),
        ),
    ])
