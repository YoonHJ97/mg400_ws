#!/usr/bin/env python3
"""MG400 상태 발행 노드.

이전 Dobot_ws/dobot/tcp_monitor.py 를 ROS2 노드로 발전시킨 버전.
30004(feed) 포트의 1440바이트 피드백 패킷을 파싱해서 print 대신 토픽으로 발행한다.

발행 토픽:
    ~/joint_states  (sensor_msgs/JointState)   q_actual -> 4관절 각도
    ~/tcp_pose      (geometry_msgs/PoseStamped) tool_vector_actual -> TCP 위치/자세
    ~/robot_mode    (std_msgs/Int32)            robot_mode 값
"""
import math
import threading
from time import sleep

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from tf_transformations import quaternion_from_euler

from mg400_pkg.dobot_api import DobotApi, MyType

# 피드백 패킷 무결성 검증용 매직 넘버
FEED_MAGIC = '0x123456789abcdef'
PACKET_SIZE = 1440


class MG400StatusNode(Node):
    def __init__(self):
        super().__init__('mg400_status_node')

        # --- 파라미터 (bringup config/mg400_params.yaml 에서 주입) ---
        self.declare_parameter('robot_ip', '192.168.1.6')
        self.declare_parameter('feed_port', 30004)
        self.declare_parameter('publish_rate', 10.0)            # Hz
        self.declare_parameter('base_frame', 'mg400_base_link')
        self.declare_parameter('joint_names',
                               ['joint1', 'joint2', 'joint3', 'joint4'])
        # 로봇은 각도(deg)/mm 로 보고 -> ROS 관례(rad/m)로 변환
        self.declare_parameter('joints_in_degrees', True)

        self.ip = self.get_parameter('robot_ip').value
        self.feed_port = self.get_parameter('feed_port').value
        rate = self.get_parameter('publish_rate').value
        self.base_frame = self.get_parameter('base_frame').value
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.deg2rad = self.get_parameter('joints_in_degrees').value

        # --- feed 소켓 연결 ---
        self.get_logger().info(
            f'MG400 feed 연결 시도: {self.ip}:{self.feed_port}')
        try:
            self.feed = DobotApi(self.ip, self.feed_port)
        except Exception as e:
            self.get_logger().error(f'feed 포트 연결 실패: {e}')
            raise
        self.get_logger().info('MG400 feed 연결 성공.')

        # --- 발행자 ---
        self.joint_pub = self.create_publisher(JointState, '~/joint_states', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '~/tcp_pose', 10)
        self.mode_pub = self.create_publisher(Int32, '~/robot_mode', 10)

        # --- 최신 피드백 공유 상태 ---
        self._lock = threading.Lock()
        self._latest = None
        self._running = True

        # 고속 수신은 스레드, 발행은 타이머로 분리
        self._reader = threading.Thread(target=self._feed_loop, daemon=True)
        self._reader.start()

        self.timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(f'상태 발행 시작 ({rate:.1f} Hz).')

    def _feed_loop(self):
        """30004 피드백을 끊김 없이 수신해 최신 패킷만 보관한다."""
        while self._running:
            try:
                data = b''
                while len(data) < PACKET_SIZE:
                    chunk = self.feed.socket_dobot.recv(PACKET_SIZE - len(data))
                    if not chunk:
                        raise ConnectionError('feed 소켓 종료됨')
                    data += chunk
                feed_info = np.frombuffer(data, dtype=MyType)
                if hex(feed_info['test_value'][0]) == FEED_MAGIC:
                    with self._lock:
                        self._latest = feed_info
            except Exception as e:
                if self._running:
                    self.get_logger().warn(f'feed 수신 오류: {e}')
                    sleep(0.5)

    def _publish(self):
        with self._lock:
            info = self._latest
        if info is None:
            return

        now = self.get_clock().now().to_msg()

        # 1) JointState : q_actual[0:n] (n = joint_names 개수)
        q = info['q_actual'][0]
        n = len(self.joint_names)
        positions = [math.radians(v) if self.deg2rad else float(v)
                     for v in q[:n]]
        js = JointState()
        js.header.stamp = now
        js.name = self.joint_names
        js.position = positions
        self.joint_pub.publish(js)

        # 2) PoseStamped : tool_vector_actual = [X,Y,Z(mm), Rx,Ry,Rz(deg)]
        tcp = info['tool_vector_actual'][0]
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = self.base_frame
        ps.pose.position.x = float(tcp[0]) / 1000.0   # mm -> m
        ps.pose.position.y = float(tcp[1]) / 1000.0
        ps.pose.position.z = float(tcp[2]) / 1000.0
        qx, qy, qz, qw = quaternion_from_euler(
            math.radians(float(tcp[3])),
            math.radians(float(tcp[4])),
            math.radians(float(tcp[5])))
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pose_pub.publish(ps)

        # 3) robot_mode
        self.mode_pub.publish(Int32(data=int(info['robot_mode'][0])))

    def destroy_node(self):
        self._running = False
        try:
            self.feed.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MG400StatusNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
