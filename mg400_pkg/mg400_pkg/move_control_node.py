#!/usr/bin/env python3
"""MG400 모션 제어 노드 (bringup 연동용).

기존 move_node / move_service_server 와 달리 feed(30004) 포트를 직접 열지 않는다.
대신 mg400_status_node 가 발행하는 tcp_pose 토픽을 구독해 도착을 판정하므로,
한 시스템에서 feed 연결은 status_node 하나만 유지된다 (포트 중복 충돌 방지).

명령은 dashboard(29999) + move(30003) 포트만 사용한다.

제공 서비스:
    ~/move_to  (mg400_pkg_msgs/MoveTo)  target=[X,Y,Z,R] 로 MovL 이동 후 도착까지 대기
구독 토픽:
    tcp_pose_topic (geometry_msgs/PoseStamped)  현재 TCP 위치 (status_node 발행, m 단위)
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion

from mg400_pkg_msgs.srv import MoveTo
from mg400_pkg.dobot_api import DobotApiDashboard, DobotApiMove


class MG400MoveNode(Node):
    def __init__(self):
        super().__init__('mg400_move_node')

        # --- 파라미터 ---
        self.declare_parameter('robot_ip', '192.168.1.6')
        self.declare_parameter('dashboard_port', 29999)
        self.declare_parameter('move_port', 30003)
        self.declare_parameter('tcp_pose_topic',
                               '/mg400_status_node/tcp_pose')
        self.declare_parameter('auto_enable', True)
        self.declare_parameter('pos_tolerance_mm', 1.0)    # X,Y,Z 허용오차(mm)
        self.declare_parameter('rot_tolerance_deg', 1.0)   # R(Rz) 허용오차(deg)
        self.declare_parameter('arrival_timeout', 30.0)    # 도착 대기 한계(s)

        ip = self.get_parameter('robot_ip').value
        dash_port = self.get_parameter('dashboard_port').value
        move_port = self.get_parameter('move_port').value
        topic = self.get_parameter('tcp_pose_topic').value
        self.auto_enable = self.get_parameter('auto_enable').value
        self.pos_tol = self.get_parameter('pos_tolerance_mm').value
        self.rot_tol = self.get_parameter('rot_tolerance_deg').value
        self.timeout = self.get_parameter('arrival_timeout').value

        # --- 명령 포트 연결 (feed 는 열지 않음) ---
        self.get_logger().info(f'MG400 명령 포트 연결: {ip} ({dash_port}/{move_port})')
        self.dashboard = DobotApiDashboard(ip, dash_port)
        self.move = DobotApiMove(ip, move_port)

        if self.auto_enable:
            self.dashboard.EnableRobot()
            self.get_logger().info('로봇 활성화 완료.')

        # --- status_node 의 tcp_pose 구독 (도착 판정용) ---
        self._current = None    # (x_mm, y_mm, z_mm, rz_deg)
        sub_cb = MutuallyExclusiveCallbackGroup()
        srv_cb = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            PoseStamped, topic, self._pose_cb, 10, callback_group=sub_cb)
        self.srv = self.create_service(
            MoveTo, '~/move_to', self._move_cb, callback_group=srv_cb)

        self.get_logger().info(
            f"'{topic}' 구독, ~/move_to 서비스 준비됨 "
            f"(status_node 가 함께 떠 있어야 도착 판정 가능).")

    def _pose_cb(self, msg: PoseStamped):
        # status_node 는 m 단위로 발행 -> mm 로 환산해 보관
        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._current = (
            msg.pose.position.x * 1000.0,
            msg.pose.position.y * 1000.0,
            msg.pose.position.z * 1000.0,
            math.degrees(yaw),
        )

    def _arrived(self, target):
        if self._current is None:
            return False
        for i in range(3):
            if abs(self._current[i] - target[i]) > self.pos_tol:
                return False
        # R(Rz) 비교: -180/180 경계 고려한 최소 각도차
        dr = abs((self._current[3] - target[3] + 180.0) % 360.0 - 180.0)
        return dr <= self.rot_tol

    def _move_cb(self, request, response):
        target = list(request.target)
        if len(target) < 4:
            response.success = False
            response.message = 'target 은 [X, Y, Z, R] 4개 값이어야 합니다.'
            return response
        if self._current is None:
            response.success = False
            response.message = ('tcp_pose 미수신 - mg400_status_node(bringup)'
                                ' 가 실행 중인지 확인하세요.')
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(f'이동 요청: {target}')
        self.move.MovL(target[0], target[1], target[2], target[3])

        # tcp_pose 피드백으로 도착 대기 (구독 콜백은 다른 콜백그룹에서 갱신)
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            if self._arrived(target):
                response.success = True
                response.message = '목표 위치 도착.'
                self.get_logger().info(response.message)
                return response
            time.sleep(0.02)

        response.success = False
        response.message = f'도착 타임아웃({self.timeout}s). 현재: {self._current}'
        self.get_logger().warn(response.message)
        return response

    def destroy_node(self):
        try:
            if self.auto_enable:
                self.dashboard.DisableRobot()
        except Exception as e:
            self.get_logger().error(f'DisableRobot 오류: {e}')
        try:
            self.dashboard.close()
            self.move.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor()
    try:
        node = MG400MoveNode()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
