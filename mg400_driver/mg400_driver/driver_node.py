#!/usr/bin/env python3
"""MG400 단일 드라이버 노드.

로봇 TCP 포트(29999 dashboard / 30003 move / 30004 feed)에 붙는 유일한 노드.
나머지 모든 노드(AMR 포함)는 이 드라이버가 제공하는 ROS 인터페이스로만 대화하므로,
같은 포트를 두 곳에서 여는 충돌이 구조적으로 발생하지 않는다.

제공 인터페이스 (모두 노드 이름이 앞에 붙음, 예: /mg400_driver/...):
  발행 토픽
    ~/joint_states  (sensor_msgs/JointState)
    ~/tcp_pose      (geometry_msgs/PoseStamped)
    ~/robot_mode    (std_msgs/Int32)
  액션 (Nav2 스타일)
    ~/move_l        (mg400_interfaces/MoveL)      직선 이동
    ~/move_joint    (mg400_interfaces/MoveJoint)  관절 이동
  서비스
    ~/set_enabled   (mg400_interfaces/SetEnabled)
    ~/clear_error   (mg400_interfaces/ClearError)
"""
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Int32
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from mg400_interfaces.action import MoveL, MoveJoint
from mg400_interfaces.srv import SetEnabled, ClearError
from mg400_driver.dobot_api import DobotApiDashboard, DobotApiMove, DobotApi, MyType

FEED_MAGIC = '0x123456789abcdef'
PACKET_SIZE = 1440


class MG400Driver(Node):
    def __init__(self):
        super().__init__('mg400_driver')

        # ---------------- 파라미터 ----------------
        self.declare_parameter('robot_ip', '192.168.1.6')
        self.declare_parameter('dashboard_port', 29999)
        self.declare_parameter('move_port', 30003)
        self.declare_parameter('feed_port', 30004)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('base_frame', 'mg400_base_link')
        self.declare_parameter('joint_names',
                               ['joint1', 'joint2', 'joint3', 'joint4'])
        self.declare_parameter('auto_enable', True)
        self.declare_parameter('pos_tolerance_mm', 1.0)
        self.declare_parameter('rot_tolerance_deg', 1.0)
        self.declare_parameter('joint_tolerance_deg', 1.0)
        self.declare_parameter('arrival_timeout', 30.0)
        self.declare_parameter('control_rate', 20.0)   # 액션 모니터링 주기

        self.ip = self.get_parameter('robot_ip').value
        self.base_frame = self.get_parameter('base_frame').value
        self.joint_names = list(self.get_parameter('joint_names').value)
        self.njoints = len(self.joint_names)
        self.pos_tol = self.get_parameter('pos_tolerance_mm').value
        self.rot_tol = self.get_parameter('rot_tolerance_deg').value
        self.joint_tol = self.get_parameter('joint_tolerance_deg').value
        self.timeout = self.get_parameter('arrival_timeout').value
        self.ctrl_dt = 1.0 / self.get_parameter('control_rate').value
        pub_rate = self.get_parameter('publish_rate').value

        # ---------------- 로봇 연결 (유일한 소유자) ----------------
        dash_port = self.get_parameter('dashboard_port').value
        move_port = self.get_parameter('move_port').value
        feed_port = self.get_parameter('feed_port').value
        self.get_logger().info(
            f'MG400 연결: {self.ip} ({dash_port}/{move_port}/{feed_port})')
        self.dashboard = DobotApiDashboard(self.ip, dash_port)
        self.move = DobotApiMove(self.ip, move_port)
        self.feed = DobotApi(self.ip, feed_port)

        if self.get_parameter('auto_enable').value:
            self.dashboard.EnableRobot()
            self.get_logger().info('로봇 활성화 완료.')

        # ---------------- 공유 피드백 상태 ----------------
        self._lock = threading.Lock()
        self._latest = None
        self._running = True
        # 로봇은 한 대 → 동시에 하나의 모션만 허용
        self._motion_lock = threading.Lock()

        threading.Thread(target=self._feed_loop, daemon=True).start()

        # ---------------- ROS 인터페이스 ----------------
        cb = ReentrantCallbackGroup()
        self.joint_pub = self.create_publisher(JointState, '~/joint_states', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '~/tcp_pose', 10)
        self.mode_pub = self.create_publisher(Int32, '~/robot_mode', 10)
        self.create_timer(1.0 / pub_rate, self._publish_state, callback_group=cb)

        self.create_service(SetEnabled, '~/set_enabled',
                            self._set_enabled_cb, callback_group=cb)
        self.create_service(ClearError, '~/clear_error',
                            self._clear_error_cb, callback_group=cb)

        self._move_l_srv = ActionServer(
            self, MoveL, '~/move_l',
            execute_callback=self._exec_move_l,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cb)
        self._move_j_srv = ActionServer(
            self, MoveJoint, '~/move_joint',
            execute_callback=self._exec_move_joint,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=cb)

        self.get_logger().info('mg400_driver 준비 완료.')

    # ===================== 피드백 =====================
    def _feed_loop(self):
        while self._running:
            try:
                data = b''
                while len(data) < PACKET_SIZE:
                    chunk = self.feed.socket_dobot.recv(PACKET_SIZE - len(data))
                    if not chunk:
                        raise ConnectionError('feed 소켓 종료')
                    data += chunk
                info = np.frombuffer(data, dtype=MyType)
                if hex(info['test_value'][0]) == FEED_MAGIC:
                    with self._lock:
                        self._latest = info
            except Exception as e:
                if self._running:
                    self.get_logger().warn(f'feed 수신 오류: {e}')
                    time.sleep(0.5)

    def _snapshot(self):
        with self._lock:
            return self._latest

    def _publish_state(self):
        info = self._snapshot()
        if info is None:
            return
        now = self.get_clock().now().to_msg()

        q = info['q_actual'][0]
        js = JointState()
        js.header.stamp = now
        js.name = self.joint_names
        js.position = [math.radians(float(v)) for v in q[:self.njoints]]
        self.joint_pub.publish(js)

        tcp = info['tool_vector_actual'][0]
        self.pose_pub.publish(self._pose_stamped(tcp, now))
        self.mode_pub.publish(Int32(data=int(info['robot_mode'][0])))

    def _pose_stamped(self, tcp, stamp):
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = self.base_frame
        ps.pose = self._tcp_to_pose(tcp)
        return ps

    @staticmethod
    def _tcp_to_pose(tcp):
        """tcp = [X,Y,Z(mm), Rx,Ry,Rz(deg)] -> geometry_msgs/Pose (m, quat)."""
        p = Pose()
        p.position.x = float(tcp[0]) / 1000.0
        p.position.y = float(tcp[1]) / 1000.0
        p.position.z = float(tcp[2]) / 1000.0
        qx, qy, qz, qw = quaternion_from_euler(
            math.radians(float(tcp[3])),
            math.radians(float(tcp[4])),
            math.radians(float(tcp[5])))
        p.orientation.x, p.orientation.y = qx, qy
        p.orientation.z, p.orientation.w = qz, qw
        return p

    # ===================== 서비스 =====================
    def _set_enabled_cb(self, request, response):
        try:
            if request.enable:
                self.dashboard.EnableRobot()
                response.message = '로봇 활성화'
            else:
                self.dashboard.DisableRobot()
                response.message = '로봇 비활성화'
            response.success = True
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f'오류: {e}'
        return response

    def _clear_error_cb(self, request, response):
        try:
            self.dashboard.ClearError()
            time.sleep(0.05)
            self.dashboard.Continue()
            response.success = True
            response.message = '에러 클리어 및 재개'
        except Exception as e:
            response.success = False
            response.message = f'오류: {e}'
        return response

    # ===================== 액션 공통 =====================
    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _stop_motion(self):
        try:
            self.move.Sync()
        except Exception:
            pass
        try:
            self.dashboard.ResetRobot()
        except Exception as e:
            self.get_logger().warn(f'정지 처리 오류: {e}')

    def _apply_speed(self, speed_ratio):
        if speed_ratio and speed_ratio > 0:
            try:
                self.dashboard.SpeedFactor(int(max(1, min(100, speed_ratio))))
            except Exception as e:
                self.get_logger().warn(f'SpeedFactor 오류: {e}')

    # ===================== MoveL 액션 =====================
    def _exec_move_l(self, goal_handle):
        result = MoveL.Result()
        if not self._motion_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = '다른 모션 실행 중 (로봇은 동시 하나만)'
            return result
        try:
            g = goal_handle.request
            # Pose(m, quat) -> 로봇 좌표(mm, Rz deg)
            x = g.target.position.x * 1000.0
            y = g.target.position.y * 1000.0
            z = g.target.position.z * 1000.0
            o = g.target.orientation
            _, _, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
            r = math.degrees(yaw)
            target_mm = (x, y, z, r)

            self._apply_speed(g.speed_ratio)
            self.get_logger().info(f'MoveL -> {target_mm}')
            self.move.MovL(x, y, z, r)

            fb = MoveL.Feedback()
            deadline = time.time() + self.timeout
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop_motion()
                    goal_handle.canceled()
                    result.success = False
                    result.message = '취소됨'
                    return result
                info = self._snapshot()
                if info is not None:
                    tcp = info['tool_vector_actual'][0]
                    dist = math.sqrt(sum(
                        (float(tcp[i]) - target_mm[i]) ** 2 for i in range(3)))
                    fb.current_pose = self._tcp_to_pose(tcp)
                    fb.distance_remaining = dist / 1000.0
                    goal_handle.publish_feedback(fb)
                    if self._reached_cart(tcp, target_mm):
                        goal_handle.succeed()
                        result.success = True
                        result.message = '도착'
                        result.final_pose = self._tcp_to_pose(tcp)
                        return result
                if time.time() > deadline:
                    self._stop_motion()
                    goal_handle.abort()
                    result.success = False
                    result.message = f'타임아웃({self.timeout}s)'
                    return result
                time.sleep(self.ctrl_dt)
            goal_handle.abort()
            result.success = False
            result.message = 'shutdown'
            return result
        finally:
            self._motion_lock.release()

    def _reached_cart(self, tcp, target_mm):
        for i in range(3):
            if abs(float(tcp[i]) - target_mm[i]) > self.pos_tol:
                return False
        dr = abs((float(tcp[5]) - target_mm[3] + 180.0) % 360.0 - 180.0)
        return dr <= self.rot_tol

    # ===================== MoveJoint 액션 =====================
    def _exec_move_joint(self, goal_handle):
        result = MoveJoint.Result()
        if not self._motion_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = '다른 모션 실행 중 (로봇은 동시 하나만)'
            return result
        try:
            g = goal_handle.request
            target_deg = [math.degrees(v) for v in g.joint_positions[:4]]

            self._apply_speed(g.speed_ratio)
            self.get_logger().info(f'MoveJoint -> {target_deg}')
            self.move.JointMovJ(target_deg[0], target_deg[1],
                                target_deg[2], target_deg[3])

            fb = MoveJoint.Feedback()
            deadline = time.time() + self.timeout
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop_motion()
                    goal_handle.canceled()
                    result.success = False
                    result.message = '취소됨'
                    return result
                info = self._snapshot()
                if info is not None:
                    q = info['q_actual'][0]
                    errs = [abs(float(q[i]) - target_deg[i]) for i in range(4)]
                    fb.current_positions = [math.radians(float(q[i]))
                                            for i in range(4)]
                    fb.max_error = math.radians(max(errs))
                    goal_handle.publish_feedback(fb)
                    if max(errs) <= self.joint_tol:
                        goal_handle.succeed()
                        result.success = True
                        result.message = '도착'
                        result.final_positions = [math.radians(float(q[i]))
                                                  for i in range(4)]
                        return result
                if time.time() > deadline:
                    self._stop_motion()
                    goal_handle.abort()
                    result.success = False
                    result.message = f'타임아웃({self.timeout}s)'
                    return result
                time.sleep(self.ctrl_dt)
            goal_handle.abort()
            result.success = False
            result.message = 'shutdown'
            return result
        finally:
            self._motion_lock.release()

    # ===================== 종료 =====================
    def destroy_node(self):
        self._running = False
        try:
            self.dashboard.DisableRobot()
        except Exception:
            pass
        for c in (getattr(self, 'dashboard', None),
                  getattr(self, 'move', None), getattr(self, 'feed', None)):
            try:
                c.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor()
    try:
        node = MG400Driver()
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
