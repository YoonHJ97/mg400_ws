#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
import math, threading, time, re, numpy as np
from mg400_pkg.dobot_api import (
    DobotApiDashboard, DobotApiMove, DobotApi, MyType, alarmAlarmJsonFile
)

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')

        # --- 파라미터 선언 ---
        self.declare_parameter('nav_a_x', 1.0)
        self.declare_parameter('nav_a_y', 0.0)
        self.declare_parameter('nav_a_yaw', 0.0)
        self.declare_parameter('nav_return_x', 0.0)
        self.declare_parameter('nav_return_y', 0.0)
        self.declare_parameter('nav_return_yaw', 0.0)
        self.declare_parameter('dobot_x', 300.0)
        self.declare_parameter('dobot_y',   0.0)
        self.declare_parameter('dobot_z',  50.0)
        self.declare_parameter('dobot_r',   0.0)

        # --- MG400 연결 & 활성화 ---
        ip = "192.168.1.6"
        self.dashboard = DobotApiDashboard(ip, 29999)
        self.move      = DobotApiMove(ip, 30003)
        self.feed      = DobotApi(ip, 30004)
        self.dashboard.EnableRobot()
        # (에러/피드백 스레드는 생략하거나 필요한 경우 위 예제를 참조)
        time.sleep(0.1)

        # --- Nav2 ActionClient 설정 ---
        self._ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        if not self._ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 서버 없음')
            rclpy.shutdown()
            return

        # 상태 트래킹: 0=to_A,1=dobot,2=return
        self.phase = 0

        # 첫 번째 목표 전송
        self.send_nav_goal(
            self.get_parameter('nav_a_x').value,
            self.get_parameter('nav_a_y').value,
            self.get_parameter('nav_a_yaw').value
        )

    def send_nav_goal(self, x, y, yaw_deg):
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        q = quaternion_from_euler(0,0,math.radians(yaw_deg))
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        goal.pose = ps

        self.get_logger().info(f'[{self.phase}] Nav2 목표 전송 → x={x}, y={y}, yaw={yaw_deg}°')
        fut = self._ac.send_goal_async(goal, feedback_callback=self.fb_cb)
        fut.add_done_callback(self.nav_resp_cb)

    def fb_cb(self, fb_msg):
        self.get_logger().info(f'[FB] 남은 거리: {fb_msg.feedback.distance_remaining:.2f} m')

    def nav_resp_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('Nav2 거부됨')
            rclpy.shutdown(); return
        gh.get_result_async().add_done_callback(self.nav_res_cb)

    def nav_res_cb(self, fut):
        res = fut.result().result
        self.get_logger().info(f'[Nav2 완료] error_code={res.error_code}')
        if res.error_code != 0:
            self.get_logger().error('네비 실패 → 종료')
            rclpy.shutdown(); return

        if self.phase == 0:
            # 1) A 도착 → MG400 이동
            dx = self.get_parameter('dobot_x').value
            dy = self.get_parameter('dobot_y').value
            dz = self.get_parameter('dobot_z').value
            dr = self.get_parameter('dobot_r').value
            self.get_logger().info(f'A 도착 → MG400 이동: [{dx},{dy},{dz},{dr}]')
            self.move.MovL(dx,dy,dz,dr)
            # MoveL는 blocking이므로 충분 시간 대기
            time.sleep(1.0)
            # phase 전환 후 복귀 목표 전송
            self.phase = 2
            self.send_nav_goal(
                self.get_parameter('nav_return_x').value,
                self.get_parameter('nav_return_y').value,
                self.get_parameter('nav_return_yaw').value
            )
        else:
            # 2) 복귀 완료
            self.get_logger().info('복귀 완료. 모든 미션 종료.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)

if __name__=='__main__':
    main()
