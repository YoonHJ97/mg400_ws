import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler
import math
import threading
import numpy as np
import re
from time import sleep
from mg400_pkg.dobot_api import DobotApiDashboard, DobotApiMove, DobotApi, MyType, alarmAlarmJsonFile

# 피드백·에러 스레드가 공유하는 전역 상태 (move_node.py와 동일)
current_actual = None
algorithm_queue = None
enableStatus_robot = None
robotErrorState = False
globalLockValue = threading.Lock()

class NavAndDobotNode(Node):
    def __init__(self):
        super().__init__('nav_dobot_node')

        # —— MG400 연결 및 초기화 —— 
        try:
            ip = "192.168.1.6"                  # 로봇 IP
            dash_port, move_port, feed_port = 29999, 30003, 30004
            self.dashboard = DobotApiDashboard(ip, dash_port)
            self.move      = DobotApiMove(ip, move_port)
            self.feed      = DobotApi(ip, feed_port)
        except Exception as e:
            self.get_logger().error(f"MG400 연결 실패: {e}")
            rclpy.shutdown()
            return

        self.get_logger().info("MG400 연결 성공, 로봇 활성화 중...")
        self.dashboard.EnableRobot()
        self.get_logger().info("MG400 활성화 완료.")

        # 피드백·에러 모니터링 스레드 (기존 move_node.py 로직 재사용)
        threading.Thread(target=self._feedback_thread, daemon=True).start()
        threading.Thread(target=self._error_thread,    daemon=True).start()

        # —— Nav2 ActionClient 설정 —— 
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Nav2 액션 서버 대기 중…')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('액션 서버 연결 실패, 종료.')
            rclpy.shutdown()
            return
        self.get_logger().info('Nav2 액션 서버 연결됨.')

        # 파라미터 선언 & 읽기
        self.declare_parameter('goal_x', 1.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw_deg', 0.0)
        self.declare_parameter('dobot_x', 300.0)
        self.declare_parameter('dobot_y', 0.0)
        self.declare_parameter('dobot_z', 50.0)
        self.declare_parameter('dobot_r', 0.0)

        # 네비게이션 목표 전송
        self._send_nav_goal()

    def _send_nav_goal(self):
        gx = self.get_parameter('goal_x').value
        gy = self.get_parameter('goal_y').value
        gyaw = math.radians(self.get_parameter('goal_yaw_deg').value)

        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = gx
        ps.pose.position.y = gy
        qx, qy, qz, qw = quaternion_from_euler(0,0,gyaw)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        goal.pose = ps

        self.get_logger().info(f'네비 목표 전송: x={gx:.2f}, y={gy:.2f}, yaw={math.degrees(gyaw):.1f}°')
        fut = self._action_client.send_goal_async(goal, feedback_callback=self._fb_cb)
        fut.add_done_callback(self._nav_response_cb)

    def _fb_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'[Nav2 피드백] 남은 거리: {dist:.2f} m')

    def _nav_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('네비 목표 거부됨.')
            rclpy.shutdown()
            return
        self.get_logger().info('네비 목표 수락, 결과 대기…')
        res_fut = handle.get_result_async()
        res_fut.add_done_callback(self._nav_result_cb)

    def _nav_result_cb(self, future):
        res = future.result().result
        self.get_logger().info(f'[Nav2 완료] 코드: {res.error_code}')
        if res.error_code == 0:
            # 네비 성공 시 Dobot 이동
            dx = self.get_parameter('dobot_x').value
            dy = self.get_parameter('dobot_y').value
            dz = self.get_parameter('dobot_z').value
            dr = self.get_parameter('dobot_r').value
            self.get_logger().info(f'도착 감지 → MG400 이동: [{dx}, {dy}, {dz}, {dr}]')
            self.move.MovL(dx, dy, dz, dr)
        else:
            self.get_logger().warn('네비 실패, MG400 이동 생략.')
        rclpy.shutdown()

    # 이하 move_node.py에서 가져온 스레드용 메서드
    def _feedback_thread(self):
        global current_actual, algorithm_queue, enableStatus_robot, robotErrorState
        has = 0
        while rclpy.ok():
            data = bytes()
            while has < 1440:
                tmp = self.feed.socket_dobot.recv(1440-has)
                has += len(tmp); data += tmp
            has = 0
            info = np.frombuffer(data, dtype=MyType)
            if hex(info['test_value'][0])=='0x123456789abcdef':
                with globalLockValue:
                    current_actual    = info['tool_vector_actual'][0]
                    algorithm_queue   = info['isRunQueuedCmd'][0]
                    enableStatus_robot= info['EnableStatus'][0]
                    robotErrorState   = info['ErrorStatus'][0]
            sleep(0.001)

    def _error_thread(self):
        global robotErrorState, enableStatus_robot, algorithm_queue
        dc, ds = alarmAlarmJsonFile()
        while rclpy.ok():
            with globalLockValue:
                err = robotErrorState
                en  = enableStatus_robot
                aq  = algorithm_queue
            if err:
                nums = [int(n) for n in re.findall(r'-?\d+', self.dashboard.GetErrorID())]
                if nums and nums[0]==0 and len(nums)>1:
                    # 사용자 확인 로직… (생략)
                    self.dashboard.ClearError()
                    self.dashboard.Continue()
            elif en is not None and aq is not None:
                if int(en[0])==1 and int(aq[0])==0:
                    self.dashboard.Continue()
            sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = NavAndDobotNode()
    rclpy.spin(node)

if __name__=='__main__':
    main()
