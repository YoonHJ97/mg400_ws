#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import threading
from mg400_pkg.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from mg400_pkg_msgs.srv import MoveTo  # 사용자 정의 서비스 임포트
from time import sleep
import numpy as np
import re

# Global variables (shared among threads)
current_actual = None
algorithm_queue = None
enableStatus_robot = None
robotErrorState = False
globalLockValue = threading.Lock()


def ConnectRobot():
    try:
        ip = "192.168.1.6"  # 로봇 IP (필요시 수정)
        dashboardPort = 29999
        movePort = 30003
        feedPort = 30004
        print("Establishing connection...")
        dashboard = DobotApiDashboard(ip, dashboardPort)
        move = DobotApiMove(ip, movePort)
        feed = DobotApi(ip, feedPort)
        print(">.<Connection successful>!<")
        return dashboard, move, feed
    except Exception as e:
        print(":(Connection failed:(")
        raise e


def RunPoint(move: DobotApiMove, point_list: list):
    move.MovL(point_list[0], point_list[1], point_list[2], point_list[3])


def GetFeed(feed: DobotApi):
    global current_actual, algorithm_queue, enableStatus_robot, robotErrorState
    hasRead = 0
    while True:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0
        feedInfo = np.frombuffer(data, dtype=MyType)
        if hex(feedInfo['test_value'][0]) == '0x123456789abcdef':
            globalLockValue.acquire()
            # 현재 값 갱신
            current_actual = feedInfo["tool_vector_actual"][0]
            algorithm_queue = feedInfo['isRunQueuedCmd'][0]
            enableStatus_robot = feedInfo['EnableStatus'][0]
            robotErrorState = feedInfo['ErrorStatus'][0]
            globalLockValue.release()
        sleep(0.001)


def WaitArrive(point_list):
    # 목표 좌표와 현재 좌표 간의 오차가 허용범위(여기서는 1)를 넘지 않을 때까지 대기
    while True:
        is_arrive = True
        globalLockValue.acquire()
        if current_actual is not None:
            for index in range(4):
                if abs(current_actual[index] - point_list[index]) > 1:
                    is_arrive = False
            if is_arrive:
                globalLockValue.release()
                return
        globalLockValue.release()
        sleep(0.001)


def ClearRobotError(dashboard: DobotApiDashboard):
    global robotErrorState, enableStatus_robot, algorithm_queue
    dataController, dataServo = alarmAlarmJsonFile()    # 컨트롤러 및 서보 알람 코드 읽기
    while True:
        globalLockValue.acquire()
        if robotErrorState:
            numbers = re.findall(r'-?\d+', dashboard.GetErrorID())
            numbers = [int(num) for num in numbers]
            if numbers and numbers[0] == 0:
                if len(numbers) > 1:
                    for i in numbers[1:]:
                        alarmState = False
                        if i == -2:
                            print("Robot Alarm: Collision detected", i)
                            alarmState = True
                        if alarmState:
                            continue                
                        for item in dataController:
                            if i == item["id"]:
                                print("Robot Alarm: Controller error ID", i, item["en"]["description"])
                                alarmState = True
                                break 
                        if alarmState:
                            continue
                        for item in dataServo:
                            if i == item["id"]:
                                print("Robot Alarm: Servo error ID", i, item["en"]["description"])
                                break  
                    # 에러 클리어를 위한 사용자 입력 (ROS2 환경에서는 콘솔 입력이 제한될 수 있으므로, 필요시 수정)
                    choose = input("Enter 1 to clear errors and continue operation: ")     
                    if int(choose) == 1:
                        dashboard.ClearError()
                        sleep(0.01)
                        dashboard.Continue()
        else:  
            # 에러 상태가 없고, 로봇이 활성화 상태이면 continue 명령 전송
            if int(enableStatus_robot[0]) == 1 and int(algorithm_queue[0]) == 0:
                dashboard.Continue()
        globalLockValue.release()
        sleep(5)


class DobotNode(Node):
    def __init__(self):
        super().__init__('dobot_node')
        self.get_logger().info("Initializing Dobot Node...")
        try:
            self.dashboard, self.move, self.feed = ConnectRobot()
        except Exception as e:
            self.get_logger().error("Failed to connect to robot: {}".format(e))
            rclpy.shutdown()
            return

        # 로봇 활성화
        self.get_logger().info("Starting enable...")
        self.dashboard.EnableRobot()
        self.get_logger().info("Enable complete :)")

        # 스레드 생성: 피드백 수신
        self.feed_thread = threading.Thread(target=GetFeed, args=(self.feed,))
        self.feed_thread.setDaemon(True)
        self.feed_thread.start()

        # 스레드 생성: 에러 클리어 모니터링
        self.error_thread = threading.Thread(target=ClearRobotError, args=(self.dashboard,))
        self.error_thread.setDaemon(True)
        self.error_thread.start()

        # 서비스 서버 생성: 특정 위치로 이동하는 명령을 수신
        self.move_to_srv = self.create_service(MoveTo, 'move_to', self.move_to_callback)
        self.get_logger().info("MoveTo service is ready.")

    def move_to_callback(self, request, response):
        """
        MoveTo 서비스 요청 콜백 함수
        요청으로 전달된 target 좌표로 로봇을 이동시키고, 도착 여부를 응답한다.
        """
        target = request.target
        self.get_logger().info("Received move request: {}".format(target))
        try:
            RunPoint(self.move, target)
            WaitArrive(target)
            response.success = True
            response.message = "Arrived at target position."
            self.get_logger().info("Successfully arrived at target: {}".format(target))
        except Exception as e:
            response.success = False
            response.message = "Error: {}".format(e)
            self.get_logger().error("Failed to move to target: {} - Error: {}".format(target, e))
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.get_logger().info("Disabling robot before shutdown...")
        try:
            node.dashboard.DisableRobot()
        except Exception as e:
            node.get_logger().error("Error disabling robot: {}".format(e))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
