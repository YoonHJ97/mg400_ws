# mg400_ws

Dobot **MG400** 로봇 팔을 ROS2로 제어하기 위한 패키지 모음입니다.

저장소 루트에 ROS2 패키지들이 위치하는 **패키지 저장소** 형태이며,
colcon 워크스페이스의 `src/` 안에 클론해서 사용합니다.

- `main` 브랜치 : MG400 단독 제어 (이 README 기준)
- `amr-integration` 브랜치 : Nav2 기반 AMR 연동 (드라이버 인터페이스를 호출)

---

## 설계 철학 — 단일 드라이버

로봇과 TCP(29999/30003/30004)로 통신하는 노드는 **`mg400_driver` 하나뿐**입니다.
다른 모든 노드(AMR 포함)는 이 드라이버가 제공하는 **ROS 인터페이스로만** 대화하므로,
같은 포트를 두 곳에서 여는 충돌이 구조적으로 발생하지 않습니다.
모션은 **액션**으로 추상화해 Nav2 의 `NavigateToPose` 와 동일한 사용 방식을 따릅니다.

```
            ┌──────────── ROS 인터페이스 ────────────┐
  사용자 / AMR ──(action/service/topic)──►  mg400_driver  ──(TCP)──►  MG400
            └────────────────────────────────────────┘   (유일한 연결 소유자)
```

---

## 1. 패키지 구성

| 패키지 | 빌드 타입 | 설명 |
|---|---|---|
| **`mg400_driver`** | ament_python | 로봇과 통신하는 **유일한** 노드 + Dobot TCP API |
| **`mg400_interfaces`** | ament_cmake | 액션/서비스 정의 (`MoveL`, `MoveJoint`, `SetEnabled`, `ClearError`) |
| **`mg400_bringup`** | ament_cmake | 실행용 launch + 파라미터(config) |

```
mg400_ws/
├── mg400_driver/
│   └── mg400_driver/
│       ├── driver_node.py    # mg400_driver 노드 (상태/모션/모드 전부)
│       ├── dobot_api.py      # Dobot TCP 통신 SDK
│       └── files/            # 알람 코드(JSON)
├── mg400_interfaces/
│   ├── action/MoveL.action
│   ├── action/MoveJoint.action
│   └── srv/{SetEnabled,ClearError}.srv
└── mg400_bringup/
    ├── launch/mg400.launch.py
    └── config/mg400_driver.yaml
```

---

## 2. mg400_driver 인터페이스

> 모든 이름 앞에 노드 이름이 붙습니다 (예: `/mg400_driver/tcp_pose`).

### 발행 토픽 (상태)
| 토픽 | 타입 | 내용 |
|---|---|---|
| `~/joint_states` | `sensor_msgs/JointState` | 관절 각도 (rad) |
| `~/tcp_pose` | `geometry_msgs/PoseStamped` | TCP 위치/자세 (m, quaternion) |
| `~/robot_mode` | `std_msgs/Int32` | 로봇 모드 |

### 액션 (모션)
| 액션 | 타입 | 내용 |
|---|---|---|
| `~/move_l` | `mg400_interfaces/MoveL` | 직선 이동 (`geometry_msgs/Pose` 목표) |
| `~/move_joint` | `mg400_interfaces/MoveJoint` | 관절 이동 (`float64[4]` rad) |

- 피드백 : 현재 자세 + 남은 거리(`distance_remaining`)
- 취소 지원, 도착/타임아웃 시 결과 반환
- 로봇은 한 대이므로 **동시에 하나의 모션만** 수행 (중복 요청은 거절)

### 서비스 (모드)
| 서비스 | 타입 | 내용 |
|---|---|---|
| `~/set_enabled` | `mg400_interfaces/SetEnabled` | EnableRobot / DisableRobot |
| `~/clear_error` | `mg400_interfaces/ClearError` | 에러 클리어 후 재개 |

---

## 3. 빌드

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/YoonHJ97/mg400_ws.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y   # (선택)
colcon build
source install/setup.bash
```

> `tf_transformations` 가 없다면: `sudo apt install ros-$ROS_DISTRO-tf-transformations`

---

## 4. 실행

```bash
# 드라이버 실행 (상태 발행 + 액션/서비스 서버)
ros2 launch mg400_bringup mg400.launch.py

# 로봇 IP 변경
ros2 launch mg400_bringup mg400.launch.py robot_ip:=192.168.1.10
```

### 상태 확인
```bash
ros2 topic echo /mg400_driver/tcp_pose
ros2 topic echo /mg400_driver/joint_states
ros2 action list           # /mg400_driver/move_l, /mg400_driver/move_joint
```

### 모션 명령 (액션)
```bash
# 직선 이동: (0.3, 0.0, 0.05) m 위치로
ros2 action send_goal /mg400_driver/move_l mg400_interfaces/action/MoveL \
  "{target: {position: {x: 0.3, y: 0.0, z: 0.05}, orientation: {w: 1.0}}, speed_ratio: 50.0}" \
  --feedback

# 관절 이동 (rad)
ros2 action send_goal /mg400_driver/move_joint mg400_interfaces/action/MoveJoint \
  "{joint_positions: [0.0, 0.2, 0.2, 0.0], speed_ratio: 50.0}"
```

### 모드 명령 (서비스)
```bash
ros2 service call /mg400_driver/set_enabled mg400_interfaces/srv/SetEnabled "{enable: true}"
ros2 service call /mg400_driver/clear_error mg400_interfaces/srv/ClearError "{}"
```

---

## 5. 파라미터 (config/mg400_driver.yaml)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `robot_ip` | `192.168.1.6` | MG400 IP |
| `dashboard_port` / `move_port` / `feed_port` | 29999 / 30003 / 30004 | TCP 포트 |
| `publish_rate` | `10.0` | 상태 발행 주기 (Hz) |
| `control_rate` | `20.0` | 액션 도착 모니터링 주기 (Hz) |
| `auto_enable` | `true` | 시작 시 EnableRobot |
| `pos_tolerance_mm` / `rot_tolerance_deg` | 1.0 / 1.0 | 직선 이동 도착 오차 |
| `joint_tolerance_deg` | `1.0` | 관절 이동 도착 오차 |
| `arrival_timeout` | `30.0` | 도착 대기 한계 (s) |

---

## 6. 네트워크 / 포트

| 포트 | 용도 |
|---|---|
| 29999 | Dashboard (Enable/Disable, 에러 클리어) |
| 30003 | Move (MovL/MovJ 등 모션) |
| 30004 | Feed (실시간 피드백) |

---

## 7. 향후 작업(TODO)

- [ ] URDF + `robot_state_publisher` 로 RViz 실시간 시각화 (joint_states 활용)
- [ ] `amr-integration` 브랜치를 드라이버 액션(`move_l`) 호출 방식으로 갱신
- [ ] 속도/가속 프로파일, I/O 제어 인터페이스 추가
