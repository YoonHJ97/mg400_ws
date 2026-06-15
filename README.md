# mg400_ws

Dobot **MG400** 로봇 팔을 ROS2로 제어하기 위한 패키지 모음입니다.

저장소 루트에 ROS2 패키지들이 바로 위치하는 **패키지 저장소** 형태이며,
colcon 워크스페이스의 `src/` 안에 클론해서 사용합니다.

- `main` 브랜치 : **순수 MG400** 제어 (이 README 기준)
- `amr-integration` 브랜치 : Nav2 기반 **AMR(모바일 로봇) 연동** 노드 추가

---

## 1. 패키지 구성

| 패키지 | 빌드 타입 | 설명 |
|---|---|---|
| **`mg400_pkg`** | ament_python | MG400 노드 + Dobot TCP API (`dobot_api.py`) |
| **`mg400_pkg_msgs`** | ament_cmake | 사용자 정의 인터페이스 (`MoveTo.srv`) |
| **`mg400_bringup`** | ament_cmake | 실행용 launch + 파라미터(config) |

```
mg400_ws/
├── mg400_pkg/
│   └── mg400_pkg/
│       ├── dobot_api.py          # Dobot TCP 통신 SDK (소켓 래퍼)
│       ├── status_node.py        # 상태 발행 노드 (mg400_status_node, bringup)
│       ├── move_control_node.py  # 모션 제어 노드 (mg400_move_node, bringup 연동)
│       ├── move_service_server.py# MoveTo 서비스 서버 (독립 실행)
│       ├── move_node.py          # A↔B 왕복 모션 데모 (독립 실행)
│       └── files/                # 알람 코드(JSON) + 로더
├── mg400_pkg_msgs/
│   └── srv/MoveTo.srv
└── mg400_bringup/
    ├── launch/mg400.launch.py
    └── config/mg400_params.yaml
```

---

## 2. 노드 설명

### `mg400_status_node` (status_node.py) — 상태 발행
30004(feed) 포트의 1440바이트 피드백 패킷을 파싱해 ROS2 토픽으로 발행합니다.
(수신은 별도 스레드, 발행은 타이머로 분리)

| 발행 토픽 | 타입 | 내용 |
|---|---|---|
| `~/joint_states` | `sensor_msgs/JointState` | `q_actual` → 4관절 각도 (deg→rad 변환) |
| `~/tcp_pose` | `geometry_msgs/PoseStamped` | TCP 위치/자세 (mm→m, RPY→quaternion) |
| `~/robot_mode` | `std_msgs/Int32` | 로봇 모드 값 |

> 노드 이름이 `mg400_status_node`이므로 실제 토픽은
> `/mg400_status_node/joint_states` 처럼 노드 이름이 앞에 붙습니다.

**파라미터** (config/mg400_params.yaml 에서 주입)

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `robot_ip` | `192.168.1.6` | MG400 IP |
| `feed_port` | `30004` | 실시간 피드백 포트 |
| `publish_rate` | `10.0` | 발행 주기 (Hz) |
| `base_frame` | `mg400_base_link` | PoseStamped frame_id |
| `joint_names` | `[joint1..joint4]` | JointState 관절 이름 |
| `joints_in_degrees` | `true` | deg→rad 변환 여부 |

### `mg400_move_node` (move_control_node.py) — 모션 제어 (bringup 연동)
bringup 과 함께 동작하는 모션 제어 노드입니다. **feed(30004)를 직접 열지 않고**
`mg400_status_node` 가 발행하는 `tcp_pose` 를 구독해 도착을 판정하므로, 한 시스템에서
feed 연결은 status_node 하나만 유지됩니다 (포트 충돌 없음). 명령은 29999/30003 만 사용.

- 서비스 : `~/move_to` (`mg400_pkg_msgs/srv/MoveTo`) → `/mg400_move_node/move_to`
  ```
  float64[4] target      # 요청: [X, Y, Z, R]
  ---
  bool   success         # 응답: 도착 여부
  string message
  ```
- 구독 : `tcp_pose_topic` (기본 `/mg400_status_node/tcp_pose`)
- 파라미터 : `robot_ip`, `dashboard_port`(29999), `move_port`(30003),
  `tcp_pose_topic`, `auto_enable`, `pos_tolerance_mm`, `rot_tolerance_deg`,
  `arrival_timeout`
- **반드시 `mg400_status_node` 와 함께** 실행 (도착 판정에 tcp_pose 필요)

### `move_service_server` / `move_node` — 독립 실행용 (bringup 별개)
둘 다 dashboard/move/**feed(30004)를 자체적으로 열고** 로봇을 점유하는 독립 노드라
bringup 과 동시에 실행하면 feed 포트가 충돌합니다. **bringup 과 분리해 단독 실행**하세요.

- `move_service_server` : `move_to` 서비스로 목표 좌표 이동 (자체 feed 로 도착 판정)
- `move_node` : 고정 두 점(point_a ↔ point_b) 무한 왕복 데모

```bash
# 단독 실행 (bringup 을 끈 상태에서)
ros2 run mg400_pkg move_service_server
ros2 run mg400_pkg move_node
```

> 이 두 노드는 아직 IP가 코드에 하드코딩(`192.168.1.6`)되어 있습니다. 추후 파라미터화 예정.

---

## 3. 빌드

```bash
# 1) 워크스페이스 준비 후 src/ 안에 클론
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/YoonHJ97/mg400_ws.git

# 2) 의존성 설치 (선택)
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3) 빌드 & 환경 적용
colcon build
source install/setup.bash
```

> `tf_transformations` 가 없다면: `sudo apt install ros-$ROS_DISTRO-tf-transformations`

---

## 4. 실행 (bringup)

```bash
# 상태 발행 노드만 실행
ros2 launch mg400_bringup mg400.launch.py

# 로봇 IP 변경 (config 값 덮어쓰기)
ros2 launch mg400_bringup mg400.launch.py robot_ip:=192.168.1.10

# 모션 제어 노드(mg400_move_node)도 함께 실행
ros2 launch mg400_bringup mg400.launch.py enable_move:=true
```

### 상태 확인 / 명령 예시
```bash
# 발행되는 상태 확인
ros2 topic echo /mg400_status_node/tcp_pose
ros2 topic echo /mg400_status_node/joint_states

# 특정 좌표로 이동 (enable_move:=true 로 실행했을 때)
ros2 service call /mg400_move_node/move_to mg400_pkg_msgs/srv/MoveTo "{target: [300.0, 0.0, 50.0, 0.0]}"
```

---

## 5. 네트워크 / 포트

MG400은 TCP로 통신하며 3개 포트를 사용합니다 (기본 IP `192.168.1.6`).

| 포트 | 용도 |
|---|---|
| 29999 | Dashboard (Enable/Disable, 에러 클리어 등 명령) |
| 30003 | Move (MovL 등 모션 명령) |
| 30004 | Feed (실시간 피드백 패킷 수신) |

---

## 6. 브랜치

| 브랜치 | 내용 |
|---|---|
| `main` | 순수 MG400 제어 (위 패키지들) |
| `amr-integration` | main + Nav2 연동 노드(`nav_and_move`, `nav_and_return`, `test`, `test_mg400`) |

AMR 연동 작업은 `git checkout amr-integration` 후 진행합니다.

---

## 7. 향후 작업(TODO)

- [ ] URDF + `robot_state_publisher` 로 RViz 실시간 시각화 (joint_states 활용)
- [ ] `move_node` / `move_service_server` IP 파라미터화 → bringup 통합
- [ ] 상태 발행 + 명령을 합친 통합 driver 노드
