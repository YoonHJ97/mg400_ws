# mg400_ws — Claude 작업 가이드

Dobot **MG400** 로봇 팔의 ROS2 제어 워크스페이스. 자세한 사용법은 `README.md` 참고.

## 핵심 설계 원칙 (반드시 지킬 것)

**로봇 TCP(29999 dashboard / 30003 move / 30004 feed)에 연결하는 노드는 `mg400_driver` 단 하나뿐이다.**
다른 모든 노드(AMR 포함)는 드라이버가 제공하는 ROS 인터페이스로만 대화한다.
→ 새 기능을 추가할 때 로봇에 직접 소켓을 여는 노드를 만들지 말 것. 반드시 드라이버의
액션/서비스/토픽을 사용하거나 드라이버에 인터페이스를 추가할 것. (과거 여러 노드가 각자
연결해 포트 충돌이 났고, 이를 없애려고 단일 드라이버 구조로 재편함.)

## 패키지 구조 (저장소 루트 = 패키지 모음, colcon ws의 src/ 안에 클론)

- `mg400_driver` (ament_python): 유일한 하드웨어 노드 `driver_node.py` + `dobot_api.py`
- `mg400_interfaces` (ament_cmake): `action/MoveL`, `action/MoveJoint`, `srv/SetEnabled`, `srv/ClearError`
- `mg400_bringup` (ament_cmake): `launch/mg400.launch.py` + `config/mg400_driver.yaml`

## mg400_driver 인터페이스 (이름 앞에 노드명 붙음: `/mg400_driver/...`)

- 토픽: `~/joint_states`, `~/tcp_pose`, `~/robot_mode`
- 액션(Nav2 스타일, 피드백/취소/타임아웃, 동시 1개만): `~/move_l`(Pose 목표), `~/move_joint`(rad)
- 서비스: `~/set_enabled`, `~/clear_error`

## 빌드 / 실행

```bash
# 워크스페이스 루트(이 저장소의 상위 = colcon ws)에서
colcon build && source install/setup.bash
ros2 launch mg400_bringup mg400.launch.py
```

빌드 산출물 `build/ install/ log/` 는 `.gitignore` 처리됨.

## 코드 컨벤션

- Python 노드는 ament_python, 진입점은 `setup.py` console_scripts 에 등록.
- 파라미터는 코드 하드코딩 대신 `config/mg400_driver.yaml` 로 주입.
- 좌표 변환: 로봇은 mm/deg, ROS 토픽/액션은 m/rad(+quaternion). 드라이버에서 변환.

## 브랜치

- `main`: 단일 드라이버 + 액션 기반 구조 (현재 기준)
- `amr-integration`: **옛 구조**(`mg400_pkg`/`mg400_pkg_msgs`, `MoveTo.srv`) 기반.
  AMR 통합 시 드라이버의 `move_l` 액션 호출 방식으로 재작업 필요.

## 환경 메모

- 로봇 IP 기본값 `192.168.1.6`.
- 이 저장소는 보통 `~/mg400_ws/src/mg400_ws/` 에 위치(워크스페이스=`~/mg400_ws`).
- origin: github.com/YoonHJ97/mg400_ws
