# 터틀봇3 + OpenMANIPULATOR-X 로봇
# Lane Detecting 및 무브잇 역기구학 계산

---

## 시연 영상

[![Video Label](http://img.youtube.com/vi/r10zCx1z2HY/0.jpg)](https://youtu.be/r10zCx1z2HY)

---


## 사용 장비 및 개발 환경

### **하드웨어**

Turtlebot3 Waffle Pi

https://emanual.robotis.com/docs/en/platform/turtlebot3/features/#specifications

<img width="320" alt="image" src="https://github.com/user-attachments/assets/ec18f961-035e-46a9-bf0c-8a51bdeb5d10" />


OpenMANIPULATOR-X

https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/#hardware-specification

<img width="320" alt="image" src="https://github.com/user-attachments/assets/bb6fd4d8-2662-4d7b-9889-9a8054dbd5d4" />


C270 HD 웹캠

https://www.logitech.com/ko-kr/shop/p/c270-hd-webcam.960-000626

<img width="160" alt="image" src="https://github.com/user-attachments/assets/2a6cd844-5c2f-4982-877f-de9cfa5ab85f" />

---

## 소프트웨어 및 기술 스택

**AI 및 컴퓨터 비전** : **OpenCV** 영상처리

**OS**: Ubuntu 22.04 LTS
---

## 시나리오

1. 로봇이 아루코마커 위치 인식 및 로봇팔로 잡아서 옮김 
2. 로봇이 주어진 환경(트랙)에서 영상처리 및 자율주행 

---

## 프로젝트 트리 구조

```bash
turtlebot3_ws/project/
├── aruco_detector.py
├── cmd_vel_graph.py
├── control_lane_finall.py
├── detect_lane_ori.py
├── gray_img_pub.py
├── pick_and_place.py
└── turtlebot_arm_controller.cpp

turtlebot3_ws/src/
├── [aruco_yolo]
├── dynamic_obstacle_plugin
├── DynamixelSDK
├── fsd_pkg
├── image_pipeline
├── ld08_driver
├── pyqt_robot
├── sample_pkg
├── turtlebot3
│   ├── turtlebot3
│   ├── turtlebot3_bringup
│   ├── turtlebot3_cartographer
│   ├── turtlebot3_description
│   ├── turtlebot3_example
│   ├── turtlebot3_navigation2
│   ├── turtlebot3_node
│   └── turtlebot3_teleop
├── turtlebot3_autorace
│   ├── turtlebot3_autorace
│   ├── turtlebot3_autorace_camera
│   ├── turtlebot3_autorace_detect
│   └── turtlebot3_autorace_mission
├── [turtlebot3_manipulation]
│   ├── turtlebot3_manipulation
│   ├── turtlebot3_manipulation_bringup
│   ├── turtlebot3_manipulation_description
│   ├── turtlebot3_manipulation_hardware
│   ├── turtlebot3_manipulation_moveit_config
│   └── turtlebot3_manipulation_teleop
├── turtlebot3_msgs
├── turtlebot3_simulations
├── [turtlebot_cosmo_interface]
├── [turtlebot_moveit]
│   ├── manipulator_moveit
│   └── turtlebot_moveit
└── vision_opencv
    ├── cv_bridge
    ├── image_geometry
    ├── opencv_tests
    └── vision_opencv
```
**requirement**

[https://github.com/bako98/gazebo_turtlebot3_lane_detect](https://github.com/bako98/gazebo_turtlebot3_lane_detect)

위 깃헙대로 requirement 사전설치

**설치방법**

turtlebot3_autorace_moveit_project.zip 을 다운받고 project폴더는 turtlebot3_ws에 이동

turtlebot_arm_controller.cpp 의 경우 turtlebot3_ws/src/turtlebot_moveit/turtlebot_moveit/src/turtlebot_arm_controller.cpp 로 이동

나머지 modified 폴더안에 패키지들은 위 트리구조의 [package] 처럼 대괄호 친 것들을 바꾸기

그리고 빌드

```bash
cd turtlebot3_ws/
colcon build
```
---

## ROS2 노드 구성

- **image publisher**
- **aruco detector**
- **detect lane**
- **control lane**
- **pick and place**
- **arm control**

<img width="2075" height="869" alt="image" src="https://github.com/user-attachments/assets/3f86a342-cb47-4ffb-bfd7-02ffa23b616c" />

---
## 1. gray img pub

LogitechCameraPublisher는 ROS2 환경에서 USB 카메라로부터 실시간 영상을 수신하고, 그레이스케일로 변환한 뒤 JPEG 압축 형식(CompressedImage)으로 퍼블리시하는 카메라 드라이버 노드입니다.

✅ 주요 기능

1. 카메라 초기화 및 설정
- /dev/video0에 연결된 USB 카메라(예: 로지텍 웹캠) 열기
- 해상도: 320x240
- 프레임 속도(FPS): 30fps
- 설정 결과를 ROS 로그로 출력

2. ROS 퍼블리셔 생성
- 퍼블리시 토픽: /camera/image/compressed
- 메시지 타입: sensor_msgs/CompressedImage
- 퍼블리시 주기: 1/30초 (약 33ms)로 타이머 콜백 실행

3. 영상 처리 및 퍼블리시
- 실시간으로 프레임 캡처
- RGB → Grayscale 변환 후 다시 BGR로 재변환 → 압축 시 색상 채널이 필요하기 때문 (JPEG)
- cv_bridge를 사용하여 OpenCV 이미지 → ROS 메시지로 변환
- 타임스탬프와 frame_id 포함 후 퍼블리시

4. 예외 처리 및 자원 해제
- 카메라 오픈 실패 시 에러 로그 출력 후 종료
- 프레임 캡처 실패 및 cv_bridge 예외 발생 시 경고 출력
- 노드 종료 시 VideoCapture 해제 및 rclpy.shutdown()
  

**터미널1 gray img pub 노드 실행**

```bash
~/turtlebot3_ws$ python3 gray_img_pub.py 
```

---
## 2. aruco detector 노드


✅ 주요 기능

1. 카메라 보정 파라미터 로딩
- aruco_yolo/config/calibration_params.yaml에서 camera_matrix와 distortion_coefficients 로딩
- 보정된 카메라 파라미터를 통해 ArUco 마커의 3D 위치 계산 수행

2. 이미지 수신 및 전처리
- /camera/image/compressed 토픽으로부터 영상 구독
- cv_bridge를 통해 압축된 이미지를 OpenCV 이미지로 변환
- 해상도 320x240으로 리사이즈 후, Grayscale 변환 및 CLAHE 보정 적용 → 인식률 향상

3. ArUco 마커 검출 및 포즈 추정
- OpenCV의 cv2.aruco로 4x4_1000 딕셔너리를 사용해 마커 검출
- 마커마다:
- - 2D 중심 좌표로부터 3D 좌표 계산
- - cv2.solvePnP로 rvec, tvec (회전 및 변환 벡터) 추출
- - 회전 행렬 → 오일러 각(yaw, pitch, roll) 변환
- - 카메라 기준 X, Y, Z 좌표로 보정 거리 및 위치 계산
- - 결과: [marker_id, marker_pos, (yaw, pitch, roll), distance]

4. 거리 계산 및 보정
- 보정 거리 식: distance * 0.767 - 0.076
- 가장 가까운 마커만 선택하여 /aruco/distance (Float32)로 퍼블리시

5. MarkerArray 퍼블리시
- 가장 가까운 마커에 대한 정보(id, position, orientation)를 aruco_msgs/MarkerArray로 /detected_markers 토픽에 퍼블리시

6. 디버깅 및 시각화
- OpenCV를 통해 마커와 텍스트 정보가 포함된 결과 영상을 imshow()로 표시
- 키 입력 대기 없이 1ms마다 갱신 (cv2.waitKey(1))


**터미널2 aruco detector 노드 실행**

```bash
~/turtlebot3_ws$ python3 aruco_detector.py 
```

<img width="400" alt="image" src="https://github.com/user-attachments/assets/9f47fdd2-00a8-491e-898a-582d096bc63c" />


---

## 3. pick_and_place

pick_and_place.py는 ROS 2 환경에서 ArUco 마커를 인식하고, 이를 기반으로 로봇 팔(TurtlebotArmClient) 을 제어하여 지정된 위치로 이동시키는 Pick & Place 자동화 노드입니다. 마커 인식부터 로봇 경로 추적 시작까지의 일련의 동작을 자동화합니다.

✅ 주요 기능

1. ArUco 마커 인식 및 필터링
- 구독 토픽: /detected_markers (aruco_msgs/MarkerArray)
- 사용자가 지정한 Marker ID만 추적 (self.target_marker_id)
- 마커가 처음 인식된 위치만 저장하여 중복 동작 방지 (self.aruco_pose_saved)
- 인식 여부 및 필터링 상태는 색상 로그로 출력

2. 좌표 변환 및 Pose 저장
- 인식된 마커 위치는 기본적으로 camera_link 기준
- TF2를 이용해 base_link 기준 좌표로 변환
- tf2_ros.Buffer, tf2_ros.TransformListener 사용
- do_transform_pose_stamped() 호출
- 변환된 위치를 PoseArray로 래핑하여 이후 로봇 제어에 사용
- 자세는 yaw 회전값만 적용 → euler_to_quaternion() 함수로 변환

3. 로봇 팔 제어 및 Pick 동작 수행
- MoveIt 서비스 호출: TurtlebotArmClient.send_request() 사용
- 내부적으로 /moveit_control (turtlebot_cosmo_interface/srv/MoveitControl) 호출
- 저장된 PoseArray를 기반으로 로봇 팔 이동 수행
- Pick 동작 수행 후, 자동으로 /start_tracing (std_srvs/Trigger) 서비스 호출 → 자율 경로 추적 시작

4. 관절 상태 수신 및 실시간 디버깅
- 구독 토픽: /joint_states (sensor_msgs/JointState)
- 관절 위치 (joint1 ~ joint4)를 실시간으로 출력
- 디버깅 목적으로 관절 위치 로그 확인 가능 (self.get_joint 플래그 활성 시)
- 예외 처리 및 직관적인 디버깅 로그
- TF 변환 실패, ArUco 미인식 등 예외 상황에 대해 self.get_logger().error() 출력
- 상태 출력 시 ANSI 색상 코드 사용 (빨강, 파랑, 노랑 등)으로 로그 단계 구분
- 마커 중복 인식 방지, 위치 추적 불가 등도 로그로 안내

🧩 내부 동작 구조 및 유틸
- 좌표 변환: tf2_ros + do_transform_pose_stamped
- 회전 처리: euler_to_quaternion() 함수 (roll, pitch=0, yaw만 적용)
- 팔 제어 인터페이스: moveit_client.py의 TurtlebotArmClient 모듈 분리 사용
- cmd_vel 퍼블리셔 포함: /cmd_vel (geometry_msgs/Twist) → 현재는 예비용 코드


**터미널3 pick and place 노드 실행**
```bash
~/turtlebot3_ws$ python3 pick_and_place.py 
```


---

## 4. detect lane 노드


<img width="600" alt="image" src="https://github.com/user-attachments/assets/e4bee6a9-5a05-4f9a-8933-8356ec7946e8" />


<img width="400" alt="image" src="https://github.com/user-attachments/assets/3aefecb5-9cb9-4397-a017-e833cf34e392" />


✅ 주요기능

1. 이미지 구독 및 수신
- ROS 토픽 /camera/image/compressed 에서 압축 이미지(CompressedImage) 구독
- 3프레임 중 1프레임만 처리해 연산량 절감 (10fps → 약 3.3fps 처리)

2. 차선 후보 영역 마스크 생성 및 전처리
- 컬러 → 그레이스케일 변환
- 밝은 흰색 계열 픽셀 임계값(170)으로 이진화
- 작은 노이즈 제거 위한 모폴로지 오프닝 적용

3. 컨투어 검출 및 필터링
- 컨투어 면적 기준 필터링 (최소 500픽셀 이상)
- 큰 컨투어(20000 이상)면 세로선으로 분할 후 재검출 (가로로 붙은 차선 분리 목적)
- 컨투어 면적 기준 내림차순 정렬 후 상위 1~2개 컨투어 처리

4. 차선 좌/우 구분 및 기울기 판단
- 컨투어 중심 좌표 및 2차 다항식 피팅
- 한 개 컨투어일 경우, 기울기에 따라 좌/우 차선 판단
- 두 개 이상 컨투어일 경우, 중심 좌표 기준 왼쪽/오른쪽으로 구분

5. 차선 피팅 및 이동평균 처리
- fit_from_lines 함수로 2차 다항식 피팅
- 예외 발생 시 sliding_window 기법으로 차선 검출 시도
- 최근 5개 프레임의 계수를 이동평균해 차선 곡선을 부드럽게 유지

6. 차선 시각화 및 중심선 산출
- 차선 영역 및 좌/우 차선, 중심선 각각 다른 색으로 이미지 위에 폴리라인 그리기
- 좌/우 차선 모두 있을 때는 중심선은 좌우 평균으로 산출
- 한쪽 차선만 있을 경우 일정 픽셀 수만큼 이동하여 중심선 추정

7. 차선 상태 및 중심 좌표 ROS 퍼블리시
- /detect/lane 토픽에 중심선 x좌표 (Float64) 발행
- /detect/lane_state 토픽에 차선 상태 (UInt8) 발행
- 0: 차선 없음
- 1: 왼쪽 차선만 존재
- 2: 좌우 차선 모두 존재
- 3: 오른쪽 차선만 존재

8. 디버깅용 컨투어 이미지 퍼블리시
- /debug/contours/compressed 토픽에 컨투어가 그려진 마스크 이미지 발행


**터미널4 robot_control 노드 실행**
```bash
~/turtlebot3_ws$ python3 detect_lane_ori.py 
```

---
# 5. arm controller

✅ 주요기능

1. ROS 노드 및 서비스 초기화
- TurtlebotArmController라는 ROS 2 노드 생성
- 별도의 move_group_interface 노드와 SingleThreadedExecutor 스레드 생성 및 실행
- "moveit_control" 이름의 서비스 생성 (MoveIt 기반 로봇팔 제어 명령 수신)

2. MoveIt 인터페이스 초기화
- MoveIt MoveGroupInterface를 통해 "arm"과 "gripper" 그룹 제어
- 계획 파이프라인 설정 및 목표 위치·자세 허용 오차 지정

3. 서비스 콜백 handleMoveitControl
- 클라이언트로부터 들어오는 MoveitControl 서비스 요청 처리
- 요청 메시지 출력 (커맨드 종류, 목표 위치, 자세 등)

4. 주요 커맨드 처리
- cmd == 0 : 주어진 단일 waypoint (목표 위치)로 로봇팔을 이동
- - 현재 위치와 목표 위치 기반 yaw 각도 계산
- - RPY → 쿼터니언 변환 및 두 쿼터니언 곱셈 (방향 보정)
-  MoveIt으로 목표 자세 설정 후 경로 계획 및 실행
- cmd == 1 : "arm" 그룹의 미리 정의된 named target 위치로 이동
- cmd == 2 : "gripper" 그룹의 named target 위치로 이동 (그립퍼 동작)
- cmd == 3 : waypoint 이용한 이동 (다른 yaw 계산 방식 사용)
- cmd == 4 : waypoint의 쿼터니언 자세를 그대로 적용해 이동
- cmd == 9 : 현재 로봇팔 포즈 출력 (디버깅용)
- 그 외 커맨드는 경고 메시지 출력 및 실패 응답 반환

5. 내부 헬퍼 함수
- planAndExecute : MoveIt 경로 계획과 실행을 수행, 실패 시 에러 로그 출력
- rpyToQuaternion : Roll, Pitch, Yaw → 쿼터니언 변환
- multiply : 두 쿼터니언 곱셈 (방향 보정용)
- printCurrentPose : 현재 로봇팔 위치와 RPY 출력

6. 종료 처리
- 노드 소멸 시 executor 중지 및 스레드 종료 대기

**터미널5 arm controller 노드 실행** 
```bash
~/turtlebot3_ws$ ros2 run turtlebot_moveit turtlebot_arm_controller
# cmd == 0
```

---

# 6. control lane

카메라로부터 중앙 차선 위치를 받아 PD 제어로 로봇의 주행 방향을 제어하는 ROS2 노드


주요 기능
1. 서비스 및 구독자 초기화
- start_tracing 서비스 생성: 호출 시 차선 추적 시작(tracing_flag=True)
- /detect/lane 토픽 구독: 카메라 프레임 상의 차선 중심 X좌표 수신
- /control/max_vel 토픽 구독: 최대 속도값 수신 (현재 발행중인 노드 없음)
- /cmd_vel 토픽 퍼블리셔: 로봇 주행 속도 명령 발행

2. 차선 중심 좌표 처리 (callback_follow_lane)
- tracing_flag가 켜져 있을 때만 동작
- 차선 중심 좌표가 급격히 변하는 경우 무시 (이상치 필터링)
- 현재 중심 좌표 기준 오차(error) 계산 (160 기준으로 조정)
- PD 제어 (비례 + 미분)로 로봇의 각속도(angular.z) 결정
- 오차 크기에 따라 선속도(linear.x)를 조절하여
- 오차가 작으면 빠르게, 클수록 느리게 주행하도록 설계
- 계산된 속도 메시지를 /cmd_vel에 발행

3. 최대 속도값 업데이트 (callback_get_max_vel)
- 외부에서 최대 속도를 동적으로 설정 가능
- 추후 속도 조절에 반영 예정 (현재는 기본 0.2m/s 사용)

4. 종료 시 처리 (shut_down)
- 노드 종료 시 로봇을 정지시키기 위해 cmd_vel 0 발행



**터미널6 control lane 노드 실행**
```bash
~/turtlebot3_ws$ python3 control_lane_finall.py
```


## error(중앙차선과의 x위치 오차) 와 선속도, 각속도 관계 그래프

<img width="700" alt="image" src="https://github.com/user-attachments/assets/ea715e81-330f-4ae7-be3a-b66236e073e4" />

파라미터를 변경하며 안정적이면서도 빠르게 주행하는 값을 시행착오하며 구함.


## rosbag으로 저장한 트랙 주행 결과(시간-선속도,각속도 그래프)

<img width="600" alt="image" src="https://github.com/user-attachments/assets/eb734fdf-261d-4e49-972a-668f93a96e67" />


---
## 팀 소개

**TEAM C-4조**  
- 이희우  
- 정석환  
- 김동건  
- 김재훈

---

## 라이선스

본 프로젝트는 연구 및 학습 목적으로 개발되었으며, 상업적 사용을 금합니다.

---

