# Hyundai Steel Crane Automation System

ROS2 기반 현대제철 크레인 자동화 시스템으로, SAM2(Segment Anything Model 2)를 활용한 철판 인식 및 측정, PLC 통신을 통한 크레인 제어 기능을 제공합니다.

## 📋 목차
- [개요](#개요)
- [시스템 아키텍처](#시스템-아키텍처)
- [주요 기능](#주요-기능)
- [설치 방법](#설치-방법)
- [사용 방법](#사용-방법)
- [설정](#설정)
- [노드 설명](#노드-설명)
- [메시지 및 서비스](#메시지-및-서비스)
- [트러블슈팅](#트러블슈팅)

## 🎯 개요

본 시스템은 현대제철 크레인 작업 현장에서 철판 자동 인식 및 측정을 통한 크레인 자동화를 목표로 합니다.

### 핵심 기술
- **ROS2 Humble**: 로봇 운영 체제
- **SAM2**: Meta의 Segment Anything Model 2를 활용한 철판 세그멘테이션
- **센서 융합**: RGB 카메라 + LiDAR 데이터 융합
- **PLC 통신**: TCP/IP 기반 크레인 제어 시스템 연동

## 🏗️ 시스템 아키텍처

```
┌─────────────────┐        ┌──────────────────┐        ┌─────────────┐
│   PLC System    │◄──────►│  Socket Bridge   │◄──────►│  Inference  │
│  (172.29.84.27) │  TCP   │      Node        │  ROS2  │    Node     │
└─────────────────┘        └──────────────────┘        └─────────────┘
                                   │                           │
                                   │                           │
                                   ▼                           ▼
                           ACWL0001/WLAC0001           SAM2 + Sensor
                              Services                    Fusion
```

### 데이터 흐름
1. **PLC → Socket Bridge**: ACWL0001 요청 (작업 지시)
2. **Socket Bridge → Inference**: ROS2 서비스 호출
3. **Inference**: 센서 데이터 수집 및 SAM2 기반 측정
4. **Inference → Socket Bridge**: WLAC0001 응답 (측정 결과)
5. **Socket Bridge → PLC**: TCP 응답 전송

## ✨ 주요 기능

### 1. 철판 인식 및 측정
- SAM2 기반 자동 세그멘테이션
- RGB-D 데이터 융합을 통한 3D 위치 추정
- RANSAC 평면 추정 및 정제
- 8포인트 거리 측정 (수평/수직)

### 2. PLC 통신
- 비동기 TCP/IP 통신
- 프로토콜 자동 파싱 (ACWL0001/WLAC0001)
- 다중 클라이언트 지원
- Heartbeat 모니터링 (5Hz)

### 3. 데이터 로깅
- 요청/응답 이력 자동 저장
- FTP 기반 원격 백업
- 시각화 이미지 생성

## 🔧 설치 방법

### 1. 시스템 요구사항
- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- CUDA 11.8+ (GPU 추론 시)

### 2. ROS2 설치
```bash
# ROS2 Humble 설치 (이미 설치된 경우 생략)
sudo apt update && sudo apt install -y ros-humble-desktop
source /opt/ros/humble/setup.bash
```

### 3. 의존성 설치
```bash
cd ~/ROS2/crane_ws/src/hyundai-steel-crane-automation-system

# Python 패키지 설치
pip install -r requirements.txt

# SAM2 설치
cd src/sam2
pip install -e .
cd ../..
```

### 4. 패키지 빌드
```bash
cd ~/ROS2/crane_ws
colcon build --packages-select hyundai_steel_crane_automation_system
source install/setup.bash
```

### 5. SAM2 모델 다운로드
```bash
# SAM2 체크포인트 다운로드
cd src/sam2
wget https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_large.pt
cd ../..
```

## 🚀 사용 방법

### 1. 시스템 실행
```bash
# 전체 시스템 런치
ros2 launch hyundai_steel_crane_automation_system crane.launch.py
```

### 2. 개별 노드 실행
```bash
# Socket Bridge Node
ros2 run hyundai_steel_crane_automation_system socket_node.py

# Inference Node
ros2 run hyundai_steel_crane_automation_system inference_node.py
```

### 3. 서비스 테스트
```bash
# ACWL0001 서비스 호출 예제
ros2 service call /acwl0001_service hyundai_steel_crane_automation_system/srv/ACWL0001 "{
  header: {
    msg_div: 'ACWL',
    msg_seq: '0001',
    trx_id: 'TEST001',
    msg_len: 150
  },
  body: {
    crane_no: '01',
    job_no: 'JOB001'
  }
}"
```

## ⚙️ 설정

### config.yaml 주요 설정

#### Socket Bridge Node
```yaml
/socket_bridge_node:
  ros__parameters:
    listen_ip: '172.29.84.80'      # 수신 IP
    listen_port: 2000              # 수신 포트
    plc_ip: '172.29.84.27'        # PLC IP
    plc_port: 2000                 # PLC 포트
    buffer_size: 4096              # 버퍼 크기
```

#### Inference Node
```yaml
/inference_node:
  ros__parameters:
    # SAM2 모델 설정
    sam2_root: '/path/to/sam2'
    sam2_config: 'configs/sam2.1/sam2.1_hiera_l.yaml'
    sam2_checkpoint: '/path/to/sam2.1_hiera_large.pt'
    
    # 카메라 캘리브레이션
    camera_matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    dist_coeffs: [k1, k2, p1, p2, k3]
    
    # LiDAR-Camera 변환
    t_lidar_to_cam: [4x4 transformation matrix]
    
    # 측정 파라미터
    timeout_sec: 10.0
    max_queue_size: 50
```

### 카메라 캘리브레이션 파라미터

#### CAMERA_MATRIX (3x3)
```
[fx  s  cx]
[0  fy  cy]
[0   0   1]
```
- **fx, fy**: Focal length (초점 거리)
- **cx, cy**: Principal point (주점)
- **s**: Skew (왜곡, 일반적으로 0)

#### DIST_COEFFS [k1, k2, p1, p2, k3]
- **k1, k2, k3**: Radial distortion (방사 왜곡)
- **p1, p2**: Tangential distortion (접선 왜곡)

#### T_LIDAR_TO_CAM (4x4)
LiDAR → Camera 좌표계 변환 행렬
```
[R | t]
[0 | 1]
```
- **R**: 3x3 rotation matrix
- **t**: 3x1 translation vector

### 측정 보정 파라미터

#### MEASUREMENT_OFFSET
실측값과 계산값의 차이를 보정하는 offset (단위: mm)
```python
MEASUREMENT_OFFSET = {
    'P1-P2': 19.4,   # 위쪽 수평 거리 보정값
    'P3-P4': 19.4,   # 아래쪽 수평 거리 보정값
    'P5-P6': 32.4,   # 왼쪽 수직 거리 보정값
    'P7-P8': 32.4    # 오른쪽 수직 거리 보정값
}
```
- 최종 측정값 = 계산된 거리 + MEASUREMENT_OFFSET
- 크레인 높이나 센서 위치에 따라 조정 필요

#### MEASUREMENT_POSITION_OFFSET
측정 시작점의 위치 offset (단위: mm, 기본값: 700)
- Magnet의 edge로부터 P1, P3 시작점까지의 거리
- 값이 클수록 magnet 중심에 가까운 위치에서 측정

### 처리 파라미터

#### MORPH_OPEN_KERNEL_SIZE
Morphological opening 커널 크기 (기본값: 40)
- Mask의 노이즈 제거를 위한 파라미터
- 값이 클수록 더 강한 smoothing

#### LIDAR_DENSE
LiDAR 기반 mask refinement 활성화 여부 (기본값: False)
- **True**: LiDAR point cloud로만 plate mask를 정제 (LiDAR가 dense한 경우 성능 높음, 속도 빠름)
- **False**: LiDAR + SAM2 segmentation 병행 (LiDAR가 sparse한 경우 성능 높음, 상대적으로 느림)

#### VISUALIZE
시각화 이미지 생성 여부 (기본값: True)
- **True**: SAM2 결과, mask, box, measurement 이미지 생성
- **False**: CSV 결과만 저장

## 📦 노드 설명

### Socket Bridge Node (`socket_node.py`)
**역할**: PLC와 ROS2 시스템 간 통신 브릿지

**주요 기능**:
- TCP 서버 운영 (PLC로부터 요청 수신)
- 프로토콜 파싱 (ACWL0001/WLAC0001)
- ROS2 서비스 클라이언트/서버 역할
- Heartbeat 퍼블리싱 (5Hz)

**토픽**:
- `/heartbeat/socket` (std_msgs/Bool) - 노드 상태

**서비스**:
- 클라이언트: `/acwl0001_service`
- 서버: `/wlac0001_service`

### Inference Node (`inference_node.py`)
**역할**: SAM2 기반 철판 인식 및 측정

**주요 기능**:
- 센서 데이터 동기화 (RGB + PointCloud)
- SAM2 세그멘테이션 수행
- 3D 위치 및 크기 측정
- 결과 시각화 및 로깅

**토픽 구독**:
- `/camera/color/image_raw/compressed` (sensor_msgs/CompressedImage)
- `/camera/depth/points` (sensor_msgs/PointCloud2)

**서비스**:
- 서버: `/acwl0001_service`
- 클라이언트: `/wlac0001_service`

### 보조 모듈

| 모듈 | 설명 |
|------|------|
| `protocol_parser.py` | ACWL0001/WLAC0001 프로토콜 인코딩/디코딩 |
| `sam2_wrapper.py` | SAM2 모델 초기화 및 추론 |
| `utils.py` | 센서 데이터 전처리, 평면 추정 (RANSAC) |
| `visualizer.py` | SAM2 마스크, 바운딩 박스, 측정 결과 시각화 |
| `data_logger.py` | 요청/응답 로그, 이미지 저장 |
| `ftp_uploader.py` | 로그 데이터 원격 백업 |

## 📡 메시지 및 서비스

### 메시지 타입

#### HeadCR.msg
```
string msg_div      # 메시지 구분 (ACWL/WLAC)
string msg_seq      # 메시지 순번
string trx_id       # 트랜잭션 ID
int32 msg_len       # 메시지 길이
```

#### ACWL0001Body.msg
PLC → ROS2 작업 요청 메시지

#### WLAC0001Body.msg
ROS2 → PLC 측정 결과 메시지

### 서비스 타입

#### ACWL0001.srv
```
HeadCR header
ACWL0001Body body
---
HeadCR header
WLAC0001Body body
```

#### WLAC0001.srv
```
HeadCR header
WLAC0001Body body
---
HeadCR header
WLAC0001Body body
```

## 🔍 트러블슈팅

### 일반적인 문제

#### 1. SAM2 모델 로드 실패
```bash
# 체크포인트 경로 확인
ls -lh src/sam2/*.pt

# config.yaml의 sam2_checkpoint 경로 확인
```

#### 2. PLC 통신 연결 실패
```bash
# 네트워크 연결 확인
ping 172.29.84.27

# 포트 사용 확인
netstat -tulpn | grep 2000
```

#### 3. 센서 데이터 수신 안됨
```bash
# 토픽 리스트 확인
ros2 topic list

# 토픽 데이터 확인
ros2 topic echo /camera/color/image_raw/compressed --once
ros2 topic hz /camera/depth/points
```

#### 4. 빌드 오류
```bash
# 의존성 재설치
rosdep install --from-paths src --ignore-src -r -y

# 클린 빌드
colcon build --packages-select hyundai_steel_crane_automation_system --cmake-clean-cache
```

#### 5. 측정 정확도 문제
- `MEASUREMENT_OFFSET` 값 조정
- 카메라 캘리브레이션 재수행
- LiDAR-Camera 외부 파라미터 검증

## 📂 디렉토리 구조

```
hyundai-steel-crane-automation-system/
├── config/                 # 설정 파일
│   ├── config.yaml
│   └── config2.yaml
├── launch/                 # 런치 파일
│   └── crane.launch.py
├── msg/                    # 메시지 정의
│   ├── ACWL0001Body.msg
│   ├── WLAC0001Body.msg
│   └── HeadCR.msg
├── srv/                    # 서비스 정의
│   ├── ACWL0001.srv
│   └── WLAC0001.srv
├── src/                    # ROS2 노드 및 통신
│   ├── socket_node.py      # PLC 통신 노드
│   ├── inference_node.py   # 측정 노드
│   ├── acwl_client.py      # ACWL0001 테스트 클라이언트
│   ├── wlac_server.py      # WLAC0001 Mock WCS 서버
│   ├── wlac_client.py      # WLAC0001 테스트 클라이언트
│   ├── protocol_parser.py  # 프로토콜 파서
│   ├── data_logger.py      # 로거
│   └── ftp_uploader.py     # FTP 업로더
├── modules/                # 핵심 처리 모듈
│   ├── main.py             # 메인 처리 로직
│   ├── data_loader.py      # 데이터 로더
│   ├── sam2_wrapper.py     # SAM2 래퍼
│   ├── utils.py            # 유틸리티
│   ├── visualizer.py       # 시각화
│   └── sam2/               # SAM2 모델
├── params/                 # 파라미터 파일
│   ├── intrinsic.csv       # 카메라 내부 파라미터
│   ├── intrinsic2.csv
│   ├── transform.txt       # LiDAR-Camera 변환 행렬
│   └── transform2.txt
├── docs/                   # 문서
├── oneshot/                # 일회성 스크립트
├── CMakeLists.txt
├── package.xml
├── requirements.txt
└── README.md
```