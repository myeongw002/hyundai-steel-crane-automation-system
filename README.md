# hyundai-steel-crane-automation-system
## Installation
```bash
pip install -r requirements.txt
cd sam2
pip install -e .
cd ..
```

## Getting Started
### Download Checkpoints
```bash
cd checkpoints && \
wegt https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_large.pt && \
cd ..
```
## Configuration

### Data Paths
- **VIDEO_DIR**: 이미지 시퀀스가 저장된 폴더 경로
  - 주의: 이미지는 0001.jpg, 0002.jpg로 최소 2장 이상 저장되어야 함
  - 0000.jpg는 ONE_SHOT_IMAGE가 자동으로 복사됨
  
- **PCD_DIR**: LiDAR point cloud 파일(.pcd)이 저장된 폴더 경로
  - 이미지와 파일명이 동일해야 함 (예: 0001.jpg ↔ 0001.pcd)

- **ONE_SHOT_IMAGE**: SAM2의 one-shot segmentation을 위한 기준 이미지
  - 이 이미지가 VIDEO_DIR의 0000.jpg로 복사됨
  - magnet과 plate가 명확하게 보이는 이미지 선택 권장

### One-Shot Prompts
SAM2의 초기 segmentation을 위한 point prompts

- **magnet_pts**: Magnet 객체를 지정하는 2D 픽셀 좌표 [[x1, y1], [x2, y2], ...]
- **magnet_lbl**: 각 point의 레이블 (positive=1, negative=0)
  - positive: 해당 객체에 포함되는 점
  - negative: 해당 객체에서 제외되는 점

- **plate_pts**: Steel plate 객체를 지정하는 2D 픽셀 좌표
- **plate_lbl**: 각 point의 레이블 (positive=1, negative=0)

**예시:**
```python
magnet_pts = np.array([[1010, 460], [1010, 714]], dtype=np.float32)
magnet_lbl = np.array([1, 1], np.int32)  # 두 점 모두 positive
```

### Measurement Calibration

- **MEASUREMENT_OFFSET**: 실측값과 계산값의 차이를 보정하는 offset (단위: mm)
  - 크레인 높이나 센서 위치에 따라 조정 필요
```python
  MEASUREMENT_OFFSET = {
      'P1-P2': 19.4,   # 위쪽 수평 거리 보정값
      'P3-P4': 19.4,   # 아래쪽 수평 거리 보정값
      'P5-P6': 32.4,   # 왼쪽 수직 거리 보정값
      'P7-P8': 32.4    # 오른쪽 수직 거리 보정값
  }
```
  - 최종 측정값 = 계산된 거리 + MEASUREMENT_OFFSET

- **MEASUREMENT_POSITION_OFFSET**: 측정 시작점의 위치 offset (단위: mm, 기본값: 700)
  - Magnet의 edge로부터 P1, P3 시작점까지의 거리
  - 값이 클수록 magnet 중심에 가까운 위치에서 측정

### SAM2 Configuration
- **SAM2_ROOT**: SAM2 라이브러리 경로
- **SAM2_CHECKPOINT**: SAM2 모델 가중치 파일 경로
  - 다운로드: https://github.com/facebookresearch/sam2
- **SAM2_CONFIG**: SAM2 모델 설정 파일 경로

### Camera Calibration Parameters
카메라 내부/외부 파라미터 (calibration 결과)

- **CAMERA_MATRIX**: 카메라 intrinsic matrix (3x3)
```
  [fx  s  cx]
  [0  fy  cy]
  [0   0   1]
```
  - fx, fy: focal length
  - cx, cy: principal point
  - s: skew (일반적으로 0)

- **DIST_COEFFS**: 렌즈 왜곡 계수 [k1, k2, p1, p2, k3]
  - k1, k2, k3: radial distortion
  - p1, p2: tangential distortion

- **T_LIDAR_TO_CAM**: LiDAR → Camera 좌표계 변환 행렬 (4x4)
```
  [R | t]
  [0 | 1]
```
  - R: 3x3 rotation matrix
  - t: 3x1 translation vector

### Processing Parameters

- **MORPH_OPEN_KERNEL_SIZE**: Morphological opening 커널 크기 (기본값: 40)
  - Mask의 노이즈 제거를 위한 파라미터
  - 값이 클수록 더 강한 smoothing

- **LIDAR_DENSE**: LiDAR 기반 mask refinement 활성화 여부 (기본값: False)
  - True: LiDAR point cloud로 plate mask를 정제 (LiDar가 dense하다면 성능이 높음, 속도빠름)
  - False: LiDAR point cloud로 plate mask를 정제 이후 SAM2 segmentation 추가 사용 (LiDar가 sparse하다면 성능이 높음, 상대적으로 느림)

### Output Settings

- **VISUALIZE**: 시각화 이미지 생성 여부 (기본값: True)
  - True: SAM2 결과, mask, box, measurement 이미지 생성
  - False: CSV 결과만 저장

- **OUTPUT_DIR**: 결과 파일이 저장될 폴더 경로 (기본값: "outputs")
  - 생성되는 파일:
    - `{session_id}_sam2.png`: SAM2 segmentation 결과
    - `{session_id}_mask.png`: 정제된 mask 결과
    - `{session_id}_box.png`: Bounding box 결과
    - `{session_id}_result.png`: 최종 측정 결과
    - `{session_id}_measurements.csv`: 측정값 데이터