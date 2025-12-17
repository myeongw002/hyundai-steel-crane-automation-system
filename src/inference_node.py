#!/usr/bin/env python3
"""
inference_node2.py - Refactored SAM2-based plate measurement node
Uses main.py's modular processing pipeline with ROS2 service architecture
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import time
from collections import deque
from typing import Optional, Dict, Tuple, List
import tempfile
import shutil

from hyundai_steel_crane_automation_system.srv import ACWL0001, WLAC0001
from hyundai_steel_crane_automation_system.msg import ACWL0001Body, WLAC0001Body, HeadCR
from builtin_interfaces.msg import Time

from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import Bool
import sensor_msgs_py.point_cloud2 as pc2
import message_filters

import cv2
from cv_bridge import CvBridge
import numpy as np
import os
import sys
import open3d as o3d
from pathlib import Path

# main.py 모듈 임포트
from sam2_wrapper import SAM2Wrapper
from utils import (
    crop_fov, refine_mask, filter_points_in_mask,
    estimate_plane_ransac, refine_mask_with_plane,
    mask_to_box, unproject_to_plane, DistanceCalculator
)
from visualizer import (
    sam2_visualizer, mask_visualizer,
    box_visualizer, measurement_visualizer
)
from data_logger import (
    DataLogger,
    convert_ros_request_to_dict,
    convert_wlac_request_to_dict,
    convert_wlac_response_to_dict
)
from ftp_uploader import FTPUploader, create_ftp_uploader

class InferenceNode2(Node):
    """
    SAM2 기반 판재 크기 측정 노드 (리팩토링 버전)
    main.py의 모듈식 처리 파이프라인 사용
    """
    
    def __init__(self):
        super().__init__('inference_node2')
        
        # 콜백 그룹
        self.service_callback_group = ReentrantCallbackGroup()
        self.sensor_callback_group = ReentrantCallbackGroup()
        
        # 파라미터 선언
        self._declare_parameters()
        
        # 상태 관리
        self._is_processing = False
        self._processing_lock = threading.Lock()
        self._current_task = None
        
        # 센서 데이터 버퍼
        self._synced_buffer = deque(maxlen=self.get_parameter('sensor_buffer_size').value)
        self._synced_lock = threading.Lock()
        
        self._camera_buffer = deque(maxlen=self.get_parameter('sensor_buffer_size').value)
        self._lidar_buffer = deque(maxlen=self.get_parameter('sensor_buffer_size').value)
        self._individual_lock = threading.Lock()
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # DataLogger 초기화
        output_dir = self.get_parameter('output_dir').value
        self.data_logger = DataLogger(output_dir)
        self.get_logger().info(f'✅ DataLogger initialized: {output_dir}')
        
        # FTPUploader 초기화
        ftp_enabled = self.get_parameter('ftp_enabled').value
        if ftp_enabled:
            ftp_host = self.get_parameter('ftp_host').value
            ftp_username = self.get_parameter('ftp_username').value
            ftp_password = self.get_parameter('ftp_password').value
            ftp_remote_dir = self.get_parameter('ftp_remote_dir').value
            
            self.ftp_uploader = create_ftp_uploader(
                host=ftp_host,
                username=ftp_username,
                password=ftp_password,
                remote_base_dir=ftp_remote_dir
            )
            self.get_logger().info(f'✅ FTP uploader initialized: {ftp_host}')
        else:
            self.ftp_uploader = None
            self.get_logger().info('ℹ️  FTP upload disabled')
        
        # Heartbeat 퍼블리셔 (5Hz)
        self.heartbeat_pub = self.create_publisher(Bool, '/heartbeat/inference2', 10)
        self.heartbeat_timer = self.create_timer(0.2, self._publish_heartbeat)
        self.heartbeat_state = True
        self.get_logger().info('✅ Heartbeat publisher initialized (5Hz)')
        
        # ROS2 서비스
        self._setup_services()
        
        # 카메라 캘리브레이션 로드
        self._initialize_camera()
        
        # SAM2 Wrapper 초기화
        self._initialize_sam2()
        
        # 센서 초기화
        self._initialize_sensors()
        
        self.get_logger().info('✅ InferenceNode2 initialized (modular pipeline)')
    
    def _publish_heartbeat(self):
        """5Hz로 heartbeat 발행"""
        msg = Bool()
        msg.data = self.heartbeat_state
        self.heartbeat_pub.publish(msg)
        self.heartbeat_state = not self.heartbeat_state
    
    def _declare_parameters(self):
        """파라미터 선언"""
        # SAM2 설정
        self.declare_parameter('sam2_root', '/home/antlab/ROS2/ros2_ws/src/hyundai_crane_automation/src/sam2')
        self.declare_parameter('sam2_config', 'configs/sam2.1/sam2.1_hiera_l.yaml')
        self.declare_parameter('sam2_checkpoint', '/home/antlab/ROS2/ros2_ws/src/hyundai_crane_automation/src/sam2/sam2.1_hiera_large.pt')
        
        # 카메라 캘리브레이션 (직접 값만 사용)
        # Camera matrix (3x3) - flatten
        self.declare_parameter('camera_matrix', [
            1385.635672, -0.895992, 947.154907,
            0.0, 1388.287107, 607.421636,
            0.0, 0.0, 1.0
        ])
        # Distortion coefficients (1x5)
        self.declare_parameter('dist_coeffs', [-0.085307, 0.079377, 0.000436, -0.000662, 0.0])
        
        # Extrinsic matrix (4x4) - flatten
        self.declare_parameter('t_lidar_to_cam', [
            0.0129229, -0.999916, -0.000756324, -0.121755,
            -0.0144186, 0.000569792, -0.999896, 0.010138,
            0.999813, 0.0129326, -0.0144102, -0.148907,
            0.0, 0.0, 0.0, 1.0
        ])
        
        # 프롬프트 설정 (fallback용)
        self.declare_parameter('obj1_points', [820, 330, 820, 700])
        self.declare_parameter('obj1_labels', [1, 1])
        self.declare_parameter('obj2_points', [820, 260, 820, 800])
        self.declare_parameter('obj2_labels', [1, 1])
        
        # 측정 설정
        self.declare_parameter('confidence_threshold', 0.8)
        self.declare_parameter('processing_timeout', 30.0)
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('use_camera', True)
        self.declare_parameter('output_dir', '/tmp/sam2_results')
        self.declare_parameter('save_data', False)
        self.declare_parameter('visualize', True)
        
        # FTP 업로드 설정
        self.declare_parameter('ftp_enabled', False)
        self.declare_parameter('ftp_host', '172.29.73.49')
        self.declare_parameter('ftp_username', 'plateftp')
        self.declare_parameter('ftp_password', 'plateftp')
        self.declare_parameter('ftp_remote_dir', 'upload')
        
        # main.py 통합 파라미터
        self.declare_parameter('one_shot_image', '')  # One-shot 이미지 경로
        self.declare_parameter('morph_kernel_size', 40)  # Morphology kernel 크기
        self.declare_parameter('measurement_offset_px', 700)  # 측정 위치 offset (픽셀)
        self.declare_parameter('measurement_offset_p1_p2', 19.4)  # 위쪽 길이 offset (mm)
        self.declare_parameter('measurement_offset_p3_p4', 19.4)  # 아래쪽 길이 offset (mm)
        self.declare_parameter('measurement_offset_p5_p6', 32.4)  # 위쪽 너비 offset (mm)
        self.declare_parameter('measurement_offset_p7_p8', 32.4)  # 아래쪽 너비 offset (mm)
        self.declare_parameter('lidar_dense', False)  # Dense LiDAR 모드
        self.declare_parameter('enable_fov_crop', True)  # FOV Crop 활성화
        self.declare_parameter('fov_degrees', 120)  # FOV 각도
        self.declare_parameter('enable_plane_refinement', True)  # 평면 기반 정제
        self.declare_parameter('plane_ransac_iterations', 100)  # RANSAC 반복 횟수
        self.declare_parameter('plane_threshold', 0.05)  # 평면 threshold (m)
        
        # 센서 토픽
        self.declare_parameter('camera_topic', '/camera/image_raw/compressed')
        self.declare_parameter('lidar_topic', '/lidar/points')
        self.declare_parameter('sensor_buffer_size', 10)
        self.declare_parameter('sensor_timeout', 5.0)
        
        # ApproximateTime
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop', 0.1)
    
    def _setup_services(self):
        """ROS2 서비스 설정"""
        self.acwl_service = self.create_service(
            ACWL0001,
            'acwl0001_service',
            self.acwl_callback,
            callback_group=self.service_callback_group
        )
        
        self.wlac_client = self.create_client(
            WLAC0001,
            'wlac0001_service',
            callback_group=self.service_callback_group
        )
        self.get_logger().info('✅ Services set up')
    
    def _initialize_camera(self):
        """카메라 캘리브레이션 로드 (파라미터 방식)"""
        try:
            camera_matrix_flat = self.get_parameter('camera_matrix').value
            dist_coeffs_flat = self.get_parameter('dist_coeffs').value
            t_lidar_to_cam_flat = self.get_parameter('t_lidar_to_cam').value
            
            # Reshape to proper dimensions
            self.camera_matrix = np.array(camera_matrix_flat, dtype=np.float32).reshape(3, 3)
            self.dist_coeffs = np.array(dist_coeffs_flat, dtype=np.float32)
            self.T_lidar_to_cam = np.array(t_lidar_to_cam_flat, dtype=np.float32).reshape(4, 4)
            
            self.get_logger().info('✅ Camera calibration loaded')
            self.get_logger().info(f'   Camera matrix shape: {self.camera_matrix.shape}')
            self.get_logger().info(f'   Dist coeffs shape: {self.dist_coeffs.shape}')
            self.get_logger().info(f'   T_lidar_to_cam shape: {self.T_lidar_to_cam.shape}')
            
        except Exception as e:
            self.get_logger().error(f'Camera calibration load failed: {e}')
            self.camera_matrix = None
            self.dist_coeffs = None
            self.T_lidar_to_cam = None
    
    def _initialize_sam2(self):
        """SAM2Wrapper 초기화"""
        sam2_root = self.get_parameter('sam2_root').value
        checkpoint = self.get_parameter('sam2_checkpoint').value
        model_cfg = self.get_parameter('sam2_config').value
        
        if not os.path.isdir(sam2_root):
            self.get_logger().error(f'SAM2 root not found: {sam2_root}')
            self.sam2_wrapper = None
            return
        
        if not os.path.isfile(checkpoint):
            self.get_logger().error(f'SAM2 checkpoint not found: {checkpoint}')
            self.sam2_wrapper = None
            return
        
        # SAM2 경로를 sys.path에 추가
        if sam2_root not in sys.path:
            sys.path.insert(0, sam2_root)
        
        try:
            # SAM2Wrapper에 전달할 Config 객체 생성
            class SAM2Config:
                SAM2_ROOT = sam2_root
                SAM2_CHECKPOINT = checkpoint
                SAM2_CONFIG = model_cfg
            
            self.sam2_wrapper = SAM2Wrapper(SAM2Config)
            self.sam2_initialized = False
            self.temp_dir = None
            self.video_masks = {}
            
            self.get_logger().info('✅ SAM2Wrapper loaded')
        except Exception as e:
            self.get_logger().error(f'SAM2 initialization failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.sam2_wrapper = None
    
    def _initialize_sensors(self):
        """센서 초기화"""
        use_lidar = self.get_parameter('use_lidar').value
        use_camera = self.get_parameter('use_camera').value
        
        camera_topic = self.get_parameter('camera_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        
        if use_camera and use_lidar:
            self.camera_sub = message_filters.Subscriber(
                self, CompressedImage, camera_topic
            )
            self.lidar_sub = message_filters.Subscriber(
                self, PointCloud2, lidar_topic
            )
            
            self.ts = message_filters.ApproximateTimeSynchronizer(
                [self.camera_sub, self.lidar_sub],
                queue_size=self.get_parameter('sync_queue_size').value,
                slop=self.get_parameter('sync_slop').value
            )
            self.ts.registerCallback(self._synced_callback)
            
            self.sync_enabled = True
            self.get_logger().info('✅ ApproximateTime sync enabled')
        else:
            self.sync_enabled = False
            
            if use_camera:
                self.camera_sub = self.create_subscription(
                    CompressedImage, camera_topic,
                    self._camera_callback, 1,
                    callback_group=self.sensor_callback_group
                )
            
            if use_lidar:
                self.lidar_sub = self.create_subscription(
                    PointCloud2, lidar_topic,
                    self._lidar_callback, 1,
                    callback_group=self.sensor_callback_group
                )
        
        self.camera_available = use_camera
        self.lidar_available = use_lidar
    
    # ==================== 센서 콜백 ====================
    
    def _synced_callback(self, camera_msg: CompressedImage, lidar_msg: PointCloud2):
        """동기화된 센서 데이터 콜백"""
        try:
            camera_time = camera_msg.header.stamp.sec + camera_msg.header.stamp.nanosec * 1e-9
            lidar_time = lidar_msg.header.stamp.sec + lidar_msg.header.stamp.nanosec * 1e-9
            time_diff = abs(camera_time - lidar_time)
            
            with self._synced_lock:
                self._synced_buffer.append({
                    'timestamp': camera_msg.header.stamp,
                    'camera_msg': camera_msg,
                    'lidar_msg': lidar_msg,
                    'time_diff': time_diff
                })
        except Exception as e:
            self.get_logger().error(f'Synced callback error: {e}')
    
    def _camera_callback(self, msg: CompressedImage):
        """Camera 콜백"""
        with self._individual_lock:
            self._camera_buffer.append(msg)
    
    def _lidar_callback(self, msg: PointCloud2):
        """LiDAR 콜백"""
        with self._individual_lock:
            self._lidar_buffer.append(msg)
    
    # ==================== ACWL0001 처리 ====================
    
    def acwl_callback(self, request: ACWL0001.Request, response: ACWL0001.Response):
        """작업 지시 수신"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('ACWL0001 Request Received')
        self.get_logger().info(f'Equipment: {request.body.eqp_cd}')
        self.get_logger().info(f'CR_OP_INDI_ID: {request.body.cr_op_indi_id}')
        self.get_logger().info('=' * 60)
        
        with self._processing_lock:
            if self._is_processing:
                response.accepted = False
                return response
            self._is_processing = True
            self._current_task = request
        
        processing_thread = threading.Thread(
            target=self._process_measurement_task,
            args=(request,),
            daemon=True
        )
        processing_thread.start()
        
        response.accepted = True
        return response
    
    # ==================== 측정 처리 ====================
    
    def _process_measurement_task(self, request: ACWL0001.Request):
        """측정 작업 처리 (main.py 통합 버전 + DataLogger + FTP)"""
        error_result = None
        wlac_request_dict = None
        wlac_response_dict = None
        measurement_records = []
        
        try:
            start_time = time.time()
            sequence_id = request.body.cr_op_indi_id
            
            # 0. ACWL 요청 데이터 저장
            acwl_dict = convert_ros_request_to_dict(request)
            self.data_logger.save_acwl_request(acwl_dict, sequence_id)
            self.get_logger().info(f'💾 ACWL request saved for {sequence_id}')
            
            # 1. 센서 데이터 수집
            try:
                sensor_data_list = self._collect_multiple_sensor_data(request.body)
            except Exception as e:
                error_msg = str(e).lower()
                if 'camera' in error_msg:
                    self.get_logger().error(f'❌ Camera error: {e}')
                    error_result = {'result_code': '0010', 'len_p1_2': 0, 'len_p3_4': 0, 'width_p5_6': 0, 'width_p7_8': 0}
                elif 'lidar' in error_msg:
                    self.get_logger().error(f'❌ LiDAR error: {e}')
                    error_result = {'result_code': '0020', 'len_p1_2': 0, 'len_p3_4': 0, 'width_p5_6': 0, 'width_p7_8': 0}
                else:
                    self.get_logger().error(f'❌ Sensor data collection failed: {e}')
                    error_result = {'result_code': '0030', 'len_p1_2': 0, 'len_p3_4': 0, 'width_p5_6': 0, 'width_p7_8': 0}
                raise
            
            frame_count = len(sensor_data_list)
            self.get_logger().info(f'✅ Collected {frame_count} sensor frames')
            
            # 2. 센서 데이터 저장 (옵션)
            if self.get_parameter('save_data').value:
                self._save_sensor_data(sensor_data_list, sequence_id)
            
            # 3. SAM2 초기화 (비디오 모드)
            self._initialize_sam2_with_prompt(sensor_data_list)
            
            # 4. 각 프레임 측정 (첫 프레임 제외)
            all_measurements = []
            for idx in range(1, frame_count):
                try:
                    self.get_logger().info(f'Measuring frame {idx}/{frame_count-1}...')
                    measurements = self._measure_single_frame(sensor_data_list[idx], idx, sequence_id)
                    all_measurements.append(measurements)
                    
                    # 측정 레코드 저장
                    measurement_records.append({
                        'frame': idx,
                        'P1-P2': measurements.get('P1-P2', 0.0) / 1000.0,  # mm → m
                        'P3-P4': measurements.get('P3-P4', 0.0) / 1000.0,
                        'P5-P6': measurements.get('P5-P6', 0.0) / 1000.0,
                        'P7-P8': measurements.get('P7-P8', 0.0) / 1000.0
                    })
                except Exception as e:
                    self.get_logger().error(f'  Frame {idx} measurement failed: {e}')
            
            if len(all_measurements) == 0:
                raise ValueError("No valid measurements")
            
            # 5. 평균 계산
            avg_measurements = {}
            for key in ['P1-P2', 'P3-P4', 'P5-P6', 'P7-P8']:
                values = [m[key] for m in all_measurements if key in m]
                if values:
                    avg_measurements[key] = np.mean(values)
                else:
                    avg_measurements[key] = 0.0
            
            self.get_logger().info(f'✅ Average measurements (from {len(all_measurements)} frames):')
            for key, value in avg_measurements.items():
                self.get_logger().info(f'   {key}: {value:.1f} mm')
            
            # 6. 결과 검증 및 포맷팅
            result = {
                'result_code': '0000',
                'len_p1_2': int(avg_measurements.get('P1-P2', 0)),
                'len_p3_4': int(avg_measurements.get('P3-P4', 0)),
                'width_p5_6': int(avg_measurements.get('P5-P6', 0)),
                'width_p7_8': int(avg_measurements.get('P7-P8', 0)),
                'p1_p2': avg_measurements.get('P1-P2', 0.0) / 1000.0,  # m 단위로 저장
                'p3_p4': avg_measurements.get('P3-P4', 0.0) / 1000.0,
                'p5_p6': avg_measurements.get('P5-P6', 0.0) / 1000.0,
                'p7_p8': avg_measurements.get('P7-P8', 0.0) / 1000.0
            }
            
            # 7. 측정 데이터 CSV 저장
            self.data_logger.save_measurements_csv(measurement_records, result, sequence_id)
            self.get_logger().info(f'💾 Measurement CSV saved')
            
            # 8. WLAC0001 전송 및 저장
            wlac_request_dict, wlac_response_dict = self._send_result_to_plc(result, request)
            
            if wlac_request_dict and wlac_response_dict:
                self.data_logger.save_wlac_response(
                    wlac_request_dict, wlac_response_dict, sequence_id
                )
                self.get_logger().info(f'💾 WLAC response saved')
            
            # 9. FTP 업로드 (옵션)
            if self.ftp_uploader:
                try:
                    upload_result = self.ftp_uploader.upload_results(
                        self.data_logger.output_dir, sequence_id
                    )
                    if upload_result['success']:
                        self.get_logger().info(
                            f'📤 FTP upload successful: {len(upload_result["uploaded_files"])} files'
                        )
                    else:
                        self.get_logger().error(f'❌ FTP upload failed: {upload_result["errors"]}')
                except Exception as e:
                    self.get_logger().error(f'❌ FTP upload error: {e}')
            
            elapsed = time.time() - start_time
            self.get_logger().info(f'✅ Measurement completed in {elapsed:.2f}s')
            
        except Exception as e:
            self.get_logger().error(f'❌ Measurement failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            
            if error_result is None:
                error_result = {'result_code': '0030', 'len_p1_2': 0, 'len_p3_4': 0, 'width_p5_6': 0, 'width_p7_8': 0}
            
            try:
                self._send_result_to_plc(error_result, request)
            except Exception as e2:
                self.get_logger().error(f'Failed to send error result: {e2}')
        
        finally:
            # 임시 디렉토리 정리
            if self.temp_dir and os.path.exists(self.temp_dir):
                shutil.rmtree(self.temp_dir)
                self.temp_dir = None
            
            # 상태 초기화
            with self._processing_lock:
                self._is_processing = False
                self._current_task = None
            
            self.get_logger().info('🔄 Ready for next request')
    
    
    
    
    def _collect_multiple_sensor_data(self, body: ACWL0001Body) -> list:
        """여러 프레임의 센서 데이터 수집"""
        frame_count = self.get_parameter('sensor_buffer_size').value
        timeout = self.get_parameter('sensor_timeout').value
        max_retries = 3
        
        for attempt in range(max_retries):
            try:
                self.get_logger().info(f'Sensor data collection attempt {attempt + 1}/{max_retries}...')
                start_time = time.time()
                
                if body.camera_use and body.lidar_use and self.sync_enabled:
                    result = self._collect_multiple_synced_data(frame_count, timeout, start_time)
                else:
                    result = self._collect_multiple_individual_data(body, frame_count, timeout, start_time)
                
                self.get_logger().info(f'✅ Successfully collected {len(result)} frames on attempt {attempt + 1}')
                return result
                
            except Exception as e:
                self.get_logger().warning(f'⚠️  Attempt {attempt + 1}/{max_retries} failed: {e}')
                
                if attempt < max_retries - 1:
                    self.get_logger().info(f'Retrying in 0.5 seconds...')
                    time.sleep(0.5)
                else:
                    self.get_logger().error(f'❌ All {max_retries} attempts failed')
                    raise
    
    def _collect_multiple_synced_data(self, frame_count: int, timeout: float, start_time: float) -> list:
        """동기화된 다중 프레임 데이터 수집"""
        sensor_data_list = []
        
        while time.time() - start_time < timeout:
            with self._synced_lock:
                buffer_size = len(self._synced_buffer)
                
                if buffer_size >= frame_count:
                    frames_to_use = list(self._synced_buffer)[-frame_count:]
                    
                    for synced_data in frames_to_use:
                        camera_msg = synced_data['camera_msg']
                        np_arr = np.frombuffer(camera_msg.data, np.uint8)
                        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                        
                        lidar_msg = synced_data['lidar_msg']
                        points = []
                        for point in pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True):
                            points.append([point[0], point[1], point[2]])
                        points_array = np.array(points, dtype=np.float32)
                        
                        sensor_data_list.append({
                            'camera': cv_image,
                            'lidar': points_array,
                            'timestamp': synced_data['timestamp'],
                            'synced': True
                        })
                    
                    break
            
            time.sleep(0.1)
        
        if len(sensor_data_list) == 0:
            raise TimeoutError(f'Not enough synchronized frames (required: {frame_count})')
        
        return sensor_data_list
    
    def _collect_multiple_individual_data(self, body: ACWL0001Body, frame_count: int, timeout: float, start_time: float) -> list:
        """개별 다중 프레임 데이터 수집"""
        sensor_data_list = []
        
        camera_msgs = []
        if body.camera_use and self.camera_available:
            while time.time() - start_time < timeout:
                with self._individual_lock:
                    buffer_size = len(self._camera_buffer)
                    if buffer_size >= frame_count:
                        camera_msgs = list(self._camera_buffer)[-frame_count:]
                        break
                time.sleep(0.1)
            
            if len(camera_msgs) == 0:
                raise TimeoutError(f'Camera: Not enough frames (required: {frame_count})')
        
        lidar_msgs = []
        if body.lidar_use and self.lidar_available:
            while time.time() - start_time < timeout:
                with self._individual_lock:
                    buffer_size = len(self._lidar_buffer)
                    if buffer_size >= frame_count:
                        lidar_msgs = list(self._lidar_buffer)[-frame_count:]
                        break
                time.sleep(0.1)
            
            if len(lidar_msgs) == 0:
                raise TimeoutError(f'LiDAR: Not enough frames (required: {frame_count})')
        
        min_count = min(len(camera_msgs), len(lidar_msgs)) if camera_msgs and lidar_msgs else \
                    len(camera_msgs) if camera_msgs else len(lidar_msgs)
        
        for i in range(min_count):
            data = {'synced': False}
            
            if camera_msgs:
                camera_msg = camera_msgs[i]
                np_arr = np.frombuffer(camera_msg.data, np.uint8)
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                data['camera'] = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                data['timestamp'] = camera_msg.header.stamp
            
            if lidar_msgs:
                lidar_msg = lidar_msgs[i]
                points = []
                for point in pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True):
                    points.append([point[0], point[1], point[2]])
                data['lidar'] = np.array(points, dtype=np.float32)
            
            sensor_data_list.append(data)
        
        return sensor_data_list
    
    def _save_sensor_data(self, sensor_data_list: list, cr_op_indi_id: str):
        """센서 데이터를 디스크에 저장"""
        output_dir = self.get_parameter('output_dir').value
        
        if cr_op_indi_id:
            base_dir = os.path.join(output_dir, cr_op_indi_id)
        else:
            base_dir = output_dir
        
        image_dir = os.path.join(base_dir, 'image')
        pcd_dir = os.path.join(base_dir, 'pcd')
        
        os.makedirs(image_dir, exist_ok=True)
        os.makedirs(pcd_dir, exist_ok=True)
        
        self.get_logger().info(f'💾 Saving sensor data to {base_dir}...')
        
        for idx, sensor_data in enumerate(sensor_data_list):
            try:
                if 'camera' in sensor_data and sensor_data['camera'] is not None:
                    image_path = os.path.join(image_dir, f'{idx:04d}.jpg')
                    bgr_image = cv2.cvtColor(sensor_data['camera'], cv2.COLOR_RGB2BGR)
                    cv2.imwrite(image_path, bgr_image)
                
                if 'lidar' in sensor_data and sensor_data['lidar'] is not None:
                    pcd_path = os.path.join(pcd_dir, f'{idx:04d}.pcd')
                    self._save_pcd(sensor_data['lidar'], pcd_path)
                    
            except Exception as e:
                self.get_logger().error(f'Failed to save frame {idx}: {e}')
        
        self.get_logger().info(f'✅ Saved {len(sensor_data_list)} frames')
    
    def _save_pcd(self, points: np.ndarray, filepath: str):
        """포인트 클라우드를 PCD 파일로 저장"""
        num_points = points.shape[0]
        
        with open(filepath, 'w') as f:
            f.write('# .PCD v0.7 - Point Cloud Data file format\n')
            f.write('VERSION 0.7\n')
            f.write('FIELDS x y z\n')
            f.write('SIZE 4 4 4\n')
            f.write('TYPE F F F\n')
            f.write('COUNT 1 1 1\n')
            f.write(f'WIDTH {num_points}\n')
            f.write('HEIGHT 1\n')
            f.write('VIEWPOINT 0 0 0 1 0 0 0\n')
            f.write(f'POINTS {num_points}\n')
            f.write('DATA ascii\n')
            
            for point in points:
                f.write(f'{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n')
    
    # ==================== 결과 검증 및 전송 ====================
    
    def _validate_result(self, inference_result: dict, body: ACWL0001Body) -> dict:
        """결과 검증"""
        length = inference_result['length']
        width = inference_result['width']
        
        result_code = '0000'
        
        # if abs(length) > 200 or abs(width) > 200:
        #     result_code = '0030'
        #     self.get_logger().warning(f'❌ Measurement error')
        #     self.get_logger().warning(f'  Length: {length}mm')
        #     self.get_logger().warning(f'  Width: {width}mm')
        
        return {
            'result_code': result_code,
            'length': length,
            'width': width
        }
    
    def _send_result_to_plc(self, result: dict, original_request: ACWL0001.Request) -> Tuple[Optional[dict], Optional[dict]]:
        """
        PLC로 결과 전송
        
        Returns:
            (wlac_request_dict, wlac_response_dict) 튜플
        """
        wlac_request = WLAC0001.Request()
        
        wlac_request.head = HeadCR()
        wlac_request.head.msg_id = 'WLAC0001'
        wlac_request.head.date = time.strftime('%Y-%m-%d')
        wlac_request.head.time = time.strftime('%H-%M-%S')
        wlac_request.head.form = 'I'
        wlac_request.head.msg_len = 60
        wlac_request.head.filler = ''
        
        wlac_request.body = WLAC0001Body()
        wlac_request.body.eqp_cd = original_request.body.eqp_cd
        wlac_request.body.result_code = result['result_code']
        wlac_request.body.len_result_p1_2 = result['len_p1_2']
        wlac_request.body.len_result_p3_4 = result['len_p3_4']
        wlac_request.body.width_result_p5_6 = result['width_p5_6']
        wlac_request.body.width_result_p7_8 = result['width_p7_8']
        
        now = time.time()
        wlac_request.body.stamp = Time()
        wlac_request.body.stamp.sec = int(now)
        wlac_request.body.stamp.nanosec = int((now % 1) * 1e9)
        
        if not self.wlac_client.wait_for_service(timeout_sec=5.0):
            raise Exception('WLAC0001 service not available')
        
        future = self.wlac_client.call_async(wlac_request)
        
        timeout_start = time.time()
        while not future.done():
            if time.time() - timeout_start > 10.0:
                raise TimeoutError('WLAC0001 response timeout')
            time.sleep(0.01)
        
        response = future.result()
        
        # 딕셔너리 변환
        wlac_req_dict = convert_wlac_request_to_dict(wlac_request)
        wlac_res_dict = convert_wlac_response_to_dict(response)
        
        if response.stored:
            self.get_logger().info('✅ Result sent to PLC')
        else:
            raise Exception(f'PLC rejected: {response.error}')
        
        return wlac_req_dict, wlac_res_dict
    
    # ==================== main.py 통합: 측정 파이프라인 ====================
    
    def _initialize_sam2_with_prompt(self, sensor_data_list: list):
        """
        SAM2 비디오 모드 초기화 (main.py 방식)
        여러 프레임을 임시 디렉토리에 저장 → video_run으로 마스크 전파
        """
        # 1. 임시 디렉토리 정리 및 생성
        if self.temp_dir and os.path.exists(self.temp_dir):
            shutil.rmtree(self.temp_dir)
        self.temp_dir = tempfile.mkdtemp(prefix='sam2_video_')
        
        # 2. One-shot 이미지 처리
        one_shot = self.get_parameter('one_shot_image').value
        if one_shot and os.path.exists(one_shot):
            shutil.copy2(one_shot, os.path.join(self.temp_dir, '0000.jpg'))
            self.get_logger().info(f'Using one-shot image: {one_shot}')
        else:
            # 첫 프레임 사용
            cv2.imwrite(
                os.path.join(self.temp_dir, '0000.jpg'),
                cv2.cvtColor(sensor_data_list[0]['camera'], cv2.COLOR_RGB2BGR)
            )
        
        # 3. 나머지 프레임 저장
        for idx in range(1, len(sensor_data_list)):
            cv2.imwrite(
                os.path.join(self.temp_dir, f'{idx:04d}.jpg'),
                cv2.cvtColor(sensor_data_list[idx]['camera'], cv2.COLOR_RGB2BGR)
            )
        
        # 4. 프롬프트 준비
        obj1_points_flat = self.get_parameter('obj1_points').value
        obj1_points = np.array(obj1_points_flat).reshape(-1, 2).tolist()
        
        obj2_points_flat = self.get_parameter('obj2_points').value
        obj2_points = np.array(obj2_points_flat).reshape(-1, 2).tolist()
        
        prompts = [
            {'id': 1, 'points': obj1_points, 'labels': self.get_parameter('obj1_labels').value},
            {'id': 2, 'points': obj2_points, 'labels': self.get_parameter('obj2_labels').value}
        ]
        
        # 5. SAM2 video_run (마스크 전파)
        self.get_logger().info('Running SAM2 video segmentation...')
        self.video_masks = self.sam2_wrapper.video_run(self.temp_dir, prompts)
        self.sam2_initialized = True
        
        self.get_logger().info(f'✅ SAM2 initialized with {len(self.video_masks)} frames')
    
    def _measure_single_frame(self, sensor_data: dict, frame_idx: int, sequence_id: str = '') -> dict:
        """
        단일 프레임 측정 (main.py의 process_frame 로직)
        """
        image = sensor_data['camera']
        points_lidar = sensor_data['lidar']
        H, W = image.shape[:2]
        
        # 1. FOV Crop (옵션)
        if self.get_parameter('enable_fov_crop').value:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_lidar)
            pcd = crop_fov(pcd, self.get_parameter('fov_degrees').value)
            points_lidar = np.asarray(pcd.points)
        
        # 2. Mask 가져오기
        masks = self.video_masks.get(frame_idx)
        if masks is None:
            raise ValueError(f"No mask for frame {frame_idx}")
        
        # 3. Mask 정제 (morphology)
        kernel_size = self.get_parameter('morph_kernel_size').value
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_size, kernel_size))
        magnet_mask = refine_mask(masks.get(1), kernel)
        plate_mask = refine_mask(masks.get(2), kernel)
        
        # 4. Plate 영역 포인트 추출
        plate_points, plate_points_cam, plate_indices = filter_points_in_mask(
            points_lidar, plate_mask, self.T_lidar_to_cam, self.camera_matrix
        )
        
        if len(plate_points_cam) < 3:
            raise ValueError(f"Not enough points in plate mask: {len(plate_points_cam)}")
        
        # 5. 평면 추정 (RANSAC)
        plane_normal, plane_d, plane_centroid, _, _ = estimate_plane_ransac(
            plate_points_cam,
            iterations=self.get_parameter('plane_ransac_iterations').value,
            threshold=self.get_parameter('plane_threshold').value
        )
        
        if plane_normal is None:
            raise ValueError("Plane estimation failed")
        
        # 6. 평면 기반 마스크 정제 (옵션)
        if self.get_parameter('enable_plane_refinement').value:
            plate_mask, _ = refine_mask_with_plane(
                points_lidar, plate_mask, plate_indices,
                plane_normal, plane_d, self.T_lidar_to_cam, self.camera_matrix,
                threshold=self.get_parameter('plane_threshold').value
            )
        
        # 7. SAM2 image_run (LIDAR_DENSE=False 모드)
        if not self.get_parameter('lidar_dense').value:
            plate_corners_2d = mask_to_box(plate_mask, rotate=False, inscribed=False)
            
            # 외곽 음수 포인트 생성
            x_coords = np.arange(100, W - 100, 200)
            outlier_points = np.vstack([
                np.column_stack([x_coords, np.full_like(x_coords, 100)]),
                np.column_stack([x_coords, np.full_like(x_coords, H - 100)])
            ])
            
            plate_mask = self.sam2_wrapper.image_run(
                cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
                plate_corners_2d,
                outlier_points
            )
            plate_mask = refine_mask(plate_mask, kernel)
        
        # 8. Mask → Box 변환
        plate_corners_2d = mask_to_box(plate_mask, rotate=True, inscribed=False)
        magnet_corners_2d = mask_to_box(magnet_mask, rotate=False, inscribed=True)
        
        # 9. 3D Unprojection
        plate_corners_3d = unproject_to_plane(
            plate_corners_2d, plane_normal, plane_centroid, self.camera_matrix, self.dist_coeffs
        )
        magnet_corners_3d = unproject_to_plane(
            magnet_corners_2d, plane_normal, plane_centroid, self.camera_matrix, self.dist_coeffs
        )
        
        # 10. 거리 계산
        measurement_offsets = {
            'P1-P2': self.get_parameter('measurement_offset_p1_p2').value,
            'P3-P4': self.get_parameter('measurement_offset_p3_p4').value,
            'P5-P6': self.get_parameter('measurement_offset_p5_p6').value,
            'P7-P8': self.get_parameter('measurement_offset_p7_p8').value,
        }
        
        calculator = DistanceCalculator(
            magnet_corners_2d, magnet_corners_3d, plate_corners_3d,
            plane_normal, plane_centroid, self.camera_matrix, self.dist_coeffs,
            self.get_parameter('measurement_offset_px').value,
            measurement_offsets
        )
        
        measurements, measure_points_3d = calculator.calculate()
        
        if measurements is None:
            raise ValueError("Distance calculation failed")
        
        # 11. 시각화 (옵션)
        if self.get_parameter('visualize').value:
            self._save_visualizations(
                image, masks, magnet_mask, plate_mask,
                magnet_corners_2d, plate_corners_2d,
                measure_points_3d, measurements, frame_idx, sequence_id
            )
        
        return measurements
    
    def _save_visualizations(self, image, masks, magnet_mask, plate_mask,
                            magnet_corners_2d, plate_corners_2d,
                            measure_points_3d, measurements, frame_idx, sequence_id: str = ''):
        """4종 시각화 저장 (main.py 방식)"""
        # DataLogger와 동일한 경로 사용
        if sequence_id:
            output_dir = Path(self.data_logger.output_dir) / sequence_id / "results"
        else:
            output_dir = Path(self.data_logger.output_dir)
        output_dir.mkdir(exist_ok=True, parents=True)
        
        frame_id = f"frame_{frame_idx:04d}"
        
        # 1. SAM2 segmentation 결과
        sam2_visualizer(image.copy(), masks, frame_id, output_dir)
        
        # 2. Mask 정제 결과
        mask_visualizer(image.copy(), [magnet_mask, plate_mask], frame_id, output_dir)
        
        # 3. Box 결과
        box_visualizer(image.copy(), [magnet_corners_2d, plate_corners_2d], frame_id, output_dir)
        
        # 4. 측정 결과
        measurement_visualizer(
            image.copy(), [magnet_corners_2d, plate_corners_2d],
            measure_points_3d, self.camera_matrix, self.dist_coeffs,
            measurements, frame_id, output_dir
        )
        
        self.get_logger().info(f'   Saved visualizations for frame {frame_idx}')


def main(args=None):
    rclpy.init(args=args)
    
    node = InferenceNode2()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.heartbeat_timer.cancel()
        except:
            pass
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
