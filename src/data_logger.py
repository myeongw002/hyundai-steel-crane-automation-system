"""
data_logger.py - Data logging utilities for saving measurement results
Provides functionality to save ACWL/WLAC protocol data and measurement results
"""

import csv
import os
import time
import cv2
import numpy as np
from pathlib import Path
from typing import List, Dict, Optional


class DataLogger:
    """
    데이터 로거 클래스
    측정 결과와 프로토콜 데이터를 CSV 및 텍스트 파일로 저장
    """
    
    def __init__(self, output_dir: str = '/tmp/measurement_results'):
        """
        Args:
            output_dir: 출력 디렉토리 기본 경로
        """
        self.output_dir = output_dir
    
    def save_acwl_request(self, request_data: dict, sequence_id: str = '') -> str:
        """
        ACWL 요청 데이터를 텍스트 파일로 저장 (모든 필드 포함)
        
        Args:
            request_data: ACWL 요청 데이터 딕셔너리
            sequence_id: 시퀀스 ID (cr_op_indi_id)
        
        Returns:
            저장된 파일 경로
        """
        base_dir = self._get_base_dir(sequence_id)
        results_dir = os.path.join(base_dir, 'results')
        os.makedirs(results_dir, exist_ok=True)
        
        txt_path = os.path.join(results_dir, 'acwl_wlac_data.txt')
        
        with open(txt_path, 'w', encoding='utf-8') as f:
            f.write('=' * 80 + '\n')
            f.write('ACWL0001 REQUEST DATA\n')
            f.write('=' * 80 + '\n')
            f.write(f'Timestamp: {time.strftime("%Y-%m-%d %H:%M:%S")}\n\n')
            
            # Header 정보
            f.write('[HEADER]\n')
            f.write(f'  msg_id        : {request_data["head"]["msg_id"]}\n')
            f.write(f'  date          : {request_data["head"]["date"]}\n')
            f.write(f'  time          : {request_data["head"]["time"]}\n')
            f.write(f'  form          : {request_data["head"]["form"]}\n')
            f.write(f'  msg_len       : {request_data["head"]["msg_len"]}\n')
            f.write(f'  filler        : {request_data["head"]["filler"]}\n\n')
            
            # Body 정보
            f.write('[BODY]\n')
            f.write(f'  eqp_cd        : {request_data["body"]["eqp_cd"]}\n')
            f.write(f'  dx_direct     : {request_data["body"]["dx_direct"]} (0:정방향, 1:역방향)\n')
            f.write(f'  dy_direct     : {request_data["body"]["dy_direct"]} (0:정방향, 1:역방향)\n')
            f.write(f'  dz_mm         : {request_data["body"]["dz_mm"]} mm\n')
            f.write(f'  cr_op_indi_id : {request_data["body"]["cr_op_indi_id"]}\n')
            f.write(f'  plate_top_len : {request_data["body"]["plate_top_len"]}\n')
            f.write(f'  plate_max_width: {request_data["body"]["plate_max_width"]}\n')
            f.write(f'  lidar_use     : {request_data["body"]["lidar_use"]}\n')
            f.write(f'  camera_use    : {request_data["body"]["camera_use"]}\n')
            f.write(f'  stamp.sec     : {request_data["body"]["stamp"]["sec"]}\n')
            f.write(f'  stamp.nanosec : {request_data["body"]["stamp"]["nanosec"]}\n\n')
            
            # Raw 데이터
            f.write('[RAW DATA]\n')
            f.write(f'  {request_data["body"]["raw"]}\n\n')
        
        return txt_path
    
    def save_wlac_response(self, wlac_request_data: dict, wlac_response_data: dict, 
                          sequence_id: str = '') -> str:
        """
        WLAC 응답 데이터를 텍스트 파일에 추가 (모든 필드 포함)
        
        Args:
            wlac_request_data: WLAC 요청 데이터 딕셔너리
            wlac_response_data: WLAC 응답 데이터 딕셔너리
            sequence_id: 시퀀스 ID
        
        Returns:
            저장된 파일 경로
        """
        base_dir = self._get_base_dir(sequence_id)
        results_dir = os.path.join(base_dir, 'results')
        os.makedirs(results_dir, exist_ok=True)
        
        txt_path = os.path.join(results_dir, 'acwl_wlac_data.txt')
        
        with open(txt_path, 'a', encoding='utf-8') as f:
            f.write('=' * 80 + '\n')
            f.write('WLAC0001 REQUEST DATA\n')
            f.write('=' * 80 + '\n')
            f.write(f'Timestamp: {time.strftime("%Y-%m-%d %H:%M:%S")}\n\n')
            
            # Header 정보
            f.write('[HEADER]\n')
            f.write(f'  msg_id        : {wlac_request_data["head"]["msg_id"]}\n')
            f.write(f'  date          : {wlac_request_data["head"]["date"]}\n')
            f.write(f'  time          : {wlac_request_data["head"]["time"]}\n')
            f.write(f'  form          : {wlac_request_data["head"]["form"]}\n')
            f.write(f'  msg_len       : {wlac_request_data["head"]["msg_len"]}\n')
            f.write(f'  filler        : {wlac_request_data["head"]["filler"]}\n\n')
            
            # Body 정보
            f.write('[BODY]\n')
            f.write(f'  eqp_cd           : {wlac_request_data["body"]["eqp_cd"]}\n')
            f.write(f'  result_code      : {wlac_request_data["body"]["result_code"]}\n')
            f.write(f'  len_result_p1_2  : {wlac_request_data["body"]["len_result_p1_2"]} mm\n')
            f.write(f'  len_result_p3_4  : {wlac_request_data["body"]["len_result_p3_4"]} mm\n')
            f.write(f'  width_result_p5_6: {wlac_request_data["body"]["width_result_p5_6"]} mm\n')
            f.write(f'  width_result_p7_8: {wlac_request_data["body"]["width_result_p7_8"]} mm\n')
            f.write(f'  stamp.sec        : {wlac_request_data["body"]["stamp"]["sec"]}\n')
            f.write(f'  stamp.nanosec    : {wlac_request_data["body"]["stamp"]["nanosec"]}\n\n')
            
            f.write('-' * 80 + '\n')
            f.write('WLAC0001 RESPONSE DATA\n')
            f.write('-' * 80 + '\n')
            f.write(f'  stored        : {wlac_response_data.get("stored", False)}\n')
            f.write(f'  error         : {wlac_response_data.get("error", "")}\n')
            f.write('\n')
        
        return txt_path
    
    def save_measurements_csv(self, measurement_records: List[Dict], result: dict, 
                             sequence_id: str = '') -> str:
        """
        프레임별 측정값과 평균을 CSV로 저장
        
        Args:
            measurement_records: 프레임별 측정 레코드 리스트
            result: 평균 계산 결과 딕셔너리
            sequence_id: 시퀀스 ID
        
        Returns:
            저장된 파일 경로
        """
        base_dir = self._get_base_dir(sequence_id)
        results_dir = os.path.join(base_dir, 'results')
        os.makedirs(results_dir, exist_ok=True)
        
        csv_path = os.path.join(results_dir, 'measurements.csv')
        
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            
            # 헤더
            writer.writerow(['Frame', 'P1-P2 (m)', 'P3-P4 (m)', 'P5-P6 (m)', 'P7-P8 (m)'])
            
            # 프레임별 데이터
            for record in measurement_records:
                writer.writerow([
                    record.get('frame', 0),
                    f"{record.get('P1-P2', 0.0):.6f}",
                    f"{record.get('P3-P4', 0.0):.6f}",
                    f"{record.get('P5-P6', 0.0):.6f}",
                    f"{record.get('P7-P8', 0.0):.6f}"
                ])
            
            # 빈 줄
            writer.writerow([])
            
            # 평균값 (첫 프레임 제외)
            writer.writerow(['Average (excluding frame 0)', '', '', '', ''])
            writer.writerow([
                'P1-P2 (m)', 'P3-P4 (m)', 'P5-P6 (m)', 'P7-P8 (m)'
            ])
            writer.writerow([
                f"{result.get('p1_p2', 0.0):.6f}",
                f"{result.get('p3_p4', 0.0):.6f}",
                f"{result.get('p5_p6', 0.0):.6f}",
                f"{result.get('p7_p8', 0.0):.6f}"
            ])
            
            # 빈 줄
            writer.writerow([])
            
            # 최종 결과
            writer.writerow(['Final Results', '', '', '', ''])
            writer.writerow(['Length (mm)', 'Width (mm)'])
            writer.writerow([result.get('length', 0), result.get('width', 0)])
        
        return csv_path
    
    def save_all_measurement_data(self, acwl_request: dict, wlac_request: dict, 
                                  wlac_response: dict, measurement_records: List[Dict],
                                  result: dict, sequence_id: str = '') -> Dict[str, str]:
        """
        모든 측정 데이터를 한 번에 저장
        
        Args:
            acwl_request: ACWL 요청 데이터
            wlac_request: WLAC 요청 데이터
            wlac_response: WLAC 응답 데이터
            measurement_records: 프레임별 측정 레코드
            result: 평균 계산 결과
            sequence_id: 시퀀스 ID
        
        Returns:
            저장된 파일 경로들의 딕셔너리
        """
        paths = {}
        
        # ACWL 요청 저장
        paths['acwl_txt'] = self.save_acwl_request(acwl_request, sequence_id)
        
        # 측정 데이터 CSV 저장
        paths['measurements_csv'] = self.save_measurements_csv(
            measurement_records, result, sequence_id
        )
        
        # WLAC 응답 저장
        paths['wlac_txt'] = self.save_wlac_response(
            wlac_request, wlac_response, sequence_id
        )
        
        return paths
    
    def save_visualizations(self, image, masks, magnet_mask, plate_mask,
                           magnet_corners_2d, plate_corners_2d,
                           measure_points_3d, measurements, camera_matrix, dist_coeffs,
                           frame_idx: int, sequence_id: str = ''):
        """
        4종 시각화 결과 저장
        
        Args:
            image: 원본 이미지 (RGB)
            masks: SAM2 마스크 딕셔너리
            magnet_mask: 정제된 자석 마스크
            plate_mask: 정제된 판재 마스크
            magnet_corners_2d: 자석 코너 2D 좌표
            plate_corners_2d: 판재 코너 2D 좌표
            measure_points_3d: 측정 포인트 3D 좌표
            measurements: 측정 결과 딕셔너리
            camera_matrix: 카메라 매트릭스
            dist_coeffs: 왜곡 계수
            frame_idx: 프레임 인덱스
            sequence_id: 시퀀스 ID
        """
        from visualizer import (
            sam2_visualizer, mask_visualizer,
            box_visualizer, measurement_visualizer
        )
        
        # 출력 디렉토리 설정
        if sequence_id:
            output_dir = Path(self.output_dir) / sequence_id / "results"
        else:
            output_dir = Path(self.output_dir)
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
            measure_points_3d, camera_matrix, dist_coeffs,
            measurements, frame_id, output_dir
        )
    
    def save_sensor_data(self, sensor_data_list: list, sequence_id: str = ''):
        """
        센서 데이터를 디스크에 저장 (이미지 + 포인트클라우드)
        
        Args:
            sensor_data_list: 센서 데이터 리스트 (각 요소는 'camera', 'lidar' 키를 가진 dict)
            sequence_id: 시퀀스 ID
        """
        base_dir = self._get_base_dir(sequence_id)
        image_dir = os.path.join(base_dir, 'image')
        pcd_dir = os.path.join(base_dir, 'pcd')
        
        os.makedirs(image_dir, exist_ok=True)
        os.makedirs(pcd_dir, exist_ok=True)
        
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
                # 개별 프레임 저장 실패는 무시하고 계속 진행
                pass
    
    def _save_pcd(self, points: np.ndarray, filepath: str):
        """
        포인트 클라우드를 PCD 파일로 저장 (ASCII 형식)
        
        Args:
            points: Nx3 numpy 배열
            filepath: 저장할 PCD 파일 경로
        """
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
    
    def _get_base_dir(self, sequence_id: str = '') -> str:
        """
        시퀀스 ID에 따른 기본 디렉토리 경로 생성
        
        Args:
            sequence_id: 시퀀스 ID
        
        Returns:
            디렉토리 경로
        """
        if sequence_id:
            return os.path.join(self.output_dir, sequence_id)
        else:
            return self.output_dir


def convert_ros_request_to_dict(ros_request) -> dict:
    """
    ROS 요청 메시지를 딕셔너리로 변환 (ACWL0001) - 모든 필드 포함
    
    Args:
        ros_request: ROS ACWL0001.Request 객체
    
    Returns:
        딕셔너리로 변환된 데이터
    """
    return {
        # Header 필드
        'head': {
            'msg_id': ros_request.head.msg_id,
            'date': ros_request.head.date,
            'time': ros_request.head.time,
            'form': ros_request.head.form,
            'msg_len': ros_request.head.msg_len,
            'filler': ros_request.head.filler
        },
        # Body 필드
        'body': {
            'eqp_cd': ros_request.body.eqp_cd,
            'dx_direct': ros_request.body.dx_direct,
            'dy_direct': ros_request.body.dy_direct,
            'dz_mm': ros_request.body.dz_mm,
            'cr_op_indi_id': ros_request.body.cr_op_indi_id,
            'plate_top_len': ros_request.body.plate_top_len,
            'plate_max_width': ros_request.body.plate_max_width,
            'lidar_use': ros_request.body.lidar_use,
            'camera_use': ros_request.body.camera_use,
            'raw': ros_request.body.raw,
            'stamp': {
                'sec': ros_request.body.stamp.sec,
                'nanosec': ros_request.body.stamp.nanosec
            }
        }
    }


def convert_wlac_request_to_dict(ros_wlac_request) -> dict:
    """
    ROS WLAC 요청 메시지를 딕셔너리로 변환 - 모든 필드 포함
    
    Args:
        ros_wlac_request: ROS WLAC0001.Request 객체
    
    Returns:
        딕셔너리로 변환된 데이터
    """
    return {
        # Header 필드
        'head': {
            'msg_id': ros_wlac_request.head.msg_id,
            'date': ros_wlac_request.head.date,
            'time': ros_wlac_request.head.time,
            'form': ros_wlac_request.head.form,
            'msg_len': ros_wlac_request.head.msg_len,
            'filler': ros_wlac_request.head.filler
        },
        # Body 필드
        'body': {
            'eqp_cd': ros_wlac_request.body.eqp_cd,
            'result_code': ros_wlac_request.body.result_code,
            'len_result_p1_2': ros_wlac_request.body.len_result_p1_2,
            'len_result_p3_4': ros_wlac_request.body.len_result_p3_4,
            'width_result_p5_6': ros_wlac_request.body.width_result_p5_6,
            'width_result_p7_8': ros_wlac_request.body.width_result_p7_8,
            'stamp': {
                'sec': ros_wlac_request.body.stamp.sec,
                'nanosec': ros_wlac_request.body.stamp.nanosec
            }
        }
    }


def convert_wlac_response_to_dict(ros_wlac_response) -> dict:
    """
    ROS WLAC 응답 메시지를 딕셔너리로 변환
    
    Args:
        ros_wlac_response: ROS WLAC0001.Response 객체
    
    Returns:
        딕셔너리로 변환된 데이터
    """
    return {
        'stored': ros_wlac_response.stored,
        'error': ros_wlac_response.error
    }
