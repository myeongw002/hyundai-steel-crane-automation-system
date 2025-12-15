import os
import sys
import traceback
import argparse
import numpy as np
import pandas as pd
from data_loader import DataLoader
from sam2_wrapper import SAM2Wrapper
from utils import *
from visualizer import *
import shutil

class Config:
    # --- System & Paths ---
    VIDEO_DIR = "../hyundai-steel/data/202511270905601478/image/"
    PCD_DIR = "../hyundai-steel/data/202511270905601478/pcd/"
    
    ONE_SHOT_IMAGE = "../hyundai-steel/data/202511271034602746/image/0002.jpg"
    magnet_pts = np.array([[1010, 460], [1010, 714]], dtype=np.float32)
    magnet_lbl = np.array([1, 1], np.int32)
    plate_pts = np.array([[837, 424], [837, 766], [260, 590], [1118, 574], [1170, 804], [1119,327], [607, 892]], dtype=np.float32)
    plate_lbl = np.array([1, 1, 0, 0, 0, 0, 0], np.int32)
    
    MEASUREMENT_OFFSET = { # 크레인 높이 값에 맞춰 변경시켜줘야 됨
        'P1-P2': 19.4,   # 위쪽 길이 offset
        'P3-P4': 19.4,   # 아래쪽 길이 offset
        'P5-P6': 32.4,   # 위쪽 너비 offset
        'P7-P8': 32.4    # 아래쪽 너비 offset
    }
    
    SAM2_ROOT = "sam2"
    if SAM2_ROOT not in sys.path:
        sys.path.insert(0, SAM2_ROOT)
    SAM2_CHECKPOINT = "checkpoints/sam2.1_hiera_large.pt"
    SAM2_CONFIG = "configs/sam2.1/sam2.1_hiera_l.yaml"

    CAMERA_MATRIX = np.array([
        [1385.635672,    -0.895992,   947.154907],
        [   0.  ,     1388.287107,  607.421636],
        [   0.  ,        0.      ,    1.      ]
    ])
    DIST_COEFFS = np.array([-0.085307, 0.079377, 0.000436, -0.000662, 0.0])
    T_LIDAR_TO_CAM = np.array([
        [0.0129229, -0.999916, -0.000756324, -0.121755],
        [-0.0144186, 0.000569792, -0.999896, 0.010138],
        [0.999813, 0.0129326, -0.0144102, -0.148907],
        [0,0,0,1]
    ])

    MORPH_OPEN_KERNEL_SIZE = 40
    MEASUREMENT_POSITION_OFFSET = 700
    
    LIDAR_DENSE = False
    
    VISUALIZE = True
    OUTPUT_DIR = "outputs"

class Pipeline:
    def __init__(self):
        self.cfg = Config
        self.sam2 = SAM2Wrapper(self.cfg)
        target_file = self.cfg.VIDEO_DIR + "0000.jpg"
        shutil.copy2(self.cfg.ONE_SHOT_IMAGE, target_file)
        
    def run(self):
        # Prepare Data
        frame_names, pcd_names = DataLoader.load_data_names(self.cfg.VIDEO_DIR, self.cfg.PCD_DIR)
        if not frame_names:
            print("No frames found."); return
        
        # SAM2 Inference
        prompts = [
            {'id': 1, 'points': self.cfg.magnet_pts, 'labels': self.cfg.magnet_lbl},
            {'id': 2, 'points': self.cfg.plate_pts, 'labels': self.cfg.plate_lbl}
        ]
        masks_all = self.sam2.video_run(self.cfg.VIDEO_DIR, prompts)
        measurements = self.process_frame(frame_names[2], pcd_names[2], masks_all.get(2))
        
        # Save CSV
        self.save_csv(measurements)
                
    def process_frame(self, img_path, pcd_path, masks):
        # 이미지 로드 및 왜곡 보정
        image, camera_matrix = image_load_and_undistort(
            img_path, self.cfg.CAMERA_MATRIX, self.cfg.DIST_COEFFS
        )
        if image is None:
            return
        
        # PCD 로드
        pcd = o3d.io.read_point_cloud(pcd_path)
        pcd = crop_fov(pcd, fov_degrees=120)
        points_lidar = np.asarray(pcd.points)
        
        # mask 정제
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.cfg.MORPH_OPEN_KERNEL_SIZE,self.cfg.MORPH_OPEN_KERNEL_SIZE))
        magnet_mask = refine_mask(masks.get(1), kernel)
        plate_mask = refine_mask(masks.get(2), kernel)
        
        # plate mask 내부 포인트 추출
        plate_points, plate_points_cam, plate_indices = filter_points_in_mask(points_lidar, plate_mask, self.cfg.T_LIDAR_TO_CAM, camera_matrix)
        
        if len(plate_points_cam) < 3:
            print("오류: 철판 영역에 포인트가 부족합니다.")
            return
        
        # plate 평면 추정
        plane_normal, plane_d, plane_centroid, _, _ = estimate_plane_ransac(plate_points_cam, iterations=100, threshold=0.05)
        if plane_normal is None:
            print("오류: 평면 추정 실패")
            return
        
        print(f"   평면: {plane_normal[0]:.4f}x + {plane_normal[1]:.4f}y + {plane_normal[2]:.4f}z + {plane_d:.4f} = 0")
        
        # 평면으로 부터 노이즈 제거
        plate_mask, no_plate_mask = refine_mask_with_plane(
            points_lidar, 
            plate_mask, 
            plate_indices,
            plane_normal, 
            plane_d, 
            self.cfg.T_LIDAR_TO_CAM, 
            camera_matrix,
            threshold=0.05
        )
        
        if not self.cfg.LIDAR_DENSE:
            # plate mask로 부터 박스 생성(mask 정제용)
            plate_corners_2d = mask_to_box(plate_mask, rotate=False, inscribed=False)
            
            min_y = 100
            max_y = image.shape[0] - 100
            min_x = 100
            max_x = image.shape[1]  - 100
            x_coords = np.arange(min_x, max_x + 1, 200)
            points_min_y = np.column_stack([x_coords, np.full_like(x_coords, min_y)])
            points_max_y = np.column_stack([x_coords, np.full_like(x_coords, max_y)])
            outlier_points = np.vstack([points_min_y, points_max_y])
            
            plate_mask = self.sam2.image_run(image, plate_corners_2d, outlier_points)
            
            plate_mask = refine_mask(plate_mask, kernel)
        
        # plate mask로 부터 박스 생성
        plate_corners_2d = mask_to_box(plate_mask, rotate=True, inscribed=False)
        
        # plate 코너를 3D 평면으로 매핑
        plate_corners_3d = unproject_to_plane(plate_corners_2d, plane_normal, plane_centroid, camera_matrix, self.cfg.DIST_COEFFS)

        # magnet mask로 부터 박스 생성
        magnet_corners_2d = mask_to_box(magnet_mask, rotate=False, inscribed=True)
        
        # magnet 코너를 3D 평면으로 매핑
        magnet_corners_3d = unproject_to_plane(magnet_corners_2d, plane_normal, plane_centroid, camera_matrix, self.cfg.DIST_COEFFS)
        
        magnet_points, magnet_points_cam, _ = filter_points_in_mask(points_lidar, magnet_mask, self.cfg.T_LIDAR_TO_CAM, camera_matrix)
        
        print("\n 거리 측정")
        calculator = DistanceCalculator(
            magnet_corners_2d,
            magnet_corners_3d,
            plate_corners_3d,
            plane_normal,
            plane_centroid,
            camera_matrix,
            self.cfg.DIST_COEFFS,
            self.cfg.MEASUREMENT_POSITION_OFFSET,
            self.cfg.MEASUREMENT_OFFSET
        )
        
        measurements, measure_points_3d = calculator.calculate()
        if measurements is None:
            print("오류: 거리 계산 실패")
            return
        
        if self.cfg.VISUALIZE:
            self._visualize_and_save(
                image,
                masks,
                magnet_mask,
                plate_mask,
                magnet_corners_2d,
                plate_corners_2d,
                measure_points_3d,
                measurements
                )
        return measurements
            
    def save_csv(self, measurements):
        output_path = Path(self.cfg.OUTPUT_DIR)
        output_path.mkdir(exist_ok=True, parents=True)
        
        parts = self.cfg.VIDEO_DIR.strip('/').split('/')
        session_id = parts[-2] if len(parts) > 1 else 'unknown'
        
        pairs = [('P1', 'P2'), ('P3', 'P4'), ('P5', 'P6'), ('P7', 'P8')]
        data = {
            'session_id': session_id,
        }
        for start, end in pairs:
            pair_name = f"{start}-{end}"
            dist = measurements.get(pair_name, 0.0)
            data[pair_name] = round(dist, 2)
        
        df = pd.DataFrame([data])
        filename = output_path / f'{session_id}_measurements.csv'
        df.to_csv(filename, mode='w', header=True, index=False)
        print(f'Saved to {filename}')
        
    def _visualize_and_save(self, image, masks, magnet_mask, plate_mask, magnet_corners_2d, plate_corners_2d, measure_points_3d, measurements):
        """결과 시각화 및 저장"""
        output_path = Path(self.cfg.OUTPUT_DIR)
        output_path.mkdir(exist_ok=True, parents=True)
        
        parts = self.cfg.VIDEO_DIR.strip('/').split('/')
        session_id = parts[-2] if len(parts) > 1 else 'unknown'
        
        # segment 결과
        sam2_visualizer(image, masks, session_id, output_path=output_path)
        
        # mask 정제 결과
        mask_visualizer(image, [magnet_mask, plate_mask], session_id, output_path=output_path)
        
        # 박스 결과
        box_visualizer(image, [magnet_corners_2d, plate_corners_2d], session_id, output_path=output_path)
        
        # 측정 결과
        measurement_visualizer(
            image,
            [magnet_corners_2d, plate_corners_2d],
            measure_points_3d,
            self.cfg.CAMERA_MATRIX,
            self.cfg.DIST_COEFFS,
            measurements,
            session_id,
            output_path=output_path
        )

def parse_args():
    parser = argparse.ArgumentParser(description='Run measurement pipeline with configurable parameters')
    
    # 데이터 경로
    parser.add_argument('--video-dir', type=str, default=None, help='Video directory path')
    parser.add_argument('--pcd-dir', type=str, default=None, help='Point cloud directory path')
    parser.add_argument('--plate-dir', type=str, default=None, help='Plate results directory path')
    
    # 파라미터
    parser.add_argument('--kernel-size', type=int, default=None, help='Morphological opening kernel size')
    parser.add_argument('--offset', type=int, default=None, help='Measurement offset in mm')
    
    # 출력
    parser.add_argument('--output-dir', type=str, default=None, help='Output directory path')
    parser.add_argument('--visualize', action='store_true', help='Enable visualization')
    parser.add_argument('--exp-name', type=str, default=None, help='Experiment name for output directory')
    
    return parser.parse_args()

def apply_args_to_config(args):
    """Apply command-line arguments to Config class"""
    if args.video_dir:
        Config.VIDEO_DIR = args.video_dir
    if args.pcd_dir:
        Config.PCD_DIR = args.pcd_dir
    if args.plate_dir:
        Config.PLATE_DIR = args.plate_dir
    if args.kernel_size:
        Config.MORPH_OPEN_KERNEL_SIZE = args.kernel_size
    if args.offset:
        Config.MEASUREMENT_POSITION_OFFSET = args.offset
    if args.visualize:
        Config.VISUALIZE = True
    if args.output_dir:
        Config.OUTPUT_DIR = args.output_dir
    elif args.exp_name:
        Config.OUTPUT_DIR = f"outputs/{args.exp_name}"

if __name__ == "__main__":
    args = parse_args()
    apply_args_to_config(args)
    pipeline = Pipeline()
    pipeline.run()
