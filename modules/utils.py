import open3d as o3d
import numpy as np
import cv2
from typing import Tuple, Optional, Dict

def image_load_and_undistort(image_path, camera_matrix, dist_coeffs):
    image = cv2.imread(image_path)
    if image is None:
        print(f"오류: 이미지를 읽을 수 없습니다: {image_path}")
        return None, None
    
    h, w = image.shape[:2]
    new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )
    undistorted_img = cv2.undistort(image, camera_matrix, dist_coeffs, None, new_camera_matrix)
    
    return undistorted_img, new_camera_matrix

def points_to_depth_map(pcd, H, W, fov_up=22.5, fov_down=-22.5, fov_left=-60.0, fov_right=60.0):
    """
    3D Points (N, 3) -> 2D Depth Map (H, W)
    """
    points = np.asarray(pcd.points)
    depth_map = np.zeros((H, W), dtype=np.float32)
    
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    r = np.linalg.norm(points, axis=1)
    
    valid_indices = r > 0
    x, y, z, r = x[valid_indices], y[valid_indices], z[valid_indices], r[valid_indices]
    
    yaw = -np.arctan2(y, x)
    pitch = np.arcsin(z / r)
    
    fov_left_rad = np.deg2rad(fov_left)
    fov_right_rad = np.deg2rad(fov_right)
    fov_width = fov_right_rad - fov_left_rad
    
    fov_up_rad = np.deg2rad(fov_up)
    fov_down_rad = np.deg2rad(fov_down)
    fov_height = fov_up_rad - fov_down_rad
    
    u = ((yaw - fov_left_rad) / fov_width) * W
    v = (1.0 - (pitch - fov_down_rad) / fov_height) * H
    
    u = np.floor(u).astype(np.int32)
    v = np.floor(v).astype(np.int32)
    
    mask = (u >= 0) & (u < W) & (v >= 0) & (v < H)
    u, v, r = u[mask], v[mask], r[mask]
    
    order = np.argsort(r)[::-1]
    u, v, r = u[order], v[order], r[order]
    
    depth_map[v, u] = r
    return depth_map

def depth_map_to_points(depth_map, fov_up=22.5, fov_down=-22.5, fov_left=-60.0, fov_right=60.0):
    """
    2D Depth Map -> 3D Points (N, 3) 복원 (Back-projection)
    """
    H, W = depth_map.shape
    
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    u = u.flatten()
    v = v.flatten()
    r = depth_map.flatten()
    
    valid = r > 0
    u, v, r = u[valid], v[valid], r[valid]
    
    fov_left_rad = np.deg2rad(fov_left)
    fov_right_rad = np.deg2rad(fov_right)
    fov_width = fov_right_rad - fov_left_rad
    
    yaw = (u / W) * fov_width + fov_left_rad
    
    fov_up_rad = np.deg2rad(fov_up)
    fov_down_rad = np.deg2rad(fov_down)
    fov_height = fov_up_rad - fov_down_rad
    
    pitch = (1.0 - v / H) * fov_height + fov_down_rad
    
    x = r * np.cos(pitch) * np.cos(-yaw)
    y = r * np.cos(pitch) * np.sin(-yaw)
    z = r * np.sin(pitch)
    
    points = np.stack((x, y, z), axis=-1)
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def crop_fov(pcd, fov_degrees=120):
    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None
    
    x, y = points[:, 0], points[:, 1]
    angles_deg = np.degrees(np.arctan2(y, x))
    
    half_fov = fov_degrees / 2.0
    mask = (np.abs(angles_deg) <= half_fov) & (x > 0)
    
    cropped_pcd = o3d.geometry.PointCloud()
    cropped_pcd.points = o3d.utility.Vector3dVector(points[mask])
    
    if colors is not None:
        cropped_pcd.colors = o3d.utility.Vector3dVector(colors[mask])
        
    return cropped_pcd

def project_lidar_to_camera(points_3d, extrinsic, camera_matrix):
    R = extrinsic[:3, :3]
    t = extrinsic[:3, 3]
    
    points_cam = (R @ points_3d.T + t[:, np.newaxis]).T
    valid_mask = points_cam[:, 2] > 0.1
    
    rvec, _ = cv2.Rodrigues(R)
    pixels_2d, _ = cv2.projectPoints(points_3d, rvec, t, camera_matrix, None)
    pixels = pixels_2d.reshape(-1, 2)
    
    return pixels, valid_mask, points_cam

def refine_mask(mask, kernel, mode="open", iterations=1):
    mask = mask.astype(np.uint8) * 255
    for _ in range(iterations):
        if mode == "open":
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=kernel)
        else:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel=kernel)
    return mask > 0

def filter_points_in_mask(points_lidar, mask, extrinsic, camera_matrix):
    pixels, valid_mask, points_cam = project_lidar_to_camera(
        points_lidar, extrinsic, camera_matrix
    )
    h, w = mask.shape
    
    in_image = (
        (pixels[:, 0] >= 0) & (pixels[:, 0] < w) &
        (pixels[:, 1] >= 0) & (pixels[:, 1] < h) &
        valid_mask
    )
    
    pixels_int = pixels.astype(int)
    
    roi_mask = np.zeros(len(points_lidar), dtype=bool)
    roi_mask[in_image] = mask[pixels_int[in_image, 1], pixels_int[in_image, 0]]
    
    roi_indices = np.where(roi_mask)[0]
    roi_points = points_lidar[roi_indices]
    roi_points_cam = points_cam[roi_indices]
    
    return roi_points, roi_points_cam, roi_indices

def estimate_plane_ransac(points, iterations=100, threshold=None, percentile_threshold=70):
    best_inliers = []
    best_normal = None
    best_d = None
    
    if len(points) < 3:
        return None, None, None, np.array([]), np.array([])
    
    for i in range(iterations):
        sample_indices = np.random.choice(len(points), 3, replace=False)
        sample_points = points[sample_indices]
        
        try:
            centroid = np.mean(sample_points, axis=0)
            _, _, vh = np.linalg.svd(sample_points - centroid)
            normal = vh[2, :]
            d = -np.dot(normal, centroid)
        except np.linalg.LinAlgError:
            continue
        
        distances = np.abs(np.dot(points, normal) + d)
        
        if threshold is None:
            current_threshold = np.percentile(distances, percentile_threshold)
        else:
            current_threshold = threshold
        
        inlier_indices = np.where(distances < current_threshold)[0]
        
        if len(inlier_indices) > len(best_inliers):
            best_inliers = inlier_indices
            best_normal = normal
            best_d = d
    
    inlier_points = points[best_inliers]
    best_centroid = np.mean(inlier_points, axis=0) if len(inlier_points) > 0 else np.mean(points, axis=0)
    
    return best_normal, best_d, best_centroid, inlier_points, best_inliers

def refine_mask_with_plane(points_lidar, original_mask, roi_indices, 
                          plane_normal, plane_d, extrinsic, camera_matrix, 
                          threshold=0.05):
    if plane_normal is None or plane_d is None:
        print("Warning: 평면 파라미터가 없습니다. 원본 마스크 반환.")
        return original_mask, np.zeros_like(original_mask, dtype=bool)
    
    pixels, valid_mask, points_cam = project_lidar_to_camera(
        points_lidar, extrinsic, camera_matrix
    )
    
    roi_points_cam = points_cam[roi_indices]
    distances = np.abs(np.dot(roi_points_cam, plane_normal) + plane_d)
    inlier_mask = distances < threshold
    
    inlier_lidar_indices = roi_indices[inlier_mask]
    outlier_lidar_indices = roi_indices[~inlier_mask]
    
    refined_mask = np.zeros_like(original_mask, dtype=bool)
    outlier_mask = np.zeros_like(original_mask, dtype=bool)
    pixels_int = pixels.astype(int)
    
    h, w = original_mask.shape
    
    for idx in inlier_lidar_indices:
        if valid_mask[idx]:
            px, py = pixels_int[idx]
            if 0 <= px < w and 0 <= py < h:
                refined_mask[py, px] = True
    
    for idx in outlier_lidar_indices:
        if valid_mask[idx]:
            px, py = pixels_int[idx]
            if 0 <= px < w and 0 <= py < h:
                outlier_mask[py, px] = True
    
    return refined_mask, outlier_mask

def _order_corners(pts: np.ndarray) -> np.ndarray:
    rect = np.zeros((4, 2), dtype=np.float32)
    
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    
    diff = np.diff(pts, axis=1)
    rect[3] = pts[np.argmin(diff)]
    rect[1] = pts[np.argmax(diff)]
    
    return rect

def _get_largest_inscribed_rect(binary_mask):
    rows, cols = binary_mask.shape
    heights = np.zeros(cols, dtype=np.int32)
    max_area = 0
    best_rect = (0, 0, 0, 0)

    for r in range(rows):
        heights = np.where(binary_mask[r] == 1, heights + 1, 0)
        
        stack = [] 
        extended_heights = np.append(heights, 0)
        
        for i, h in enumerate(extended_heights):
            start_index = i
            while stack and stack[-1][1] > h:
                index, height = stack.pop()
                width = i - index
                area = width * height
                
                if area > max_area:
                    max_area = area
                    best_rect = (index, r - height + 1, width, height)
                    
                start_index = index
            stack.append((start_index, h))
            
    return best_rect

def mask_to_box(mask_bool, rotate=True, inscribed=False):
    ys, xs = np.where(mask_bool)
    
    if len(xs) == 0:
        return np.zeros((4, 2), dtype=np.float32)

    if not rotate:
        if inscribed:
            pts = np.stack([xs, ys], axis=1).astype(np.int32)
            
            hull = cv2.convexHull(pts)
            
            filled_mask = np.zeros(mask_bool.shape, dtype=np.uint8)
            cv2.drawContours(filled_mask, [hull], -1, 1, thickness=-1)
            
            x, y, w, h = _get_largest_inscribed_rect(filled_mask)
            
            if w == 0 or h == 0:
                return np.zeros((4, 2), dtype=np.float32)

            xmin, ymin = x, y
            xmax = x + w - 1 if w > 0 else x
            ymax = y + h - 1 if h > 0 else y
            
            corners = np.array([[xmin, ymin], [xmin, ymax], [xmax, ymax], [xmax, ymin]], dtype=np.float32)
            return corners
            
        else:
            xmin, xmax = xs.min(), xs.max()
            ymin, ymax = ys.min(), ys.max()
            corners = np.array([[xmin, ymin], [xmin, ymax], [xmax, ymax], [xmax, ymin]], dtype=np.float32)
            return corners
        
    pts = np.stack([xs, ys], axis=1).astype(np.float32)
    rect = cv2.minAreaRect(pts)
    corners = cv2.boxPoints(rect)
    corners = _order_corners(corners)
    return corners

def unproject_to_plane(
    corners_2d: np.ndarray,
    plane_normal: np.ndarray,
    plane_centroid: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray
):
    pts_reshaped = corners_2d.reshape(-1, 1, 2).astype(np.float32)
    xy_undist = cv2.undistortPoints(
        pts_reshaped, 
        camera_matrix, 
        None
    ).squeeze()
    
    if xy_undist.ndim == 1:
        xy_undist = xy_undist.reshape(1, -1)
    
    n = plane_normal / (np.linalg.norm(plane_normal) + 1e-12)
    p0 = plane_centroid.astype(np.float32)
    
    points_3d = []
    for x_n, y_n in xy_undist:
        ray_dir = np.array([x_n, y_n, 1.0], dtype=np.float32)
        ray_dir = ray_dir / np.linalg.norm(ray_dir)
        
        denom = np.dot(n, ray_dir)
        if abs(denom) < 1e-6:
            continue
        
        t = np.dot(n, p0) / denom
        if t <= 0:
            continue
        
        intersection = ray_dir * t
        points_3d.append(intersection)
    
    return np.array(points_3d) if points_3d else None

    
class DistanceCalculator:
    """
    3D 평면상에서 두 객체 간의 거리를 측정하는 클래스
    - obj1 (마그넷): 측정 시작점이 되는 기준 객체
    - obj2 (철판): 측정 대상 객체
    - 평면 상에서 4개의 측정선(P1-P2, P3-P4, P5-P6, P7-P8)을 계산
    """
    
    # 측정 설정 상수
    MEASUREMENT_PAIRS = ['P1-P2', 'P3-P4', 'P5-P6', 'P7-P8']
    EPSILON = 1e-6  # 수치 안정성을 위한 작은 값
    
    def __init__(
        self,
        obj1_corners_2d: np.ndarray,
        obj1_corners_3d: np.ndarray,
        obj2_corners_3d: np.ndarray,
        plane_normal: np.ndarray,
        plane_centroid: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        offset_mm: float,
        measurement_offsets: Optional[Dict[str, float]] = None
    ):
        """
        Args:
            obj1_corners_2d: obj1의 2D 코너 좌표 (4x2) [tl, bl, br, tr]
            obj1_corners_3d: obj1의 3D 코너 좌표 (4x3) [tl, bl, br, tr]
            obj2_corners_3d: obj2의 3D 코너 좌표 (4x3)
            plane_normal: 평면의 법선 벡터
            plane_centroid: 평면의 중심점
            camera_matrix: 카메라 내부 파라미터 행렬
            dist_coeffs: 렌즈 왜곡 계수
            offset_mm: obj1 엣지에서의 오프셋 (mm)
            measurement_offsets: 각 측정선별 보정 오프셋 (mm)
        """
        self.obj1_corners_2d = obj1_corners_2d
        self.obj1_corners_3d = obj1_corners_3d
        self.obj2_corners_3d = obj2_corners_3d
        self.plane_normal = self._normalize_vector(plane_normal)
        self.plane_centroid = plane_centroid
        self.camera_matrix = camera_matrix
        self.dist_coeffs = None  # 왜곡 보정 사용 안 함
        self.offset_mm = offset_mm
        self.offset_m = offset_mm / 1000.0  # 미터 단위로 변환
        
        # 측정 오프셋 초기화
        self.measurement_offsets = measurement_offsets or {
            pair: 0.0 for pair in self.MEASUREMENT_PAIRS
        }
    
    # ==================== 메인 계산 메서드 ====================
    
    def calculate(self) -> Tuple[Optional[Dict[str, float]], Optional[Dict[str, np.ndarray]]]:
        """
        모든 측정선에 대한 거리를 계산
        
        Returns:
            measurements: 각 측정선별 거리 (mm)
            points_dict: 측정에 사용된 3D 포인트들
        """
        # 1. 측정 시작점 생성 (2D -> 3D)
        measure_points_3d = self._generate_and_unproject_points()
        if measure_points_3d is None:
            print("측정점 생성 실패")
            return None, None
        
        P1, P3, P5, P7 = measure_points_3d
        
        # 2. 방향 벡터 및 기준점 계산
        directions = self._calculate_directions()
        reference_points = self._calculate_reference_points()
        
        # 3. 각 측정선별 거리 계산
        measurements = {}
        points_dict = {}
        
        configs = self._get_measurement_configs(
            [P1, P3, P5, P7], directions, reference_points
        )
        
        for config in configs:
            distance, start_pt, end_pt = self._calculate_single_measurement(config)
            
            name = config['name']
            measurements[name] = distance
            points_dict[config['start_label']] = start_pt
            points_dict[config['end_label']] = end_pt
        
        return measurements, points_dict
    
    def _calculate_single_measurement(self, config: dict) -> Tuple[float, np.ndarray, Optional[np.ndarray]]:
        """단일 측정선에 대한 거리 계산"""
        name = config['name']
        start_pt = config['start_pt']
        direction = config['direction']
        ref_pt = config['ref_pt']
        desc = config['desc']
        
        # obj2와의 교차점 찾기
        end_pt = self._find_intersection_with_obj2(start_pt, direction)
        
        if end_pt is None:
            print(f"{name} 교차점 없음")
            return 0.0, start_pt, None
        
        # 부호 있는 상대 거리 계산
        distance = self._compute_signed_relative_distance(
            start_pt, end_pt, ref_pt, direction
        )
        
        # 보정 오프셋 적용
        offset = self.measurement_offsets.get(name, 0.0)
        corrected_distance = distance + offset
        
        print(f"{name} ({desc}): {distance:.1f}mm + {offset:.1f}mm = {corrected_distance:.1f}mm")
        
        return corrected_distance, start_pt, end_pt
    
    # ==================== 측정 설정 생성 ====================
    
    def _get_measurement_configs(
        self,
        start_points: list,
        directions: dict,
        reference_points: dict
    ) -> list:
        """측정 설정 리스트 생성"""
        P1, P3, P5, P7 = start_points
        
        return [
            {
                'name': 'P1-P2',
                'start_pt': P1,
                'direction': directions['P1'],
                'ref_pt': reference_points['P1_ref'],
                'start_label': 'P1',
                'end_label': 'P2',
                'desc': '위쪽 길이'
            },
            {
                'name': 'P3-P4',
                'start_pt': P3,
                'direction': directions['P3'],
                'ref_pt': reference_points['P3_ref'],
                'start_label': 'P3',
                'end_label': 'P4',
                'desc': '아래쪽 길이'
            },
            {
                'name': 'P5-P6',
                'start_pt': P5,
                'direction': directions['P5'],
                'ref_pt': reference_points['P5_ref'],
                'start_label': 'P5',
                'end_label': 'P6',
                'desc': '위쪽 너비'
            },
            {
                'name': 'P7-P8',
                'start_pt': P7,
                'direction': directions['P7'],
                'ref_pt': reference_points['P7_ref'],
                'start_label': 'P7',
                'end_label': 'P8',
                'desc': '아래쪽 너비'
            }
        ]
    
    # ==================== 포인트 생성 ====================
    
    def _generate_and_unproject_points(self) -> Optional[np.ndarray]:
        """2D 측정점 생성 후 3D로 언프로젝션"""
        measure_points_2d = self._generate_measurement_points_2d()
        measure_points_3d = self._unproject_to_plane(measure_points_2d)
        
        if measure_points_3d is None or len(measure_points_3d) < 4:
            return None
        
        return measure_points_3d
    
    def _generate_measurement_points_2d(self) -> np.ndarray:
        """obj1의 왼쪽 엣지에서 측정 시작점 4개 생성 (2D)"""
        tl, bl, br, tr = self.obj1_corners_2d
        
        # 왼쪽 엣지의 길이
        left_edge_length_3d = np.linalg.norm(
            self.obj1_corners_3d[1] - self.obj1_corners_3d[0]
        )
        
        # 오프셋 비율 계산
        t1 = self.offset_m / left_edge_length_3d
        t2 = 1.0 - t1
        
        # 4개의 측정 시작점
        p1 = tl + t1 * (bl - tl)  # 위쪽에서 offset만큼 아래
        p3 = tl + t2 * (bl - tl)  # 아래쪽에서 offset만큼 위
        p5 = tl                    # 왼쪽 위 코너
        p7 = bl                    # 왼쪽 아래 코너
        
        return np.array([p1, p3, p5, p7])
    
    def _calculate_reference_points(self) -> Dict[str, np.ndarray]:
        """obj1의 기준 엣지 포인트 계산 (측정 종료 기준점)"""
        tl_3d, bl_3d, br_3d, tr_3d = self.obj1_corners_3d
        
        left_edge_length = np.linalg.norm(bl_3d - tl_3d)
        t1 = self.offset_m / left_edge_length
        t2 = 1.0 - t1
        
        return {
            # 가로 측정: obj1의 오른쪽 엣지가 기준
            'P1_ref': tr_3d + t1 * (br_3d - tr_3d),
            'P3_ref': tr_3d + t2 * (br_3d - tr_3d),
            
            # 세로 측정: obj1의 반대편 엣지가 기준
            'P5_ref': bl_3d,  # 위쪽 측정 -> 아래 엣지가 기준
            'P7_ref': tl_3d   # 아래쪽 측정 -> 위 엣지가 기준
        }
    
    def _calculate_directions(self) -> Dict[str, np.ndarray]:
        """각 측정선의 방향 벡터 계산"""
        tl_3d, bl_3d, br_3d, tr_3d = self.obj1_corners_3d
        
        left_edge_length = np.linalg.norm(bl_3d - tl_3d)
        t1 = self.offset_m / left_edge_length
        t2 = 1.0 - t1
        
        # 오른쪽 엣지의 대응점
        p1_right = tr_3d + t1 * (br_3d - tr_3d)
        p3_right = tr_3d + t2 * (br_3d - tr_3d)
        
        # 왼쪽 엣지의 대응점
        p1_left = tl_3d + t1 * (bl_3d - tl_3d)
        p3_left = tl_3d + t2 * (bl_3d - tl_3d)
        
        return {
            # 가로 방향 (왼쪽 → 오른쪽)
            'P1': self._normalize_vector(p1_right - p1_left),
            'P3': self._normalize_vector(p3_right - p3_left),
            
            # 세로 방향
            'P5': self._normalize_vector(tl_3d - bl_3d),  # 아래 → 위
            'P7': self._normalize_vector(bl_3d - tl_3d)   # 위 → 아래
        }
    
    # ==================== 거리 계산 ====================
    
    def _compute_signed_relative_distance(
        self,
        start_pt: np.ndarray,
        end_pt: np.ndarray,
        ref_pt: np.ndarray,
        direction: np.ndarray
    ) -> float:
        """
        부호 있는 상대 거리 계산
        
        거리 = (기준점→obj2) - (기준점→obj1 엣지)
        - 양수: obj2가 obj1보다 큰 경우
        - 음수: obj2가 obj1보다 작은 경우
        """
        # 평면에 투영
        proj_start = self._project_to_plane(start_pt)
        proj_end = self._project_to_plane(end_pt)
        proj_ref = self._project_to_plane(ref_pt)
        
        # 평면 상의 방향 벡터
        dir_plane = self._project_direction_to_plane(direction)
        
        if dir_plane is None:
            return 0.0
        
        # 기준점 기준 부호 있는 거리 계산
        signed_dist_start = self._compute_signed_distance(
            proj_ref, proj_start, dir_plane
        )
        signed_dist_end = self._compute_signed_distance(
            proj_ref, proj_end, dir_plane
        )
        
        # 상대 거리 (mm 단위)
        relative_distance = (signed_dist_end - signed_dist_start) * 1000.0
        
        return relative_distance
    
    def _compute_signed_distance(
        self,
        from_pt: np.ndarray,
        to_pt: np.ndarray,
        direction: np.ndarray
    ) -> float:
        """두 점 사이의 부호 있는 거리 계산"""
        vec = to_pt - from_pt
        dist = np.linalg.norm(vec)
        
        if dist < self.EPSILON:
            return 0.0
        
        # 방향 벡터와의 내적으로 부호 결정
        sign = np.sign(np.dot(vec / dist, direction))
        
        return sign * dist
    
    # ==================== 교차점 계산 ====================
    
    def _find_intersection_with_obj2(
        self,
        start_pt: np.ndarray,
        direction: np.ndarray
    ) -> Optional[np.ndarray]:
        """obj2의 엣지와 측정선의 교차점 찾기"""
        # 방향을 평면에 투영
        dir_plane = self._project_direction_to_plane(direction)
        if dir_plane is None:
            return None
        
        # 시작점을 평면에 투영
        start_plane = self._project_to_plane(start_pt)
        
        # obj2의 모든 엣지와 교차점 검사
        obj2_edges = self._get_obj2_edges()
        
        best_intersection = None
        best_distance = np.inf
        
        for edge_start, edge_end in obj2_edges:
            intersection = self._compute_line_segment_intersection(
                start_plane, dir_plane, edge_start, edge_end
            )
            
            if intersection is not None:
                dist = np.linalg.norm(intersection - start_plane)
                if dist < best_distance:
                    best_distance = dist
                    best_intersection = intersection
        
        return best_intersection
    
    def _get_obj2_edges(self) -> list:
        """obj2의 4개 엣지를 리스트로 반환"""
        corners = self.obj2_corners_3d
        return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]
    
    def _compute_line_segment_intersection(
        self,
        ray_origin: np.ndarray,
        ray_dir: np.ndarray,
        seg_start: np.ndarray,
        seg_end: np.ndarray
    ) -> Optional[np.ndarray]:
        """
        2D 평면 상에서 ray와 line segment의 교차점 계산
        평면의 법선 방향을 제외한 2개 축에서 계산
        """
        # 평면 법선에 수직인 2개의 축 선택
        k = np.argmax(np.abs(self.plane_normal))
        axes = [i for i in range(3) if i != k]
        
        if len(axes) < 2:
            return None
        
        i, j = axes[0], axes[1]
        
        # 2x2 선형 시스템 구성
        seg_dir = seg_end - seg_start
        
        A = np.array([
            [ray_dir[i], -seg_dir[i]],
            [ray_dir[j], -seg_dir[j]]
        ], dtype=float)
        
        b = np.array([
            seg_start[i] - ray_origin[i],
            seg_start[j] - ray_origin[j]
        ], dtype=float)
        
        # 행렬식 체크
        det = np.linalg.det(A)
        if abs(det) < self.EPSILON:
            return None
        
        # 해 구하기
        s, u = np.linalg.solve(A, b)
        
        # segment 범위 체크 (0 <= u <= 1)
        if u < 0.0 or u > 1.0:
            return None
        
        # 교차점 계산
        intersection = ray_origin + s * ray_dir
        
        return intersection
    
    # ==================== 3D 변환 유틸리티 ====================
    
    def _unproject_to_plane(self, points_2d: np.ndarray) -> Optional[np.ndarray]:
        """2D 이미지 좌표를 3D 평면 좌표로 언프로젝션"""
        # 왜곡 보정
        pts_reshaped = points_2d.reshape(-1, 1, 2).astype(np.float32)
        xy_undist = cv2.undistortPoints(
            pts_reshaped, self.camera_matrix, self.dist_coeffs
        ).squeeze()
        
        if xy_undist.ndim == 1:
            xy_undist = xy_undist.reshape(1, -1)
        
        # 각 점에 대해 ray-plane 교차점 계산
        points_3d = []
        for x_n, y_n in xy_undist:
            point_3d = self._ray_plane_intersection(x_n, y_n)
            if point_3d is not None:
                points_3d.append(point_3d)
        
        return np.array(points_3d) if points_3d else None
    
    def _ray_plane_intersection(self, x_norm: float, y_norm: float) -> Optional[np.ndarray]:
        """정규화된 이미지 좌표에서 출발하는 ray와 평면의 교차점"""
        # Ray 방향 벡터
        ray_dir = np.array([x_norm, y_norm, 1.0], dtype=np.float32)
        ray_dir = self._normalize_vector(ray_dir)
        
        # Ray-plane 교차점 계산
        denom = np.dot(self.plane_normal, ray_dir)
        if abs(denom) < self.EPSILON:
            return None  # Ray가 평면과 평행
        
        t = np.dot(self.plane_normal, self.plane_centroid) / denom
        if t <= 0:
            return None  # 카메라 뒤쪽
        
        return ray_dir * t
    
    def _project_to_plane(self, point: np.ndarray) -> np.ndarray:
        """3D 점을 평면에 투영"""
        vec_to_point = point - self.plane_centroid
        distance = np.dot(vec_to_point, self.plane_normal)
        return point - distance * self.plane_normal
    
    def _project_direction_to_plane(self, direction: np.ndarray) -> Optional[np.ndarray]:
        """방향 벡터를 평면에 투영 (평면의 법선 성분 제거)"""
        # 법선 성분 제거
        projected = direction - np.dot(direction, self.plane_normal) * self.plane_normal
        
        # 정규화
        norm = np.linalg.norm(projected)
        if norm < self.EPSILON:
            return None
        
        return projected / norm
    
    # ==================== 벡터 유틸리티 ====================
    
    @staticmethod
    def _normalize_vector(vector: np.ndarray) -> np.ndarray:
        """벡터 정규화"""
        norm = np.linalg.norm(vector)
        epsilon = 1e-12
        return vector / (norm + epsilon) if norm > epsilon else vector