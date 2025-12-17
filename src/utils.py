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

def refine_mask(mask, kernel):
    mask = mask.astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=kernel)
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
        self.obj1_corners_2d = obj1_corners_2d
        self.obj1_corners_3d = obj1_corners_3d
        self.obj2_corners_3d = obj2_corners_3d
        self.plane_normal = plane_normal / (np.linalg.norm(plane_normal) + 1e-12)
        self.plane_centroid = plane_centroid
        self.camera_matrix = camera_matrix
        self.dist_coeffs = None
        self.offset_mm = offset_mm
        if measurement_offsets is None:
            self.measurement_offsets = {
                'P1-P2': 0.0,
                'P3-P4': 0.0,
                'P5-P6': 0.0,
                'P7-P8': 0.0
            }
        else:
            self.measurement_offsets = measurement_offsets
        
    def calculate(self) -> Tuple[Optional[Dict[str, float]], Optional[Dict[str, np.ndarray]]]:
        measure_points_2d = self._generate_measurement_points()
        measure_points_3d = self._unproject_to_plane(measure_points_2d)
        
        if measure_points_3d is None or len(measure_points_3d) < 4:
            print("측정점 생성 실패")
            return None, None
        
        P1, P3, P5, P7 = measure_points_3d
        
        directions = self._calculate_directions()
        
        measurements = {}
        points_dict = {}
        
        measure_configs = [
            ('P1-P2', P1, directions['P1'], 'P1', 'P2', '위쪽 길이'),
            ('P3-P4', P3, directions['P3'], 'P3', 'P4', '아래쪽 길이'),
            ('P5-P6', P5, directions['P5'], 'P5', 'P6', '위쪽 너비'),
            ('P7-P8', P7, directions['P7'], 'P7', 'P8', '아래쪽 너비')
        ]
        
        for name, start_pt, direction, start_label, end_label, desc in measure_configs:
            end_pt = self._find_intersection(start_pt, direction)
            
            if end_pt is not None:
                proj_start = self._project_to_plane(start_pt)
                distance = np.linalg.norm(end_pt - proj_start) * 1000.0
                
                offset = self.measurement_offsets.get(name, 0.0)
                corrected_distance = distance + offset
                measurements[name] = corrected_distance
                points_dict[start_label] = start_pt
                points_dict[end_label] = end_pt
                
                print(f"{name} ({desc}): {distance:.1f}mm + {offset:.1f}mm = {corrected_distance:.1f}mm")
            else:
                measurements[name] = 0.0
                print(f"{name} 교차점 없음")
        
        return measurements, points_dict
    
    def _generate_measurement_points(self) -> np.ndarray:
        tl, bl, br, tr = self.obj1_corners_2d
        
        left_edge_length = np.linalg.norm(self.obj1_corners_3d[1] - self.obj1_corners_3d[0])
        offset_m = self.offset_mm / 1000.0
        
        t1 = offset_m / left_edge_length
        t2 = 1.0 - t1
        
        p1 = tl + t1 * (bl - tl)
        p3 = tl + t2 * (bl - tl)
        p5 = tl
        p7 = bl
        
        return np.array([p1, p3, p5, p7])
    
    def _calculate_directions(self) -> Dict[str, np.ndarray]:
        tl_3d, bl_3d, br_3d, tr_3d = self.obj1_corners_3d
        
        left_edge_length = np.linalg.norm(bl_3d - tl_3d)
        offset_m = self.offset_mm / 1000.0
        t1 = offset_m / left_edge_length
        t2 = 1.0 - t1
        
        p1_right = tr_3d + t1 * (br_3d - tr_3d)
        p3_right = tr_3d + t2 * (br_3d - tr_3d)
        
        dir_p1 = self._normalize(p1_right - (tl_3d + t1 * (bl_3d - tl_3d)))
        dir_p3 = self._normalize(p3_right - (tl_3d + t2 * (bl_3d - tl_3d)))
        dir_p5 = self._normalize(bl_3d - tl_3d)
        dir_p7 = dir_p5.copy()
        
        return {
            'P1': dir_p1,
            'P3': dir_p3,
            'P5': dir_p5,
            'P7': dir_p7
        }
    
    def _find_intersection(self, start_pt: np.ndarray, direction: np.ndarray) -> Optional[np.ndarray]:
        dir_plane = self._project_direction_to_plane(direction)
        if dir_plane is None:
            return None
        
        start_plane = self._project_to_plane(start_pt)
        
        corners = self.obj2_corners_3d
        edges = [(corners[i], corners[(i+1)%4]) for i in range(4)]
        
        best_point = None
        best_dist = np.inf
        
        for edge_start, edge_end in edges:
            intersection = self._line_segment_intersection(
                start_plane, dir_plane, edge_start, edge_end
            )
            
            if intersection is not None:
                dist = np.abs(np.linalg.norm(intersection - start_plane))
                if dist < best_dist:
                    best_dist = dist
                    best_point = intersection
        
        return best_point
    
    def _line_segment_intersection(
        self, 
        ray_origin: np.ndarray,
        ray_dir: np.ndarray,
        seg_start: np.ndarray,
        seg_end: np.ndarray
    ) -> Optional[np.ndarray]:
        k = np.argmax(np.abs(self.plane_normal))
        idx = [i for i in range(3) if i != k]
        
        if len(idx) < 2:
            return None
        
        i, j = idx[0], idx[1]
        
        seg_dir = seg_end - seg_start
        
        M = np.array([
            [ray_dir[i], -seg_dir[i]],
            [ray_dir[j], -seg_dir[j]]
        ], dtype=float)
        
        rhs = np.array([
            seg_start[i] - ray_origin[i],
            seg_start[j] - ray_origin[j]
        ], dtype=float)
        
        det = np.linalg.det(M)
        if abs(det) < 1e-12:
            return None
        
        s, u = np.linalg.solve(M, rhs)
        
        if u < 0.0 or u > 1.0:
            return None
        
        return ray_origin + s * ray_dir
    
    def _unproject_to_plane(self, points_2d: np.ndarray) -> Optional[np.ndarray]:
        pts_reshaped = points_2d.reshape(-1, 1, 2).astype(np.float32)
        xy_undist = cv2.undistortPoints(
            pts_reshaped, self.camera_matrix, self.dist_coeffs
        ).squeeze()
        
        if xy_undist.ndim == 1:
            xy_undist = xy_undist.reshape(1, -1)
        
        points_3d = []
        for x_n, y_n in xy_undist:
            ray_dir = np.array([x_n, y_n, 1.0], dtype=np.float32)
            ray_dir /= np.linalg.norm(ray_dir)
            
            denom = np.dot(self.plane_normal, ray_dir)
            if abs(denom) < 1e-6:
                continue
            
            t = np.dot(self.plane_normal, self.plane_centroid) / denom
            if t <= 0:
                continue
            
            points_3d.append(ray_dir * t)
        
        return np.array(points_3d) if points_3d else None
    
    def _project_to_plane(self, point: np.ndarray) -> np.ndarray:
        v = point - self.plane_centroid
        dist = np.dot(v, self.plane_normal)
        return point - dist * self.plane_normal
    
    def _project_direction_to_plane(self, direction: np.ndarray) -> Optional[np.ndarray]:
        proj = direction - np.dot(direction, self.plane_normal) * self.plane_normal
        norm = np.linalg.norm(proj)
        return proj / norm if norm > 1e-12 else None
    
    @staticmethod
    def _normalize(vector: np.ndarray) -> np.ndarray:
        norm = np.linalg.norm(vector)
        return vector / norm if norm > 1e-12 else vector