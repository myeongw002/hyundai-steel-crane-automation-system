import open3d as o3d
import numpy as np
import cv2
from pathlib import Path
from typing import List, Tuple, Optional, Dict

COLORS = [
    (255, 0, 0),
    (0, 0, 255),
    (0, 255, 0),
    (255, 255, 0),
    (255, 0, 255),
    (0, 255, 255),
]

LINE_COLORS = {
    'P1-P2': (0, 255, 255),
    'P3-P4': (255, 255, 0),
    'P5-P6': (255, 0, 255),
    'P7-P8': (0, 0, 255)
}

def sam2_visualizer(image, masks, session_id, output_path="output"):
    for obj_id, mask in masks.items():
        color = COLORS[obj_id % len(COLORS)]
        colored_mask = np.zeros_like(image)
        colored_mask[mask] = color
        image = cv2.addWeighted(image, 0.7, colored_mask, 0.3, 0)
    
    save_path = output_path / f"{session_id}_sam2.png"
    cv2.imwrite(str(save_path), image)
    print(f"저장: {save_path.name}")

def mask_visualizer(image, masks, session_id, output_path):
    for obj_id, mask in enumerate(masks):
        obj_id = obj_id + 1
        color = COLORS[obj_id % len(COLORS)]
        colored_mask = np.zeros_like(image)
        colored_mask[mask] = color
        image = cv2.addWeighted(image, 0.7, colored_mask, 0.3, 0)
    
    save_path = output_path / f"{session_id}_mask.png"
    cv2.imwrite(str(save_path), image)
    print(f"저장: {save_path.name}")

def box_visualizer(image, corners_2d_list, session_id, output_path):
    for corners_2d in corners_2d_list:
        corners_2d = np.int32(corners_2d)
        cv2.drawContours(image, [corners_2d], 0, COLORS[0], 3)
    
    save_path = output_path / f"{session_id}_box.png"
    cv2.imwrite(str(save_path), image)
    print(f"저장: {save_path.name}")

def measurement_visualizer(
    image,
    corners,
    measure_points_3d,
    camera_matrix,
    dist_coeffs,
    measurements,
    session_id,
    output_path
):
    overlay = image.copy()
    
    cv2.polylines(overlay, [corners[0].astype(int)], True, (0, 255, 255), 2)
    cv2.polylines(overlay, [corners[1].astype(int)], True, (0, 255, 0), 2)
    
    point_names = ['P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7', 'P8']
    points_3d = [measure_points_3d[name] for name in point_names if name in measure_points_3d]
    
    if not points_3d:
        return overlay
    
    pixels_2d, _ = cv2.projectPoints(
        np.array(points_3d),
        np.zeros(3),
        np.zeros(3),
        camera_matrix,
        None
    )
    pixels = pixels_2d.reshape(-1, 2).astype(int)
    
    idx_map = {name: i for i, name in enumerate(point_names) if name in measure_points_3d}
    pairs = [('P1', 'P2'), ('P3', 'P4'), ('P5', 'P6'), ('P7', 'P8')]
    
    for start, end in pairs:
        pair_name = f"{start}-{end}"
        
        if start not in idx_map or end not in idx_map:
            continue
        
        pt1 = tuple(pixels[idx_map[start]])
        pt2 = tuple(pixels[idx_map[end]])
        color = LINE_COLORS[pair_name]
        
        cv2.circle(overlay, pt1, 5, color, -1)
        cv2.drawMarker(overlay, pt2, color, cv2.MARKER_CROSS, 10, 2)
        cv2.arrowedLine(overlay, pt1, pt2, color, 2, tipLength=0.03)
        
        mid = ((pt1[0] + pt2[0]) // 2, (pt1[1] + pt2[1]) // 2)
        dist = measurements.get(pair_name, 0.0)
            
        text = f"{dist:.0f}mm"
        cv2.putText(
            overlay, text, (mid[0] - 30, mid[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA
        )
    
    save_path = output_path / f"{session_id}_result.png"
    cv2.imwrite(str(save_path), overlay)
    print(f"저장: {save_path.name}")
