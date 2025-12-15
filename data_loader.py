import os
from glob import glob
from PIL import Image
import open3d as o3d
import re

class DataLoader:
    @staticmethod
    def load_data_names(video_dir: str, pcd_dir: str):
        frame_names = sorted(glob(os.path.join(video_dir, "*.jpg")))
        pcd_names = sorted(glob(os.path.join(pcd_dir, "*.pcd")))
        return frame_names, pcd_names

    @staticmethod
    def load_image(path: str):
        img = Image.open(path)
        return img, img.shape[1], img.shape[0]

    @staticmethod
    def load_pcd(path: str):
        return o3d.io.read_point_cloud(path)
