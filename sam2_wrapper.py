import cv2
import numpy as np
import torch
import sys
from typing import List, Tuple, Dict, Optional, Any


class SAM2Wrapper:
    def __init__(self, config):
        self.device = self._setup_device()
        try:
            from sam2.build_sam import build_sam2_video_predictor
            self.video_predictor = build_sam2_video_predictor(
                config.SAM2_CONFIG, config.SAM2_CHECKPOINT, device=self.device
            )
            
            from sam2.build_sam import build_sam2
            from sam2.sam2_image_predictor import SAM2ImagePredictor
            sam2_model = build_sam2(config.SAM2_CONFIG, config.SAM2_CHECKPOINT, device=self.device)
            self.predictor = SAM2ImagePredictor(sam2_model)
        except ImportError:
            print("SAM2 module import failed.")
            sys.exit(1)
        self.state = None

    def _setup_device(self):
        if torch.cuda.is_available():
            torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
            if torch.cuda.get_device_properties(0).major >= 8:
                torch.backends.cuda.matmul.allow_tf32 = True
                torch.backends.cudnn.allow_tf32 = True
            return torch.device("cuda")
        return torch.device("cpu")

    def video_run(self, video_dir, prompts: List[Dict]):
        """Init state -> Add Prompts -> Propagate -> Return masks"""
        self.state = self.video_predictor.init_state(video_path=video_dir)
        self.video_predictor.reset_state(self.state)
        
        for p in prompts:
            self.video_predictor.add_new_points_or_box(
                inference_state=self.state,
                frame_idx=0,
                obj_id=p['id'],
                points=p['points'],
                labels=p['labels']
            )

        results = {}
        for f_idx, obj_ids, mask_logits in self.video_predictor.propagate_in_video(self.state):
            frame_res = {}
            for i, oid in enumerate(obj_ids):
                mask = (mask_logits[i] > 0).cpu().numpy().squeeze()
                frame_res[oid] = mask.astype(bool)
            results[f_idx] = frame_res
        return results
    
    def image_run(self, image, input_box, negative_point):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        input_box = np.append(input_box[0], input_box[2])
        if negative_point is not None:
            negative_label = np.zeros((len(negative_point)), dtype=int)
        else:
            negative_label = None
        
        self.predictor.set_image(image)
        masks, scores, _ = self.predictor.predict(
            point_coords=negative_point,
            point_labels=negative_label,
            box=input_box[None, :],
            multimask_output=False,
        )

        return masks.squeeze().astype(np.bool_)
