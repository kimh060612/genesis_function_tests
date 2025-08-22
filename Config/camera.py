from typing import Tuple
from dataclasses import dataclass

@dataclass
class CameraConfig:
    res: Tuple[int]
    fov: float
    GUI: bool
    pos: Tuple[float]
    lookat: Tuple[float]
    
    def set_pose(self, _p: Tuple):
        self.pos = _p
        
    def set_viewpoint(self, _la):
        self.lookat = _la