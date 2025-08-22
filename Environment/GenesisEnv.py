from typing import Dict, Union
import gymnasium as gym
import genesis as gs

from Config import *

class GenesisGym(gym.Env):
    
    scene: gs.Scene
    action_space: gym.Space
    state_space: gym.Space
    
    _sensor_configs: CameraConfig
    
    def __init__(self, n_envs):
        super(GenesisGym, self).__init__()
        

    def reset(self,):
        pass
    
    def add_entity(self, entity: Union[gs.morphs.Morph, gs.morphs.Drone, gs.morphs.Mesh]):
        self.scene.add_entity(entity)
    
    def build_scene(self):
        
        pass
    
    def step(self, action):
        return super().step(action)
    
    def close(self):
        return super().close()
    
    def get_agent_state(self):
        pass