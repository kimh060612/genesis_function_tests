from typing import List, Union, Dict
import genesis as gs
from Environment import GenesisEnv
import os
import json
import numpy as np

from components import *
from constant import *

DATASET_CONFIG_DIR = os.path.join(os.path.dirname(__file__), "metadata")

class GenesisReplicaCADScene():
    
    build_configs: List[str] = None
    
    def __init__(self, num_envs=1, show_viewer=True, include_staging_scenes=True):
        with open(os.path.join(DATASET_CONFIG_DIR, "scene_configs.json")) as f:
            build_config_json = json.load(f)
            self.build_configs = build_config_json["scenes"]
            if include_staging_scenes:
                self.build_configs += build_config_json["staging_scenes"]
        
        self._navigable_positions = [None] * len(self.build_configs)
        self.build_config_idxs: List[int] = None
        
        self.scene = gs.Scene(
            num_envs=num_envs,
            show_viewer=show_viewer
        )
        
        
    def build(self, build_config_idxs: Union[int, List[int]]):
        """
        Inputs:
            - build_config_idxs: Scene Indices(or Index) to build with Genesis Simulation
        """
        if isinstance(build_config_idxs, int):
            build_config_idxs = [build_config_idxs] * self.scene.n_envs
        assert all([isinstance(bci, int) for bci in build_config_idxs])
        assert len(build_config_idxs) == self.scene.n_envs
        
        self.__dict__.pop("ray_traced_lighting", None)
        self.build_config_idxs = build_config_idxs
        self.scene_objects: Dict[str, Actor] = dict()
        self.movable_objects: Dict[str, Actor] = dict()
        self.articulations: Dict[str, Articulation] = dict()
        
        # keep track of background objects separately as we need to disable mobile robot collisions
        # note that we will create a merged actor using these objects to represent the bg
        bgs = [None] * self.env.num_envs
        for bci in np.unique(build_config_idxs):
            """
            Given a list of sampled build_config_idxs, build/load the scene objects

            TODO (arth): return builder option to set static/dynamic in env
                -  for now leave as-is since has smaller change in performance
            """

            env_idx = [i for i, v in enumerate(build_config_idxs) if v == bci]
            unique_id = "scs-" + str(env_idx).replace(" ", "")
            build_config_path = self.build_configs[bci]
        
            # We read the json config file describing the scene setup for the selected ReplicaCAD scene
            with open(
                os.path.join(
                    REPLICACAD_ASSET_DIR,
                    "scene_datasets/replica_cad_dataset/configs/scenes",
                    build_config_path,
                ),
                "rb",
            ) as f:
                build_config_json = json.load(f)
        
            # The complex part of porting over scene datasets is that each scene dataset often has it's own format and there is no
            # one size fits all solution to read that format and use it. The best way to port a scene dataset over is to look
            # at the configuration files, get a sense of the pattern and find how they reference .glb model files and potentially
            # decomposed convex meshes for physical simulation

            # ReplicaCAD stores the background model here
            background_template_name = os.path.basename(
                build_config_json["stage_instance"]["template_name"]
            )
            bg_path = str(
                REPLICACAD_ASSET_DIR
                / f"scene_datasets/replica_cad_dataset/stages/{background_template_name}.glb"
            )
            
        
        pass
    
    
    
    