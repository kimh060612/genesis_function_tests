"""
replicacad2genesis_hf.py
------------------------
ReplicaCAD(HF) → Genesis 0.2  변환 & 로더
• Python 3.11+
• pip install genesis-world==0.2.* scipy numpy tqdm
"""

from __future__ import annotations
import json, pathlib, numpy as np, genesis as gs
from scipy.spatial.transform import Rotation as R
import os
import trimesh

###############################################################################
# 0. 경로 설정 ---------------------------------------------------------------
###############################################################################
DATA_ROOT   = pathlib.Path("/dataset/scene_datasets/replica_cad_dataset/")                    # HF repo root
SCENE_DIR   = DATA_ROOT / "configs" / "scenes"
OBJECT_DIR  = DATA_ROOT / "configs" / "objects"

###############################################################################
# 1. 좌표계 · 회전 변환 -------------------------------------------------------
# Habitat:  (x right,  y up,   z forward -)
# Genesis:  (x right,  z up,   y forward  +)
###############################################################################
_AX_SWAP = np.array([[1,  0,  0],
                     [0,  0, -1],
                     [0,  1,  0]])

def hab2gen_pos(pos_xyz: list[float]) -> tuple[float, float, float]:
    """(x, y, z)_Hab → (x, y', z')_Gen"""
    x, y, z = pos_xyz
    return  (x, -z, y + 0.3) # (x, -z, y)

def hab2gen_quat(q) -> tuple[float, float, float, float]:
    """
    HF ReplicaCAD 쿼터니언은 [w, x, y, z] 순 (문서 기준).
    Genesis `quat` 역시 (w, x, y, z)이므로,
    먼저 SciPy → 행렬 → 축 변환 → 다시 쿼터니언으로 돌려준다.
    """
    if isinstance(q, dict):      # {"w":..., "x":..., ...} 형태
        w, x, y, z = q["w"], q["x"], q["y"], q["z"]
    else:                        # [w, x, y, z] 리스트/튜플
        w, x, y, z = q
    r_hab = R.from_quat([x, y, z, w])          # SciPy는 [x,y,z,w]
    m_gen = _AX_SWAP @ r_hab.as_matrix() # @ _AX_SWAP.T
    xg, yg, zg, wg = R.from_matrix(m_gen).as_quat()
    return (wg, xg, yg, zg)                    # Genesis convention

def quat_to_rot(q: list):
    np_q = np.array(q)
    return R.from_quat(np_q).as_euler('XYZ', degrees=False)

def merge_glbs(render_glb_path, collision_glb_path, output_glb_path):
    # 렌더링 mesh 불러오기
    render_mesh = trimesh.load(render_glb_path)
    # 충돌용 mesh 불러오기
    collision_mesh = trimesh.load(collision_glb_path)

    # 각각 scene의 다른 노드로 추가
    scene = trimesh.Scene()
    scene.add_geometry(render_mesh, node_name="render")
    scene.add_geometry(collision_mesh, node_name="collision")

    # 병합된 glb로 내보내기
    scene.export(output_glb_path)
    print(f"병합 완료: {output_glb_path}")

###############################################################################
# 2. Object config 로드 -------------------------------------------------------
###############################################################################
_cfg_cache: dict[str, dict] = {}

def load_obj_cfg(template_name: str) -> dict:
    """template_name 예: 'objects/frl_apartment_basket' (확장자 없이)"""
    if template_name not in _cfg_cache:
        fname = OBJECT_DIR / (pathlib.Path(template_name).name + ".object_config.json")
        _cfg_cache[template_name] = json.loads(fname.read_text())
    return _cfg_cache[template_name]

###############################################################################
# 3. Scene 생성 --------------------------------------------------------------
###############################################################################
def load_replicacad(scene_json: pathlib.Path,
                    backend: str = "gpu",
                    show_viewer: bool = True) -> gs.Scene:
    """
    * `backend`: 'gpu' | 'vulkan' | 'cpu'
    * return: 완성된 genesis.Scene
    """
    inst = json.loads(scene_json.read_text())

    gs.init(backend=getattr(gs, backend))       # gs.init(...) -> 필수 초기화:contentReference[oaicite:0]{index=0}
    camera_pos = hab2gen_pos((0.0, 5.0, 0.0))  # ⭐︎ 추가
    camera_quat = quat_to_rot(hab2gen_quat([0., 1., 0., 0.]))
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(
            dt=0.01,
            substeps=1,
            gravity=(0, 0, -9.81),
        ),
        viewer_options=gs.options.ViewerOptions(
            max_FPS=60,
            camera_pos=camera_pos,
            camera_lookat=camera_quat,
            camera_fov=40,
        ),
        rigid_options=gs.options.RigidOptions(
            enable_collision=True,
            enable_self_collision=True
        ),
        show_viewer=show_viewer
    )   # gs.Scene(...) → 엔티티 컨테이너:contentReference[oaicite:1]{index=1}    
    # ────────── Stage (고정) ──────────
    stage_name = inst["stage_instance"]["template_name"]   # 'stages/apt_0_stage'
    stage_glb  = DATA_ROOT / f"{stage_name}.glb"
    
    scene.add_entity(
        morph=gs.morphs.Mesh(
            file=str(stage_glb),
            pos=[0.0, 0.0, 0.1],            # ← 변환 후 좌표
            euler=[90.0, 0.0, 0.0],         # ← 변환 후 쿼터니언
            fixed=True, 
            collision=True,
            visualization=True,
            convexify=False,
            # decimate_face_num = 10000
        ),
            visualize_contact=True
        # vis_mode="collision"
    )
    
    # box = scene.add_entity(
    #     gs.morphs.Box(
    #         pos=(0.0, 0.0, 6.5),
    #         size=(30.0, 30.0, 0.1),
    #         fixed=False
    #     ),
    #     material=gs.materials.Rigid(rho=0.1, friction=0.8),
    #     visualize_contact=True,
    # )
    
    # floor = scene.add_entity(
    #     gs.morphs.Box(
    #         size=(30, 30, 0.1),
    #         pos=(0.0, 0.0, -0.1), 
    #         euler=(0.0, 0.0, 90.0),
    #         fixed=True, collision=True, visualization=False
    #     )
    # )
    scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True),
                     visualize_contact=False
    )  # 바닥: URDF로 추가, 고정체
    
    # ────────── Objects ──────────
    idx = 0
    count_non_collision = 0
    for obj in inst["object_instances"]:
        tname = obj["template_name"]                      # 'objects/xxx'
        if "frl_apartment_bike_0" in tname:
            continue
        
        cfg   = load_obj_cfg(tname)
        
        vis_glb = DATA_ROOT / "configs/objects" / cfg["render_asset"]         # 시각 메시
        merged_glb = DATA_ROOT / (tname + "_merged.glb")
        pos     = hab2gen_pos(obj["translation"])
        quat    = hab2gen_quat(obj["rotation"])
        fixed = obj["motion_type"] == "STATIC" or cfg.get("mass", 1.0) > 5       # 무게 0이면 고정체
                                                # 일반 rigid CAD
        if "collision_asset" in cfg:
            collision_file = OBJECT_DIR / cfg["collision_asset"]
            # merge_glbs(vis_glb, collision_file, merged_glb)
            scene.add_entity(
                gs.morphs.Mesh(
                    file=str(vis_glb),
                    pos=pos, quat=quat, fixed=False,
                    visualization=True, collision=True,
                    convexify=False
                )
            )
        else: 
            count_non_collision += 1
            scene.add_entity(
                gs.morphs.Mesh(
                    file=str(vis_glb),
                    pos=pos, quat=quat, 
                    fixed=False,
                    visualization=True, 
                    collision=True,
                    coacd_options=gs.options.CoacdOptions(
                        decimate=True,
                        pca=True
                    )
                ),
            )
  
        idx += 1
        # if idx == 1:  # 첫 5개만 로드
        #     break
  
    
    for aobj in inst["articulated_object_instances"]:
        template_name = aobj["template_name"]
        if "door" in template_name:
            continue
        pos = hab2gen_pos(aobj["translation"])
        quat = hab2gen_quat(aobj["rotation"])
        urdf_path = os.path.join(
            DATA_ROOT,
            f"urdf/{template_name}/{template_name}.urdf",
        )
        scale = 1.0
        if "uniform_scale" in aobj:
            scale = aobj["uniform_scale"]
        scene.add_entity(
            gs.morphs.URDF(
                file=urdf_path,
                pos=pos, quat=quat,
                fixed=False,
                collision=True,
                visualization=True,
                scale=scale,
                convexify=False
            )
        )
    
    # locobot = scene.add_entity(
    #     gs.morphs.URDF(
    #         file="urdf/go2/urdf/go2.urdf", 
    #         pos=[0.0, 0.0, 0.0],
    #         euler=[0.0, 0.0, 0.0]
    #     ),
    # )
    # print("Number of Non-Collisionable Dynamic Objects:", count_non_collision)
    scene.build()                                         # ★필수: 시뮬 초기화
    return scene

###############################################################################
# 4. 간단 실행 예시 -----------------------------------------------------------
###############################################################################
if __name__ == "__main__":
    # 첫 번째 아파트 장면 하나만 로드
    demo_scene = SCENE_DIR / "apt_0.scene_instance.json"
    scene = load_replicacad(demo_scene, backend="gpu", show_viewer=True)
    # 필요하다면 여기서 로봇(LoCoBot 등) 추가 후 scene.build() 재호출
    print("Genesis + ReplicaCAD 장면 로드 완료! 창을 닫으면 종료됩니다.")
    while True:
        scene.step()
        input()
