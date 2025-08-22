# test_no_transform.py
import genesis as gs, json, pathlib
import numpy as np
# import math
# import trimesh

gs.init(backend=gs.gpu)
scene = gs.Scene(
    viewer_options=gs.options.ViewerOptions(
        max_FPS=int(50),
        camera_pos=(0.0, 5.0, 0.0),
        camera_lookat=(0.0, 0.0, 1.0),
        camera_fov=60,
    ),
    show_viewer=True
)

SCENE = pathlib.Path("/datasets/scene_datasets/replica_cad_dataset/configs/scenes/apt_0.scene_instance.json")
inst = json.loads(SCENE.read_text())
stage_name = inst["stage_instance"]["template_name"]
print(stage_name)
stage_glb  = pathlib.Path("/datasets/scene_datasets/replica_cad_dataset") / f"{stage_name}.glb"

# 1) 변환 *없이* 바로 로드
stage = scene.add_entity(
    morph=gs.morphs.Mesh(
        file=str(stage_glb),
        pos=[0.0, 0.0, 0.0],                 
        euler=[90.0, 0.0, 0.0],              
        fixed=True, 
        collision=True,
        visualization=True,
        convexify=False,
    ),
    # vis_mode="collision"
)

probes=[]
for i,(x,y) in enumerate([(0,0),(1,0),(-1,0),(0,1),(0,-1)]):
    probes.append(scene.add_entity(
        morph=gs.morphs.Sphere(
            radius=0.1, 
            pos=(x, y, 1.0), 
            collision=True
        ),
        # vis_mode="collision"
    ))
    
# scene.add_entity(gs.morphs.URDF(file="urdf/plane/plane.urdf", fixed=True))
floor = scene.add_entity(gs.morphs.Box(
    size=(50,50,0.1),
    pos=(0,0,0), fixed=True, collision=True, visualization=False)
)

scene.build()
print("Stage pos:", stage.get_pos())
print("Stage quat:", stage.get_quat())
print("Stage AABB:", stage.get_AABB())

scene.draw_debug_frame(np.eye(4))

# 2) 월드 축 표시 (가능하면)
# scene.draw_debug_frame(np.eye(4))  # 일부 버전에서 지원

step = 0
while True:
    scene.step()
    if step % 5 == 0:
        print(step, [ (
            round(p.get_pos()[0].item(),2), 
            round(p.get_pos()[1].item(),2), 
            round(p.get_pos()[2].item(),2)
        ) for p in probes ])
    step += 1


