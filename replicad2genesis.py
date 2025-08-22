import json, pathlib, numpy as np, genesis as gs
from scipy.spatial.transform import Rotation as R

DATA_ROOT = pathlib.Path('/datasets/scene_datasets/replica_cad_dataset/')
SCENE_CFG  = DATA_ROOT / 'replicaCAD.scene_dataset_config.json'
GEN_ASSET  = pathlib.Path('gen_assets')  # optional texture copy

def hab2gen_pos(p):
    return (p['x'], -p['z'], p['y'])

def hab2gen_quat(q):
    r = R.from_quat([q['x'], q['y'], q['z'], q['w']])
    M = r.as_matrix()
    # swap rows/cols : (x,y,z)_hab → (x,z,y)_gen & flip hab_z
    S = np.array([[1,0,0],
                  [0,0,-1],
                  [0,1,0]])
    M_gen = S @ M @ S.T
    return R.from_matrix(M_gen).as_quat()  # x,y,z,w

def load_scene(scene_json):
    inst = json.load(open(scene_json))
    print(DATA_ROOT / inst['stage_instance']['template_name'])
    stg_glb = (DATA_ROOT / "stages" / inst['stage_instance']['template_name']).as_posix() + '.glb'
    obj_insts = inst['object_instances']

    gs.init(backend=gs.gpu)
    scene = gs.Scene(show_viewer=True)
    scene.add_entity(gs.morphs.Mesh(file=str(stg_glb)))

    for obj in obj_insts:
        cfg = (DATA_ROOT / "configs/objects" / obj['template_name']).as_posix() + '.object_config.json'
        meta = json.load(open(cfg))
        mesh = DATA_ROOT / "configs/objects" / meta['render_asset']      # .glb
        urdf = meta.get('urdf_filepath', None)

        if urdf:        # articulated 가구
            ent = scene.add_entity(gs.morphs.URDF(file=str(DATA_ROOT / urdf),
                                                  pos=hab2gen_pos(obj['translation']),
                                                  quat=hab2gen_quat(obj['rotation'])))
        else:           # rigid object
            ent = scene.add_entity(gs.morphs.Mesh(file=str(mesh),
                                                  pos=hab2gen_pos(obj['translation']),
                                                  quat=hab2gen_quat(obj['rotation']),
                                                  mass=meta['mass'],
                                                  friction=meta['friction']))
    scene.build()
    return scene
