import genesis as gs, numpy as np, prior, pathlib, json

ASSET_ROOT   = pathlib.Path('thor_glb')          # .glb 풀 경로
ASSET_INDEX  = json.load(open('thor_to_glb_map.json'))

def unity_to_genesis(p):
    # Unity: x-right, y-up, z-forward  ↔  Genesis: x-right, y-forward, z-up
    return (p['x'], p['z'], p['y'])

def load_house(split='train', idx=0):
    return prior.load_dataset("procthor-10k", split=split, index=idx)

def build_scene(house):
    gs.init(backend=gs.gpu)
    scene = gs.Scene(show_viewer=True)
    scene.add_entity(gs.morphs.Plane())          # ground

    for obj in house['objects']:
        mesh = ASSET_ROOT / ASSET_INDEX[obj['objectType']]
        pos  = unity_to_genesis(obj['position'])
        yaw  = -obj['rotation']['y']             # Unity Y↔Genesis Z
        scene.add_entity(
            gs.morphs.Mesh(file=str(mesh),
                           pos=pos,
                           euler=(0, 0, yaw),
                           scale=obj['scale']))
    locobot = scene.add_entity(gs.morphs.URDF(
        file='urdf/locobot/locobot.urdf', pos=unity_to_genesis(
            house['agent_start_position'])))
    scene.build()
    return scene, locobot

if __name__ == "__main__":
    house = load_house('train', 0)
    scene, robot = build_scene(house)
    for _ in range(500):
        scene.step()
