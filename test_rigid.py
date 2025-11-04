import genesis as gs

gs.init(backend=gs.gpu)

scene = gs.Scene(show_viewer=True)

# 바닥
plane = scene.add_entity(gs.morphs.Plane())

# 박스: (pos,euler/quat, size)로 지정. Box는 (lower,upper)로도 생성 가능
box = scene.add_entity(
    gs.morphs.Box(pos=(0,0.0,0.0), size=(0.5, 0.5, 0.5), fixed=False),
    material=gs.materials.Rigid(rho=500.0, friction=0.8),
    surface=gs.surfaces.Metal(),            # 렌더링 재질
    visualize_contact=True,
)

box2 = scene.add_entity(
    gs.morphs.Box(pos=(0,0,3.0), size=(1.5, 0.5, 0.2), euler=[90.0, 0.0, 0.0], fixed=False),
    material=gs.materials.Rigid(rho=500.0, friction=0.8),
    surface=gs.surfaces.Metal(),            # 렌더링 재질
    visualize_contact=True,
)

scene.build()
for _ in range(240):
    scene.step()
    input()
