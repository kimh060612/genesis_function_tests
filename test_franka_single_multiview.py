import genesis as gs
import numpy as np
import pickle

########################## init ##########################
gs.init(backend=gs.gpu)

########################## create a scene ##########################
scene = gs.Scene(
    viewer_options=gs.options.ViewerOptions(
        camera_pos=(3, -1, 1.5),
        camera_lookat=(0.0, 0.0, 0.5),
        camera_fov=30,
        max_FPS=60,
    ),
    sim_options=gs.options.SimOptions(
        dt=0.01,
    ),
    # vis_options=gs.options.VisOptions(
    #     lights = [{'type': 'directional', 'dir': (-1, -1, -1), 'color': (1, 1, 1), 'intensity': 50.0}],
    # ),
    show_viewer=True,
)

cam = scene.add_camera(
    res=(720, 480),
    pos=(1.5, -0.5, 1.5),
    lookat=(0.0, 0.0, 0.5),
    fov=70,
    GUI=False,
)

cam2 = scene.add_camera(
    res=(720, 480),
    pos=(-0.8, -0.8, 1.5),
    lookat=(0.0, 0.0, 0.6),
    fov=70,
    GUI=False,
)

cam3 = scene.add_camera(
    res=(720, 480),
    pos=(0.2, 1.4, 0.7),
    lookat=(0.0, -0.8, 0.0),
    fov=70,
    GUI=False,
)

########################## entities ##########################
plane = scene.add_entity(
    gs.morphs.Plane(),
)
cube = scene.add_entity(
    gs.morphs.Box(
        size=(0.04, 0.04, 0.04),
        pos=(0.65, 0.3, 0.02),
    )
)
franka = scene.add_entity(
    gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"),
)

########################## build ##########################
scene.build()
# rgb, depth, segmentation, normal = cam.render(rgb=True, depth=True, segmentation=True, normal=True)
cam.start_recording()
cam2.start_recording()
cam3.start_recording()

motors_dof = np.arange(7)
fingers_dof = np.arange(7, 9)


# set control gains
franka.set_dofs_kp(
    np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
)
franka.set_dofs_kv(
    np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
)
franka.set_dofs_force_range(
    np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
    np.array([87, 87, 87, 87, 12, 12, 12, 100, 100]),
)


end_effector = franka.get_link("hand")

# move to pre-grasp pose
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.3, 0.25]),
    quat=np.array([0, 1, 0, 0]),
)
# gripper open pos
qpos[-2:] = 0.04
path = franka.plan_path(
    qpos_goal=qpos,
    num_waypoints=200,  # 2s duration
)
# path_debug = scene.draw_debug_path(path, franka)


# execute the planned path
for wp_1 in path:
    franka.control_dofs_position(wp_1)
    scene.step()
    cam.render(depth=True)    
    cam2.render(depth=True)
    cam3.render(depth=True)

# remove the drawn path
# scene.clear_debug_object(path_debug)

# allow robot to reach the last waypoint
for i in range(20):
    scene.step()
    cam.render(depth=True)
    cam2.render(depth=True)
    cam3.render(depth=True)

# reach
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.3, 0.130]),
    quat=np.array([0, 1, 0, 0]),
)
print(qpos)
franka.control_dofs_position(qpos[:-2], motors_dof)
for i in range(80):
    scene.step()
    cam.render(depth=True)
    cam2.render(depth=True)
    cam3.render(depth=True)

# grasp
franka.control_dofs_position(qpos[:-2], motors_dof)
franka.control_dofs_force(np.array([-1.0, -1.0]), fingers_dof)

for i in range(50):
    scene.step()
    cam.render(depth=True)
    cam2.render(depth=True)
    cam3.render(depth=True)

# lift
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.3, 0.58]),
    quat=np.array([0.0, 1.0, 0.0, 0.0]), 
)
# print(qpos)
franka.control_dofs_position(qpos[:-2], motors_dof)

for i in range(80):
    scene.step()
    cam.render(depth=True)
    cam2.render(depth=True)
    cam3.render(depth=True)

cam.stop_recording(save_to_filename="video.mp4", fps=35)
cam2.stop_recording(save_to_filename="video2.mp4", fps=35)
cam3.stop_recording(save_to_filename="video3.mp4", fps=35)
