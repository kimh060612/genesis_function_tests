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
    show_viewer=True,
)

cam = scene.add_camera(
    res=(1280, 960),
    pos=(1.0, 0.5, 2.0),
    lookat=(0.5, 0.5, -1.0),
    fov=70,
    GUI=True,
)

########################## entities ##########################
plane = scene.add_entity(
    gs.morphs.Plane(),
)
cube = scene.add_entity(
    gs.morphs.Box(
        size=(0.04, 0.04, 0.04),
        pos=(0.65, 0.3, 0.02 + 0.42),
    )
)
desk = scene.add_entity(
    morph=gs.morphs.Mesh(
        file="/dataset/mesh/desk/MyScriptedDesk.obj",
        euler=(0., 0., 0.),
        scale=0.5,
        pos=(0.7, 0.3, 0.4),
        fixed=False,
        convexify=True
    ),
    material=gs.materials.Rigid(
        rho=3000
    )
)
franka = scene.add_entity(
    gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"),
)
franka_2 = scene.add_entity(
    gs.morphs.MJCF(
        file="xml/franka_emika_panda/panda.xml",
        pos=(0.0, 0.6, 0.0),
        quat=(0.7071067811865475, 0.0, 0.0, 0.7071067811865476) #(0.7071067811865476, 0.0, 0.0, 0.7071067811865475) # (0, 0.0, 0.0, 1.0) # (0.7071067811865476, 0.0, 0.0, 0.7071067811865475)
    ),
)
########################## build ##########################
scene.build()
scene.step()
input()


rgb = cam.render(rgb=True)
cam.start_recording()

franka1_state = []
franka1_action = []
franka2_state = []
franka2_action = []

motors_dof = np.arange(7)
fingers_dof = np.arange(7, 9)

motors_dof_2 = np.arange(7)
fingers_dof_2 = np.arange(7, 9)

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

franka_2.set_dofs_kp(
    np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
)
franka_2.set_dofs_kv(
    np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
)
franka_2.set_dofs_force_range(
    np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
    np.array([87, 87, 87, 87, 12, 12, 12, 100, 100]),
)

end_effector = franka.get_link("hand")
end_effector_2 = franka_2.get_link("hand")

# move to pre-grasp pose
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.3, 0.25 + 0.4]),
    quat=np.array([0, 1, 0, 0]),
)
# gripper open pos
qpos[-2:] = 0.04
path = franka.plan_path(
    qpos_goal=qpos,
    num_waypoints=200,  # 2s duration
)
# path_debug = scene.draw_debug_path(path, franka)

qpos_2 = franka_2.inverse_kinematics(
    link=end_effector_2,
    pos=np.array([0.65, 0.8, 0.58 + 0.2]),
    quat=np.array([0, 1, 0, 0]),
)
# gripper open pos
qpos_2[-2:] = 0.04
path_2 = franka_2.plan_path(
    qpos_goal=qpos_2,
    num_waypoints=200,  # 2s duration
)

# execute the planned path
for wp_1, wp_2 in zip(path, path_2):
    franka.control_dofs_position(wp_1)
    franka_2.control_dofs_position(wp_2)
    scene.step()
    cam.render()
    franka1_state.append(franka.get_dofs_position(motors_dof))
    franka2_state.append(franka_2.get_dofs_position(motors_dof_2))
    franka1_action.append(franka.get_dofs_velocity(motors_dof))
    franka2_action.append(franka_2.get_dofs_velocity(motors_dof_2))
    

# remove the drawn path
# scene.clear_debug_object(path_debug)

# allow robot to reach the last waypoint
for i in range(100):
    scene.step()
    cam.render()
    franka1_state.append(franka.get_dofs_position(motors_dof))
    franka2_state.append(franka_2.get_dofs_position(motors_dof_2))
    franka1_action.append(franka.get_dofs_velocity(motors_dof))
    franka2_action.append(franka_2.get_dofs_velocity(motors_dof_2))

# reach
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.3, 0.130 + 0.4]),
    quat=np.array([0, 1, 0, 0]),
)
print(qpos)
franka.control_dofs_position(qpos[:-2], motors_dof)
for i in range(100):
    scene.step()
    cam.render()
    franka1_state.append(franka.get_dofs_position(motors_dof))
    franka2_state.append(franka_2.get_dofs_position(motors_dof_2))
    franka1_action.append(franka.get_dofs_velocity(motors_dof))
    franka2_action.append(franka_2.get_dofs_velocity(motors_dof_2))

# grasp
franka.control_dofs_position(qpos[:-2], motors_dof)
franka.control_dofs_force(np.array([-1.0, -1.0]), fingers_dof)

for i in range(100):
    scene.step()
    cam.render()
    franka1_state.append(franka.get_dofs_position(motors_dof))
    franka2_state.append(franka_2.get_dofs_position(motors_dof_2))
    franka1_action.append(franka.get_dofs_velocity(motors_dof))
    franka2_action.append(franka_2.get_dofs_velocity(motors_dof_2))

# lift
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.3, 0.58 + 0.2]),
    quat=np.array([0.0, 1.0, 0.0, 0.0]), 
)
print(qpos)
franka.control_dofs_position(qpos[:-2], motors_dof)

for i in range(200):
    scene.step()
    cam.render()
    franka1_state.append(franka.get_dofs_position(motors_dof))
    franka2_state.append(franka_2.get_dofs_position(motors_dof_2))
    franka1_action.append(franka.get_dofs_velocity(motors_dof))
    franka2_action.append(franka_2.get_dofs_velocity(motors_dof_2))

# turn to handle cube
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.3, 0.58 + 0.2]),
    quat=np.array([0.707106, -0.707106, 0.0, 0.0]), # -90, 0, 0
)
print(qpos)
franka.control_dofs_position(qpos[:-2], motors_dof)
for i in range(200):
    scene.step()
    cam.render()
    franka1_state.append(franka.get_dofs_position(motors_dof))
    franka2_state.append(franka_2.get_dofs_position(motors_dof_2))
    franka1_action.append(franka.get_dofs_velocity(motors_dof))
    franka2_action.append(franka_2.get_dofs_velocity(motors_dof_2))

# turn to franka 1
qpos_2 = franka_2.inverse_kinematics(
    link=end_effector_2,
    pos=np.array([0.65, 0.7, 0.58 + 0.2]),
    quat=np.array([0.5, 0.5, 0.5, -0.5]), # 90, 0, -90
)
print(qpos_2)
franka_2.control_dofs_position(qpos_2[:-2], motors_dof_2)
for i in range(200):
    scene.step()
    cam.render()
    franka1_state.append(franka.get_dofs_position(motors_dof))
    franka2_state.append(franka_2.get_dofs_position(motors_dof_2))
    franka1_action.append(franka.get_dofs_velocity(motors_dof))
    franka2_action.append(franka_2.get_dofs_velocity(motors_dof_2))

# reach to franka 1
qpos_2 = franka_2.inverse_kinematics(
    link=end_effector_2,
    pos=np.array([0.65, 0.52, 0.58 + 0.2]),
    quat=np.array([0.5, 0.5, 0.5, -0.5]),
)
print(qpos_2)
qpos_2[-2:] = 0.04
path_2 = franka_2.plan_path(
    qpos_goal=qpos_2,
    num_waypoints=200
)
for wp2 in path_2:
    franka_2.control_dofs_position(wp2)
    scene.step()
    cam.render()
    franka1_state.append(franka.get_dofs_position(motors_dof))
    franka2_state.append(franka_2.get_dofs_position(motors_dof_2))
    franka1_action.append(franka.get_dofs_velocity(motors_dof))
    franka2_action.append(franka_2.get_dofs_velocity(motors_dof_2))

for _ in range(100):
    scene.step()
    
# grasp cube 2
franka_2.control_dofs_position(qpos_2[:-2], motors_dof_2)
franka_2.control_dofs_force(np.array([-2.0, -2.0]), fingers_dof_2)
for i in range(100):
    scene.step()
    cam.render()
    franka1_state.append(franka.get_dofs_position(motors_dof))
    franka2_state.append(franka_2.get_dofs_position(motors_dof_2))
    franka1_action.append(franka.get_dofs_velocity(motors_dof))
    franka2_action.append(franka_2.get_dofs_velocity(motors_dof_2))

for _ in range(100):
    scene.step()

# release cube 1
franka.control_dofs_position(qpos[:-2], motors_dof)
franka.control_dofs_force(np.array([1.0, 1.0]), fingers_dof)
for i in range(100):
    scene.step()
    cam.render()
    franka1_state.append(franka.get_dofs_position(motors_dof))
    franka2_state.append(franka_2.get_dofs_position(motors_dof_2))
    franka1_action.append(franka.get_dofs_velocity(motors_dof))
    franka2_action.append(franka_2.get_dofs_velocity(motors_dof_2))

# return franka hand 1
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.3, 0.58 + 0.2]),
    quat=np.array([0, 1, 0, 0]),
)
print(qpos)
franka.control_dofs_position(qpos[:-2], motors_dof)

qpos_2 = franka_2.inverse_kinematics(
    link=end_effector_2,
    pos=np.array([0.65, 0.6, 0.58 + 0.2]),
    quat=np.array([0.5, 0.5, 0.5, -0.5]),
)
print(qpos_2)
franka_2.control_dofs_position(qpos_2[:-2], motors_dof_2)

for i in range(200):
    scene.step()
    cam.render()
    franka1_state.append(franka.get_dofs_position(motors_dof))
    franka2_state.append(franka_2.get_dofs_position(motors_dof_2))
    franka1_action.append(franka.get_dofs_velocity(motors_dof))
    franka2_action.append(franka_2.get_dofs_velocity(motors_dof_2))

sa_data = {
    "franka1_state": franka1_state,
    "franka1_action": franka1_action,
    "franka2_state": franka2_state,
    "franka2_action": franka2_action,
}

f = open("./state_action_data.pkl", "wb")
pickle.dump(sa_data, file=f)
f.close()
cam.stop_recording(save_to_filename="video.mp4", fps=60)