import mujoco as mj
from mujoco.glfw import glfw
import numpy as np


xml_path = '/home/roboticme/disassembly_simulation/MuJoCo_simulation/mujoco_3.0/model/abb_irb_6640/abb_irb_6640.xml' #xml file (assumes this is in the same folder as this file)
print_camera_config = 0 #set to 1 to print camera config!command -v ffmpeg >/dev/null || (apt update && apt install -y ffmpeg)
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.1
    pass

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)
    if act == glfw.PRESS and key == glfw.KEY_1:
        mj.mjv_initGeom()
    

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)
    
# MuJoCo data structures
xml = '/home/roboticme/disassembly_simulation/MuJoCo_simulation/mujoco_3.0/model/abb_irb_6640/abb_irb_6640.xml'
model = mj.MjModel.from_xml_path(xml)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                   # Abstract camera
opt = mj.MjvOption()                   # visualization options
geom = mj.MjvGeom()                    # Geom to render

mj.mj_forward(model, data)

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)

scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
#initialize the controller here. This function is called once, in the beginning
cam.azimuth = -110.000000 ; cam.elevation = -20.0000; cam.distance =  6.0000
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])

#initialize the controller
init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)

# Define joint positions for different poses of the robot
p_initial_grip = [0.89, 0.24, 1.22, 0.942, -1.44, 0.000, 0.000]
p_initial_grip_away = [1.2, 0.24, 1.22, 0.942, -1.44, 0.000, 0.000]
p_safe_away = [1.571, -0.309, 0.546, 0.00, -0.293, 0.000, 0.000]
p_zero = [0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000]
p_midscrew_up = [0.12, -0.123, 0.567, 0, -0.461, 0, 0.0]
p_mid_screw = [0.12, -0.0868, 0.589, 0, -0.503, 0, 0.00]
p_mid_unscrew = [0.12, -0.0868, 0.589, 0, -0.503, 0, 10.00]
p_topscrew_up = [-0.0095, 0.123, 0.371, 0, -0.503, 0, 0.00]
p_top_screw = [-0.012, 0.145, 0.349, 0, -0.482, 0, 0.00]
p_top_unscrew = [-0.012, 0.145, 0.349, 0, -0.482, 0, 10.00]

# Define velocities
normal_velocity, unscrewing_velocity, high_velocity = 100, 70, 40

# Define number of joints
n_joints = 7 

# Define function to generate joint trajectories
def generate_trajectory(p_from, p_to, N):
    return np.array([np.linspace(p_from[i], p_to[i], N) for i in range(n_joints)])

# Define trajectories
trajectories = [
    (p_initial_grip, p_initial_grip_away, normal_velocity),
    (p_initial_grip_away, p_safe_away, high_velocity),
    (p_safe_away, p_zero, high_velocity),
    (p_zero, p_midscrew_up, unscrewing_velocity),
    (p_midscrew_up, p_mid_screw, unscrewing_velocity),
    (p_mid_screw, p_mid_unscrew, unscrewing_velocity),
    (p_mid_unscrew, p_midscrew_up, unscrewing_velocity),
    (p_midscrew_up, p_topscrew_up, normal_velocity),
    (p_topscrew_up, p_top_screw, unscrewing_velocity),
    (p_top_screw, p_top_unscrew, unscrewing_velocity),
    (p_top_unscrew, p_topscrew_up, unscrewing_velocity),
    (p_topscrew_up, p_zero, normal_velocity),
    (p_zero, p_safe_away, high_velocity),
    (p_safe_away, p_initial_grip_away, high_velocity),
    (p_initial_grip_away, p_initial_grip, normal_velocity)
]


# Initialize variables
array_xpos0, array_xpos1, time, dt, i = [], [], 0, 0.001, 0
current_joint_trajectories_index = 0
simend = 2000 #simulation end time

# Generate trajectories for all poses 
all_joint_trajectories = []
for index, (p_from, p_to, velocity) in enumerate(trajectories, start=1):
    joint_trajectory = generate_trajectory(p_from, p_to, velocity)
    q0, q1, q2, q3, q4, q5, q6 = joint_trajectory  
    locals()[f'q0_{index}'] = q0
    locals()[f'q1_{index}'] = q1
    locals()[f'q2_{index}'] = q2
    locals()[f'q3_{index}'] = q3
    locals()[f'q4_{index}'] = q4
    locals()[f'q5_{index}'] = q5
    locals()[f'q6_{index}'] = q6
    all_joint_trajectories.append(joint_trajectory)


while time < simend:
         # Set the joint positions
        data.qpos[0] = all_joint_trajectories[current_joint_trajectories_index][0][i]
        data.qpos[1] = all_joint_trajectories[current_joint_trajectories_index][1][i]
        data.qpos[2] = all_joint_trajectories[current_joint_trajectories_index][2][i]
        data.qpos[3] = all_joint_trajectories[current_joint_trajectories_index][3][i]
        data.qpos[4] = all_joint_trajectories[current_joint_trajectories_index][4][i]
        data.qpos[5] = all_joint_trajectories[current_joint_trajectories_index][5][i]
        data.qpos[6] = all_joint_trajectories[current_joint_trajectories_index][6][i]
        
        array_xpos0.append(data.site_xpos[0])
        array_xpos1.append(data.site_xpos[1])
   
        mj.mj_forward(model, data)
 
        i += 1
        
        # Check if the current trajectory is completed
        speed_variable = all_joint_trajectories[current_joint_trajectories_index][2].size
        
        if i == speed_variable:
            print("step: ", current_joint_trajectories_index + 1)
            print("Current joint positions 6TH JOINT:", array_xpos0[-1])
            print("Current joint positions END EFFECTOR:", array_xpos1[-1])

            print("Switching to the next path")
            current_joint_trajectories_index += 1
            
            #current_joint_trajectories_index = (current_joint_trajectories_index + 1) % len(all_joint_trajectories)
            i = 0  # Reset the index for the new trajectory  
            if current_joint_trajectories_index == 15:
                current_joint_trajectories_index = 0
            
        
        # get framebuffer viewport
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

        #print camera configuration (help to initialize the view)
        if (print_camera_config==1):
            print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
            print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')
        
        # Update scene and render
        mj.mjv_updateScene(model, data, opt, None, cam,
                        mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(viewport, scene, context)

        # swap OpenGL buffers (blocking call due to v-sync)
        glfw.swap_buffers(window)
        
        # process pending GUI events, call GLFW callbacks
        glfw.poll_events()
    
glfw.terminate()

