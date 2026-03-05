import pybullet as p
import time
import pybullet_data 
import wheel_logic
import lidar
import pygame
import math
import bullet_map
import numpy as np
import camera
import robot_povs

WINDOW_HEIGHT = 640
WINDOW_WIDTH = 640
pygame.init()
WINDOW = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
clock = pygame.time.Clock()

env_map = bullet_map.map(WINDOW, WINDOW_WIDTH, WINDOW_HEIGHT)


WHEEL_DIST_FROM_ROBOT_CENTER = 0.2 #in metres
LIDAR_RANGE = 10.0 
NUM_RAYS = 36
LIDAR_HEIGHT = 0.2  
ROBOT_WIDTH = 30
ROBOT_LENGTH = 50

#env set up(gravity, ground) and robot loading 
phyCl = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
obj_file_path = "../../../test.obj"

# Create a visual shape from the .obj file
visual_shape_id = p.createVisualShape(shapeType=p.GEOM_MESH,
                                      fileName=obj_file_path,
                                      meshScale=[1, 1, 1])

# Create a collision shape from the .obj file (for non-concave or decomposed meshes)
collision_shape_id = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName=obj_file_path,
                                          meshScale=[1, 1, 1])

# Create a multi-body object using the shapes
# useBaseCollisionShapeIndex and useBaseVisualShapeIndex refer to the IDs created above
# mass of 0 makes it a static object
object_id = p.createMultiBody(baseMass=0,
                              baseCollisionShapeIndex=collision_shape_id,
                              baseVisualShapeIndex=visual_shape_id,
                              basePosition=[0, 0, 0])
startPos = [0, 0, 0]
startOr = p.getQuaternionFromEuler([0, 0, 0])
krawlBot = p.loadURDF("./krawl/urdf/krawl.urdf", startPos, startOr)
num_joints =  p.getNumJoints(krawlBot) 

#transparency
def viewThroughRobot(joint_count):
    for link_index in range(-1, joint_count):  # -1 for base
        p.changeVisualShape(
            krawlBot, 
            link_index,
            rgbaColor=[0.3, 0.5, 0.8, 0.3]  # RGBA with alpha=0.3 (transparent)
        )
#uncomment below for transparency
# viewThroughRobot(num_joints)


joint_name_idx = dict()
arm_joints = dict()
gripper_joints = dict()

for i in range(num_joints):
    joint_info = p.getJointInfo(krawlBot, i)
    joint_name = joint_info[1].decode("utf-8")
    joint_type = joint_info[2]
    
    if joint_type != p.JOINT_FIXED:
        joint_name_idx[joint_name] = i

wheel = wheel_logic.WheelLogic(krawlBot, joint_name_idx)
wheel.set_debug_param()

for joint_name, idx in joint_name_idx.items():
    # print(joint_name, idx)
    if "arm" in joint_name.lower():
        lower_lim = -3.14
        upper_lim = 3.14
        start_pos = (lower_lim + upper_lim) / 2
        joint_debug = p.addUserDebugParameter(
                joint_name, lower_lim, upper_lim, start_pos
                )
        arm_joints[joint_name] = joint_debug

    elif "gripper" in joint_name.lower():
        joint_info = p.getJointInfo(krawlBot, idx)
        lower_lim = joint_info[8]
        upper_lim = joint_info[9]
        start_pos = (lower_lim + upper_lim) / 2
        joint_debug = p.addUserDebugParameter(
                joint_name, lower_lim, upper_lim, start_pos
                )
        gripper_joints[joint_name] = joint_debug



start_time = time.perf_counter()

#for slam
robot_lidar = lidar.LidarSensor(krawlBot)



# cube size (half extents)
# half_size = [0.5, 0.5, 0.5]
#
# # create collision and visual shape
# collision = p.createCollisionShape(
#     shapeType=p.GEOM_BOX,
#     halfExtents=half_size
# )
#
# visual = p.createVisualShape(
#     shapeType=p.GEOM_BOX,
#     halfExtents=half_size,
#     rgbaColor=[1, 0, 0, 1]
# )
#
# # create the cube body
# cube_id = p.createMultiBody(
#     baseMass=1.0,
#     baseCollisionShapeIndex=collision,
#     baseVisualShapeIndex=visual,
#     basePosition=[1, 1, 0]
# )
    

####CAMERA STUFF
robot_camera = camera.Camera()
robot_pov = robot_povs.renderImages()

while True:
    """
    gets the robot current tranform
    
    """
    base_pos, base_orn = p.getBasePositionAndOrientation(krawlBot)
    euler = p.getEulerFromQuaternion(base_orn)
    base_z_rot = euler[2]
    rot_matrix = np.array(p.getMatrixFromQuaternion(base_orn)).reshape(3, 3)

    """
    gets and displays the four views of the robot for debug 
    """
    forward_image = robot_camera.forward(base_pos, base_orn)
    back_image = robot_camera.backward(base_pos, base_orn)
    left_image = robot_camera.left(base_pos, base_orn)
    right_image = robot_camera.right(base_pos, base_orn)
    robot_pov.grid(forward_image, back_image, left_image, right_image)

    """
    sets up lidar and displays it for debug
    """
    robot_lidar.project_lidar(base_pos, base_orn, start_time)
    ray_hit_location = robot_lidar.object_detection()

    for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

    WINDOW.fill("purple")
    env_map.draw_robot_frame(base_pos, ROBOT_WIDTH,ROBOT_LENGTH, base_z_rot)
    env_map.render_lidar(ray_hit_location)

    pygame.display.flip()
    clock.tick(60)

    # #wheel movement logic
    wheel.forward_movement()
    # turning test
    if (time.perf_counter() - start_time) >= 2:
        wheel.turn(20)

    if (time.perf_counter() - start_time) >= 7:
        wheel.restore_vel()


    for joint_name, idx in arm_joints.items():
        target_pos = p.readUserDebugParameter(idx)
        p.setJointMotorControl2(
                bodyUniqueId = krawlBot,
                jointIndex = joint_name_idx.get(joint_name),
                controlMode = p.POSITION_CONTROL,
                targetPosition = target_pos,
                force = 100
                )

    for joint_name, idx in gripper_joints.items():
        target_pos = p.readUserDebugParameter(idx)
        p.setJointMotorControl2(
                bodyUniqueId = krawlBot,
                jointIndex = joint_name_idx.get(joint_name),
                controlMode = p.POSITION_CONTROL,
                targetPosition = target_pos,
                force = 100
                )


    p.stepSimulation()
    time.sleep(1./240.)



p.disconnect()
pygame.quit()
