import pybullet as p
import time
import pybullet_data 
import wheel_logic
import lidar
import pygame
import numpy as np
import math

WINDOW_HEIGHT = 480
WINDOW_WIDTH = 480
RADIANS_CONST = 57
pygame.init()
WINDOW = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
clock = pygame.time.Clock()
"""
So to change the coordinate system from left top center to
mid center
""" 
WINDOW_WIDTH_CENTER = WINDOW_WIDTH / 2
WINDOW_HEIGHT_CENTER = WINDOW_HEIGHT / 2
"""
rectangle centering logic

"""
def center_circle(circle_center_x, circle_center_y):
    new_center_x = circle_center_x + WINDOW_WIDTH_CENTER
    new_center_y = circle_center_y + WINDOW_HEIGHT_CENTER

    return (new_center_x, new_center_y)


def center_rect(x_pos, y_pos, width, height):
    """
    I give in the wanted coodinats, I transform it to fit the pygame
    window
    """
    new_x_pos = ( x_pos + WINDOW_WIDTH_CENTER ) - ( width / 2 ) 
    new_y_pos = ( y_pos + WINDOW_HEIGHT_CENTER ) - ( height / 2 )

    return [new_x_pos, new_y_pos, width, height]



WHEEL_DIST_FROM_ROBOT_CENTER = 0.2 #in metres
LIDAR_RANGE = 10.0 
NUM_RAYS = 36
LIDAR_HEIGHT = 0.2  

#env set up(gravity, ground) and robot loading 
phyCl = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
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

link_state = p.getLinkState(krawlBot, 0)
robot_base_pose = link_state[0]
robot_base_orn = link_state[1]
robot_base_yaw = p.getEulerFromQuaternion(robot_base_orn)[2]


# cube size (half extents)
half_size = [0.5, 0.5, 0.5]

# create collision and visual shape
collision = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=half_size
)

visual = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=half_size,
    rgbaColor=[1, 0, 0, 1]
)

# create the cube body
cube_id = p.createMultiBody(
    baseMass=1.0,
    baseCollisionShapeIndex=collision,
    baseVisualShapeIndex=visual,
    basePosition=[1, 1, 0]
)
    
collision = p.createCollisionShape(
    shapeType=p.GEOM_BOX,
    halfExtents=half_size
)

visual = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=half_size,
    rgbaColor=[1, 0, 0, 1]
)

# create the cube body
cube_id = p.createMultiBody(
    baseMass=1.0,
    baseCollisionShapeIndex=collision,
    baseVisualShapeIndex=visual,
    basePosition=[1, -2, 0]
)


while True:
    

    base_pos, base_orn = p.getBasePositionAndOrientation(krawlBot)
    euler = p.getEulerFromQuaternion(base_orn)
    base_z_rot = euler[2]
    robot_lidar.project_lidar(base_pos, base_orn, start_time)
    ray_hit_location = robot_lidar.object_detection()
    # print(ray_hit_location)
    # robot_lidar.render_rays()

    # print(euler)

    # # #wheel movement logic
    wheel.forward_movement()
    # # wheel.right_turn(euler)
    #
    #turning test
    print(math.ceil(time.perf_counter() - start_time))
    if (time.perf_counter() - start_time) >= 2:
        wheel.turn(20)

    if (time.perf_counter() - start_time) >= 7:
        wheel.restore_vel()

    # print(wheel.right_wheel_vel)

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


    for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

    WINDOW.fill("purple")

    rect_coordinates = center_rect(base_pos[1]*100, base_pos[0]*100, 30, 50)
    rect = pygame.Rect(rect_coordinates)
    rect_surf = pygame.Surface((rect.width, rect.height), pygame.SRCALPHA)
    pygame.draw.rect(rect_surf, "red", (0, 0, rect.width, rect.height))
    rotated_surf = pygame.transform.rotate(rect_surf, np.rad2deg(base_z_rot))
    rotated_rect = rotated_surf.get_rect(center=rect.center)

    if ray_hit_location:
        for ray_hit in ray_hit_location:
            circle_center = center_circle(ray_hit[1]*100, ray_hit[0]*100)
            pygame.draw.circle(WINDOW, "black", circle_center, 1)


    WINDOW.blit(rotated_surf, rotated_rect)
        



    pygame.display.flip()
    clock.tick(60)

p.disconnect()
pygame.quit()
