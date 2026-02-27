import pybullet as p
import time
import pybullet_data 

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

#
# """
# wheel joint extraction
# using a const velocity for rear and front for easy diff nav
# """
RIGHT_WHEEL_VEL = 1
LEFT_WHEEL_VEL = 1

FRONT_LEFT_WHEEL = joint_name_idx.get("front_left_wheel_joint")
REAR_LEFT_WHEEL = joint_name_idx.get("rear_left_wheel_joint")
FRONT_RIGHT_WHEEL = joint_name_idx.get("front_right_wheel_joint")
REAR_RIGHT_WHEEL = joint_name_idx.get("rear_right_wheel_joint")

RIGHT_VELOCITY_SLIDER = p.addUserDebugParameter("RIGHT_VELOCITY_SLIDER", -10, 10, RIGHT_WHEEL_VEL)
LEFT_VELOCITY_SLIDER = p.addUserDebugParameter("LEFT_VELOCITY_SLIDER", -10, 10, LEFT_WHEEL_VEL)


for joint_name, idx in joint_name_idx.items():
    print(joint_name, idx)
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




while True:
    #for slam
    pos, orn = p.getBasePositionAndOrientation(krawlBot)
    euler = p.getEulerFromQuaternion(orn)

    # #wheel movement logic
    RIGHT_WHEEL_VEL = p.readUserDebugParameter(RIGHT_VELOCITY_SLIDER)
    LEFT_WHEEL_VEL = p.readUserDebugParameter(LEFT_VELOCITY_SLIDER)

    p.setJointMotorControl2(krawlBot, FRONT_LEFT_WHEEL, p.VELOCITY_CONTROL, targetVelocity=LEFT_WHEEL_VEL, force=100)
    p.setJointMotorControl2(krawlBot, REAR_LEFT_WHEEL, p.VELOCITY_CONTROL, targetVelocity=LEFT_WHEEL_VEL, force=100)

    p.setJointMotorControl2(krawlBot, FRONT_RIGHT_WHEEL, p.VELOCITY_CONTROL, targetVelocity=RIGHT_WHEEL_VEL, force=100)
    p.setJointMotorControl2(krawlBot, REAR_RIGHT_WHEEL, p.VELOCITY_CONTROL, targetVelocity=RIGHT_WHEEL_VEL, force=100)

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

