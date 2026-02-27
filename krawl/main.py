import pybullet as p
import time
import pybullet_data 


phyCl = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
startPos = [0, 0, 0]
startOr = p.getQuaternionFromEuler([0, 0, 0])
krawlBot = p.loadURDF("./krawl/urdf/krawl.urdf", startPos, startOr)
print( p.getNumJoints(krawlBot) )
num_joints =  p.getNumJoints(krawlBot) 


joint_Cont = []

#transparency
def viewThroughRobot(joint_count):
    for link_index in range(-1, joint_count):  # -1 for base
        p.changeVisualShape(
            krawlBot, 
            link_index,
            rgbaColor=[0.3, 0.5, 0.8, 0.3]  # RGBA with alpha=0.3 (transparent)
        )

arm_joints = []
gripper_joints = []
left_wheel_joints = []
right_wheel_joints = []

joint_name_idx = dict()

for i in range(num_joints):
    joint_info = p.getJointInfo(krawlBot, i)
    joint_name = joint_info[1].decode("utf-8")
    joint_type = joint_info[2]


    if joint_type != p.JOINT_FIXED:
        print(joint_name)
        lower_lim = joint_info[8]
        upper_lim = joint_info[9]

        if lower_lim < upper_lim:
            start_pos = (lower_lim + upper_lim) / 2
        else:
            lower_lim, upper_lim = -3.14, 3.14
            start_pos = 0

        slider = p.addUserDebugParameter(
                paramName = joint_name,
                rangeMin = lower_lim,
                rangeMax= upper_lim,
                startValue=start_pos
                )
        joint_name_idx[joint_name] = slider

"""
wheel joint extraction
using a const velocity for rear and front for easy diff nav
"""
RIGHT_WHEEL_VEL = 5
LEFT_WHEEL_VEL = 5

FRONT_LEFT_WHEEL = joint_name_idx.get("front_left_wheel_joint")
REAR_LEFT_WHEEL = joint_name_idx.get("rear_left_wheel_joint")
FRONT_RIGHT_WHEEL = joint_name_idx.get("front_right_wheel_joint")
REAR_RIGHT_WHEEL = joint_name_idx.get("rear_right_wheel_joint")

RIGHT_VELOCITY_SLIDER = p.addUserDebugParameter("RIGHT_VELOCITY_SLIDER", -10, 10, RIGHT_WHEEL_VEL)
LEFT_VELOCITY_SLIDER = p.addUserDebugParameter("LEFT_VELOCITY_SLIDER", -10, 10, LEFT_WHEEL_VEL)

while True:
    #for slam
    pos, orn = p.getBasePositionAndOrientation(krawlBot)
    euler = p.getEulerFromQuaternion(orn)

    #wheel movement logic
    RIGHT_WHEEL_VEL = p.readUserDebugParameter(RIGHT_VELOCITY_SLIDER)
    LEFT_WHEEL_VEL = p.readUserDebugParameter(LEFT_VELOCITY_SLIDER)

    p.setJointMotorControl2(krawlBot, FRONT_LEFT_WHEEL, p.VELOCITY_CONTROL, targetVelocity=LEFT_WHEEL_VEL, force=100)
    p.setJointMotorControl2(krawlBot, REAR_LEFT_WHEEL, p.VELOCITY_CONTROL, targetVelocity=LEFT_WHEEL_VEL, force=100)

    p.setJointMotorControl2(krawlBot, FRONT_RIGHT_WHEEL, p.VELOCITY_CONTROL, targetVelocity=RIGHT_WHEEL_VEL, force=100)
    p.setJointMotorControl2(krawlBot, REAR_RIGHT_WHEEL, p.VELOCITY_CONTROL, targetVelocity=RIGHT_WHEEL_VEL, force=100)
    # print(pos, orn)
    #
    # for i, controller in enumerate(joint_Cont):
    #     
    #     target_pos = p.readUserDebugParameter(controller)
    #     p.setJointMotorControl2(
    #             bodyUniqueId=krawlBot,
    #             jointIndex=i,
    #             controlMode=p.POSITION_CONTROL,
    #             targetPosition=target_pos,
    #             force=500
    #             )
    # for i, controller in enumerate(gripper_joints):
    #     target_pos = p.readUserDebugParameter(controller)
    #     p.setJointMotorControl2(
    #             bodyUniqueId=krawlBot,
    #             jointIndex=i,
    #             controlMode=p.POSITION_CONTROL,
    #             targetPosition=target_pos,
    #             force=500
    #             )
    # for joint_name, idx in joint_name_idx.items():
    #     if "wheel" in joint_name.lower():
    #         target_pos = p.readUserDebugParameter(idx)
    #
    #         p.setJointMotorControl2(
    #                 bodyUniqueId=krawlBot,
    #                 jointIndex=idx,
    #                 controlMode=p.VELOCITY_CONTROL,
    #                 targetPosition=target_pos,
    #                 force=500
    #                 )




    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()

