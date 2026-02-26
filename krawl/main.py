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

for i in range(num_joints):
    joint_info = p.getJointInfo(krawlBot, i)
    joint_name = joint_info[1].decode("utf-8")
    joint_type = joint_info[2]

    #get wheel joints
    if "wheel" in joint_name.lower() and joint_type != p.JOINT_FIXED:
        if "left" in joint_name.lower():
            left_wheel_joints.append(i)
        elif "right" in joint_name.lower():
            right_wheel_joints.append(i)
        print(joint_name)
        p.setJointMotorControl2(krawlBot, i, p.VELOCITY_CONTROL, targetVelocity = 0, force = 0)

    elif "gripper" in joint_name.lower() and joint_type != p.JOINT_FIXED:
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

        gripper_joints.append(slider)


    elif "arm" in joint_name.lower() and joint_type != p.JOINT_FIXED:
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
        arm_joints.append(slider)

# left_wheel = wheel_joints[0]
# right_wheel = wheel_joints[1]
#
# left_slider = p.addUserDebugParameter("Left Wheel Vel", -10, 10, 5)  # Default 5 rad/s
# right_slider = p.addUserDebugParameter("Right Wheel Vel", -10, 10, 5)  # Default 5 rad/s

while True:
    # left_vel = p.readUserDebugParameter(left_slider)
    # right_vel = p.readUserDebugParameter(right_slider)
    # 
    # # Apply velocity control to wheels
    # p.setJointMotorControl2(krawlBot, left_wheel, p.VELOCITY_CONTROL, 
    #                        targetVelocity=left_vel, force=100)
    # p.setJointMotorControl2(krawlBot, right_wheel, p.VELOCITY_CONTROL, 
    #                        targetVelocity=right_vel, force=100)
    #
    # # Step simulation
    # p.stepSimulation()
    #
    # # Get robot position for feedback
    # pos, orn = p.getBasePositionAndOrientation(krawlBot)
    # euler = p.getEulerFromQuaternion(orn)
    #
    for i, controller in enumerate(arm_joints):
        target_pos = p.readUserDebugParameter(controller)
        p.setJointMotorControl2(
                bodyUniqueId=krawlBot,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_pos,
                force=500
                )
    for i, controller in enumerate(gripper_joints):
        target_pos = p.readUserDebugParameter(controller)
        p.setJointMotorControl2(
                bodyUniqueId=krawlBot,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_pos,
                force=500
                )

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()

