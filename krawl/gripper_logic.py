import pybullet as p

class GripperLogic:
    def __init__ (self, robot, joint_data, gripper_joints):
        """
        current_state 
        0 -> open
        1 -> closed
        """
        self.current_state = 0 #stores if open or closed
        self.robot = robot
        self.joint_data = joint_data
        self.gripper_joints = gripper_joints

    def set_debug_param(self):
        pass

    def actuate_open(self):
        if self.current_state:
            #close the arm
            self.current_state = 0
        pass

    def actuate_closed(self):
        if not self.current_state:
            #close the arm
            self.current_state = 1
        pass
