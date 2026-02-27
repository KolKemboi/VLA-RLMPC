import pybullet as p

class WheelLogic:
    def __init__ (self, robot, joint_data, RIGHT_WHEEL_VEL = 5, LEFT_WHEEL_VEL = 5):
        self.joint_data = joint_data
        self.right_wheel_vel = RIGHT_WHEEL_VEL
        self.left_wheel_vel = LEFT_WHEEL_VEL
        self.robot = robot

        self.front_left_wheel = self.joint_data.get("front_left_wheel_joint")
        self.rear_left_wheel = self.joint_data.get("rear_left_wheel_joint")
        self.front_right_wheel = self.joint_data.get("front_right_wheel_joint")
        self.rear_right_wheel = self.joint_data.get("rear_right_wheel_joint")

    def set_debug_param(self):
        self.right_velocity_slider = p.addUserDebugParameter("RIGHT_VELOCITY_SLIDER", -10, 10, self.right_wheel_vel)
        self.left_velocity_slider = p.addUserDebugParameter("LEFT_VELOCITY_SLIDER", -10, 10, self.left_wheel_vel)

    def forward_movement(self):
        # self.right_wheel_vel = p.readUserDebugParameter(self.right_velocity_slider)
        # self.left_wheel_vel = p.readUserDebugParameter(self.left_velocity_slider)

        p.setJointMotorControl2(self.robot, self.front_left_wheel, p.VELOCITY_CONTROL, targetVelocity=self.left_wheel_vel, force=100)
        p.setJointMotorControl2(self.robot, self.rear_left_wheel, p.VELOCITY_CONTROL, targetVelocity=self.left_wheel_vel, force=100)

        p.setJointMotorControl2(self.robot, self.front_right_wheel, p.VELOCITY_CONTROL, targetVelocity=self.right_wheel_vel, force=100)
        p.setJointMotorControl2(self.robot, self.rear_right_wheel, p.VELOCITY_CONTROL, targetVelocity=self.right_wheel_vel, force=100)

    def turn(self, turn_angle):
        """
        for a right turn, I need to slowly add a value m until pi/2
        using velocity, I need to reduce the right side n amount until
        the difference between z orien and last orien is pi/2
        """
        self.right_wheel_vel = 0
        # print(turn_angle)
        pass
    def restore_vel(self):
        self.right_wheel_vel = 5
        self.left_wheel_vel = 5
