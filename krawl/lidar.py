import pybullet as p
import numpy as np
import time
import math

class LidarSensor:
    """
    on_class init, it creates basic lidar constants
    also, keep the num rays minimum, 36 is chosen for every 10deg interval.
    a high number of rays slows down the simulation
    """
    def __init__(self,robot, LIDAR_RANGE = 59.0, NUM_RAYS = 36, LIDAR_HEIGHT = 0.05) -> None:
        #all the distances are in Metres
        self.robot = robot
        self.lidar_range = LIDAR_RANGE
        self.num_rays = NUM_RAYS
        self.lidar_height = LIDAR_HEIGHT
        self.ray_origin = list()
        self.ray_destination = list()
        self.results = list()
    """
    projects the lidar from lidar origin outwards and collects any hit
    and hits are stored in self.results, the len of which can be at most 
    the number of rays
    """
    def project_lidar(self, robot_pos, robot_orn, start_time):
        self.ray_origin.clear()
        self.ray_destination.clear()
        self.robot_pos = robot_pos
        self.robot_orn = robot_orn
        self.robot_yaw = p.getEulerFromQuaternion(self.robot_orn)[2]

        if ( math.ceil(time.perf_counter() - start_time) ) % 2 != 0:
            for i in range(self.num_rays):
                angle = self.robot_yaw + (2 * np.pi * i / self.num_rays)
                origin_point = [
                        self.robot_pos[0],
                        self.robot_pos[1],
                        self.robot_pos[2] + self.lidar_height,
                        ]
                destination_point = [
                        self.robot_pos[0] + self.lidar_range * np.cos(angle),
                        self.robot_pos[1] + self.lidar_range * np.sin(angle),
                        self.robot_pos[2] + self.lidar_height
                        ]
                self.ray_origin.append(origin_point)
                self.ray_destination.append(destination_point)

            self.results = p.rayTestBatch(self.ray_origin, self.ray_destination)

    """
    grabs the results and checks if it is a self hit or if there is nothing on the
    lidar
    otherwise, it returns the hit location and distance. 
    this is used for pygame viz and future data for l4acados
    """
    def object_detection(self):
        self.hit_locations = []
        for result in self.results:
            object_id = result[0]

            if object_id == -1:
                continue
            if object_id == self.robot:
                continue

            hit_position = result[3]
            distance = result[2]
            self.hit_locations.append(hit_position)
        return self.hit_locations
    
    """
    this is to render the rays, this is for debug purposes, and might slow down the simulation if called
    """
    def render_rays(self):
        for i, result in enumerate(self.results):
            object_id = result[0]
            hit_fraction = result[2]
            self.hit_position_x = self.ray_origin[i][0] + hit_fraction * (self.ray_destination[i][0] - self.ray_origin[i][0])
            self.hit_position_y = self.ray_origin[i][1] + hit_fraction * (self.ray_destination[i][1] - self.ray_origin[i][1])
            self.hit_position_z = self.ray_origin[i][2]
            self.hit_position_xyz = [self.hit_position_x, self.hit_position_y, self.hit_position_z]
            distance = hit_fraction * self.lidar_range

            p.addUserDebugLine(self.ray_origin[i], self.hit_position_xyz, [1, 0, 0], 1, 0.1)
