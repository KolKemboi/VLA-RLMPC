import pygame
import numpy as np


class map:
    def __init__(self, WINDOW, WINDOW_WIDTH, WINDOW_HEIGHT):
        self.window_width_center = WINDOW_WIDTH / 2
        self.window_height_center = WINDOW_HEIGHT / 2
        self.window = WINDOW

    """
    transforms points in pybullet coordinates to pygame coordinates
    """
    def center_circle(self, circle_center_x, circle_center_y):
        new_circle_center_x = circle_center_x + self.window_width_center
        new_circle_center_y = circle_center_y + self.window_height_center

        return (new_circle_center_x, new_circle_center_y)

    """
    renders ray hits from pybullet onto the pygame surface
    """
    def render_lidar(self, rays_hits):
        if rays_hits:
            for ray_hit in rays_hits:
                circle_center = self.center_circle(ray_hit[1]*100, ray_hit[0]*100)
                pygame.draw.circle(self.window, "black", circle_center, 4)
    
    """
    transforms rectangle coordinated from pybullet to pygame coordinates
    """
    def center_rectangle(self, x_pos, y_pos, width, height):
        new_x_pos = (x_pos + self.window_width_center) - (width / 2)
        new_y_pos = (y_pos + self.window_height_center) - (height / 2)

        return [new_x_pos, new_y_pos, width, height]

    """
    renders the robot base frame as a red rectangle in pygame maintaining trajectory
    """
    def draw_robot_frame(self, robot_pos, robot_width, robot_length, robot_yaw):
        robot_cood = self.center_rectangle(robot_pos[1]*100, robot_pos[0]*100, robot_width, robot_length)
        robot_rect = pygame.Rect(robot_cood)
        robot_surface = pygame.Surface((robot_rect.width, robot_rect.height), pygame.SRCALPHA)
        pygame.draw.rect(robot_surface, "red", (0, 0, robot_rect.width, robot_rect.height))
        align_rotation = pygame.transform.rotate(robot_surface, np.rad2deg(robot_yaw))
        robot_new_transform = align_rotation.get_rect(center=robot_rect.center)

        self.window.blit(align_rotation, robot_new_transform)
