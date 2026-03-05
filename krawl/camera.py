import pybullet as p
import numpy as np




class Camera:
    """
    set up basic camera settings
    """
    def __init__ (self, ):
        self.up_vector = [0, 0, 1]
        self.fov = 60
        self.aspect = 1.0
        self.near = 0.1
        self.far = 100
        self.camera_width = 480
        self.camera_height = 480

    """
    a helper function to set up the camera and get the image
    """
    def _set_up_camera(self, camera_position, camera_target):
        view_matrix = p.computeViewMatrix(
                camera_position,
                camera_target,
                self.up_vector
                )
        projection_matrix = p.computeProjectionMatrixFOV(
                self.fov,
                self.aspect,
                self.near,
                self.far
                )
        image = p.getCameraImage(
                self.camera_width,
                self.camera_height,
                view_matrix,
                projection_matrix,
                p.ER_BULLET_HARDWARE_OPENGL
                )
        rgb_image = image[2] 
        depth_image = image[3]
        segmentation_image = image[4]

        return rgb_image

    """
    a helper function to render the scene, from the camera's POV
    """
    def _get_scene(self,base_position, base_orientation, offset, direction):
        rotation_matrix = np.array(p.getMatrixFromQuaternion(base_orientation)).reshape(3, 3)
        camera_pos = np.array(base_position) + rotation_matrix @ offset
        forward = rotation_matrix @ np.array(direction)
        camera_target = camera_pos + forward

        image = self._set_up_camera(camera_position=camera_pos.tolist(), camera_target=camera_target.tolist())

        return image

    """
    gets the front image
    """
    def forward(self, base_position, base_orientation):
        offset = np.array([0.15, 0.0, 0.10])
        direction = [1, 0, 0]
        image = self._get_scene(base_position, base_orientation, offset, direction)
        return image

    """
    gets the rear image
    """
    def backward(self, base_position, base_orientation):
        offset = np.array([-0.15, 0.0, 0.10])
        direction = [-1, 0, 0]
        image = self._get_scene(base_position, base_orientation, offset, direction)
        return image

    """
    gets the left image
    """
    def left(self, base_position, base_orientation):
        offset = np.array([0.0, -0.15, 0.10])
        direction = [0, -1, 0]
        image = self._get_scene(base_position, base_orientation, offset, direction)
        return image


    """
    gets the right image
    """
    def right(self, base_position, base_orientation):
        offset = np.array([0.0, 0.15, 0.10])
        direction = [0, 1, 0]
        image = self._get_scene(base_position, base_orientation, offset, direction)
        return image

