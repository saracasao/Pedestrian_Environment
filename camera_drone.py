import airsim
import json
import math
import numpy as np

from scipy.spatial.transform import Rotation as R

environment_street_limits = {"Drone1": [10, 14, -7]}# [-153.4, -2, -4]}
orientations_deg = {"Drone1": [0, 0, -135]}


class MultiRotor:
    def __init__(self, name, init_position, vel):
        self.name = name
        self.X_init = init_position[0]
        self.Y_init = init_position[1]
        self.Z_init = init_position[2]
        self.vel = vel

        self.cam_states = []
        self.states = []

        self.extrinsics_camera = None

    def arm(self, client):
        """ Arm drone
        :param client: airsim.VehicleClient"""
        client.enableApiControl(True, self.name)
        client.armDisarm(True, self.name)

    def take_off(self, client):
        """ Take off drone
        :param client: airsim.VehicleClient"""
        task = client.takeoffAsync(vehicle_name=self.name)
        return task

    def disarm(self, client):
        """ Disarm drone
        :param client: airsim.VehicleClient"""
        client.armDisarm(False, self.name)
        return client

    def moveTOpos(self, client, position, yaw=None):
        """ Move the drone to a goal position
        :param client: airsim.VehicleClient
               position (list): [x,y,z] meters
               yaw (float) = angle degrees """
        x_f = position[0] - self.X_init
        y_f = position[1] - self.Y_init
        z_f = position[2] - self.Z_init

        if yaw is not None:
            task = client.moveToPositionAsync(x_f, y_f, z_f, self.vel, yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=yaw), vehicle_name=self.name)
        else:
            task = client.moveToPositionAsync(x_f, y_f, z_f, self.vel, vehicle_name=self.name)
        return task

    def get_position(self, position):
        """ Get the actual position in AirSimLocal coordinates
        :param position: state.pose.position of the state"""
        x_f = position.x_val + self.X_init
        y_f = position.y_val + self.Y_init
        z_f = position.z_val + self.Z_init

        return x_f, y_f, z_f

    def set_position(self, client, position, orientation):
        """ Teleport drone to position with orientation
        :param client: airsim.VehicleClient
               position (list) = [x,y,z] meters
               orientation (list) = [pitch,roll,yaw] angle radians """
        pose = airsim.Pose(airsim.Vector3r(position[0], position[1], position[2]),
                           airsim.to_quaternion(orientation[0], orientation[1],orientation[2]))
        client.simSetVehiclePose(pose, True, vehicle_name=self.name)

    def cam_state(self, client, cam_name="0"):
        """
        :param client: airsim.VehicleClient
        :return: camera state in AirSimLocal coordinates"""
        camera_info = client.simGetCameraInfo(cam_name, vehicle_name=self.name)

        global_position = self.get_position(camera_info.pose.position)
        camera_info.pose.position.x_val = global_position[0]
        camera_info.pose.position.y_val = global_position[1]
        camera_info.pose.position.z_val = global_position[2]
        return camera_info

    def get_state(self, client):
        """
        :param client: airsim.VehicleClient
        :return: drone state in AirSimLocal coordinates"""
        state = client.simGetGroundTruthKinematics(vehicle_name=self.name)

        global_position = self.get_position(state.position)
        state.position.x_val = global_position[0]
        state.position.y_val = global_position[1]
        state.position.z_val = global_position[2]
        return state

    def get_focal_length(self, client):
        """
        :param client: airsim.VehicleClient
        :return: focal length of the camera [mm]
        """
        camera_name = "0"
        f = client.simGetFocalLength(camera_name, vehicle_name=self.name)
        return f

    def get_focal_length_px(self, fov, frame):
        """
        :param cam_name (str): camera name
               frame (array image)
        :return: focal lenght in pixels
        """
        f_fov = self.get_f_fov(fov)
        width_frame = np.shape(frame)[1]

        f_px = width_frame / f_fov
        return f_px

    @staticmethod
    def get_f_fov(fov):
        """"Intermediate state to obtain focal length in px f_px = (width / (2 * math.tan((fov / 2) * (math.pi / 180)))
            with fov in degrees
        :param fov degrees
        :return: denominator of focal length in pixels calculation"""
        den = 2 * math.tan((fov / 2) * (math.pi / 180))
        return den

    def get_gps_data(self, client):
        gps_data = client.getGpsData(vehicle_name=self.name)
        return gps_data

    def get_goal_position_and_orientation(self):
        """ Get new goals positions
        :return: position [x,y,z] meters
                 orientation [pitch,roll,yaw] angle degrees"""
        next_pos = environment_street_limits[self.name]
        next_yaw = orientations_deg[self.name][2]
        nex_yaw = None
        return next_pos, nex_yaw

    def get_extrinsic(self, client):
        """ Get extrinsic of the system
        :param client: airsim.VehicleClient
        :return: translation (list) = [[x],[y],[z]]
                 rot_matrix (3x3 matrix)
                 extrinsic matrix (4x3 matrix)"""
        # Get rotation matrix
        state = self.get_state(client)
        rotation = state.orientation
        quaternions = [rotation.x_val, rotation.y_val, rotation.z_val, rotation.w_val]
        rot = R.from_quat(quaternions)
        rot_matrix = rot.as_matrix()

        # Get translation
        translation = [[state.position.x_val],
                       [state.position.y_val],
                       [state.position.z_val]]
        E = np.append(rot_matrix, translation, axis=1)

        assert np.shape(E)[0] == 3 and np.shape(E)[1] == 4

        return translation, rot_matrix, E

    def set_extrinsic_camera(self, state):
        """ Get extrinsic of the system
        :param client: airsim.VehicleClient
        :return: translation (list) = [[x],[y],[z]]
                 rot_matrix (3x3 matrix)
                 extrinsic matrix (4x3 matrix)"""
        # Get rotation matrix
        rotation = state.pose.orientation
        quaternions = [rotation.x_val, rotation.y_val, rotation.z_val, rotation.w_val]
        rot = R.from_quat(quaternions)
        rot_matrix = rot.as_matrix()

        # Get translation
        translation = [[state.pose.position.x_val],
                       [state.pose.position.y_val],
                       [state.pose.position.z_val]]
        E = np.append(rot_matrix, translation, axis=1)

        assert np.shape(E)[0] == 3 and np.shape(E)[1] == 4
        self.extrinsics_camera = translation, rot_matrix

    def update_state_cam_info(self, client, frame_index, rgb):
        """ Update dictionary of camera states
        :param client: airsim.VehicleClient
               frame_index (int) """
        state = self.cam_state(client)
        self.set_extrinsic_camera(state)

        cam_position = state.pose.position
        cam_orientation = state.pose.orientation

        pos_x = cam_position.x_val
        pos_y = cam_position.y_val
        pos_z = cam_position.z_val

        orient_w = cam_orientation.w_val
        orient_x = cam_orientation.x_val
        orient_y = cam_orientation.y_val
        orient_z = cam_orientation.z_val

        f_fov = self.get_focal_length_px(state.fov, rgb)
        focal_length = self.get_focal_length(client)

        info_state = {'frame_index': frame_index,
                      'focal_length': focal_length,
                      'f_fov': f_fov,
                      'pos_x': pos_x,
                      'pos_y': pos_y,
                      'pos_z': pos_z,
                      'orient_w': orient_w,
                      'orient_x': orient_x,
                      'orient_y': orient_y,
                      'orient_z': orient_z}
        self.cam_states.append(info_state)

    def update_state_drone_info(self, client, frame_index):
        """ Update dictionary of drone states
        :param client: airsim.VehicleClient
               frame_index (int) """
        state = self.get_state(client)
        position = state.position
        orientation = state.orientation

        pos_x = position.x_val
        pos_y = position.y_val
        pos_z = position.z_val

        orient_w = orientation.w_val
        orient_x = orientation.x_val
        orient_y = orientation.y_val
        orient_z = orientation.z_val

        focal_length = self.get_focal_length(client)

        info_state = {'frame_index': frame_index,
                      'focal_length': focal_length,
                      'pos_x': pos_x,
                      'pos_y': pos_y,
                      'pos_z': pos_z,
                      'orient_w': orient_w,
                      'orient_x': orient_x,
                      'orient_y': orient_y,
                      'orient_z': orient_z}
        self.states.append(info_state)

    def save_info_state(self, path):
        """ Save camera and drone states
        :param path (str): final path to save file """
        final_path_drone = path + '/' + self.name + '/state_info.json'
        with open(final_path_drone, 'w') as f:
            json.dump(self.states, f)

        final_path_cam = path + '/' + self.name + '/state_cam_info.json'
        with open(final_path_cam, 'w') as f:
            json.dump(self.cam_states, f)
