import math
import json
import sys
import numpy as np
import airsim

from scipy.spatial.transform import Rotation as R


class Configuration:
    def __init__(self, img_types, nframes, save_mode, name_experiment, visualize_images=False,
                 vis_pedestrian_2dGT=False, save_camera_state=True, external=True, uavs=None):

        self.mode = 'record_data'
        self.settings_airsim = self.load_settings_airsim()
        self.number_frames = nframes
        self.camera_states = None
        self.image_types = img_types
        self.save_mode = save_mode
        self.name_experiment = name_experiment
        self.visualize_images = visualize_images
        self.vis_pedestrian_2dGT = vis_pedestrian_2dGT
        self.save_pose_cameras = save_camera_state
        self.external = external
        self.path_save = None

        if self.external:
            self.camera_names, self.number_cameras = self.get_cameras_info()
        elif uavs is not None:
            drone_names = [drone.name for drone in uavs]
            self.camera_names = drone_names
            self.number_cameras = len(drone_names)
        else:
            sys.exit('Camera settings is not correct: 1. External cameras or 2.UAV as movable cameras')

        if self.image_types == 'RGB-DS':
            self.semantic_segmentation = True
        else:
            self.semantic_segmentation = False

    @staticmethod
    def load_settings_airsim():
        """Load AirSim Settings"""
        path_settings = '/home/scasao/Documents/AirSim/settings.json' #Change path to corresponding ./Documents/AirSim/settings.json
        with open(path_settings, 'r') as f:
            settings = json.load(f)
        return settings

    # ---------Camera Info------------------
    def get_cameras_info(self):
        """Get camera info from AirSim Settings
        :return: camera_names (list str)
                 number of cameras (int)"""
        cameras = self.settings_airsim['ExternalCameras']
        camera_names = list(cameras.keys())

        return camera_names, len(camera_names)

    def get_camera_states(self, client):
        """Get camera states
        :param client: airsim.VehicleClient
        :return: dict camera parameters"""
        info_cameras = {}
        for cam in self.camera_names:
            f_mm = client.simGetFocalLength(cam, external=True)
            camera_info = client.simGetCameraInfo(cam, external=True)
            position = camera_info.pose.position
            orientation = camera_info.pose.orientation

            f_fov = self.get_f_fov(camera_info.fov)

            info_cameras[cam] = {
                'fov': camera_info.fov,
                'f_fov': f_fov,
                'focal_length_mm': f_mm,
                'pos_x': position.x_val,
                'pos_y': position.y_val,
                'pos_z': position.z_val,
                'orient_w': orientation.w_val,
                'orient_x': orientation.x_val,
                'orient_y': orientation.y_val,
                'orient_z': orientation.z_val
            }
        return info_cameras

    @staticmethod
    def get_f_fov(fov):
        """"Intermediate state to obtain focal length in px f_px = (width / (2 * math.tan((fov / 2) * (math.pi / 180)))
            with fov in degrees
        :param fov degrees
        :return: denominator of focal length in pixels calculation"""
        den = 2 * math.tan((fov / 2) * (math.pi / 180))
        return den

    def get_fov(self, cam_name):
        """Get fov defined in AirSim/settings.json"""
        return self.camera_states[cam_name]['fov']

    def get_focal_length_px(self, cam_name, frame):
        """
        :param cam_name (str): camera name
               frame (array image)
        :return: focal lenght in pixels
        """
        f_fov = self.camera_states[cam_name]['f_fov']
        width_frame = np.shape(frame)[1]

        f_px = width_frame / f_fov
        return f_px

    def get_focal_length_mm(self, cam_name):
        """Get focal lenght in mm
        :param cam_name (str): camera name
        :return: dict camera parameters"""
        return self.camera_states[cam_name]['focal_length_mm']

    def get_pose(self, cam_name):
        """Get camera pose
        :param cam_name (str): camera name
        :return: translation (list) = [[x],[y],[z]]
                 rot_matrix (3x3 matrix)"""
        info_cam = self.camera_states[cam_name]
        translation = [[info_cam['pos_x']],
                       [info_cam['pos_y']],
                       [info_cam['pos_z']]]

        quaternions = [info_cam['orient_x'], info_cam['orient_y'], info_cam['orient_z'], info_cam['orient_w']]
        rot = R.from_quat(quaternions)
        rot_matrix = rot.as_matrix()
        return translation, rot_matrix

    # ---------Weather------------------
    @staticmethod
    def set_weather(client, type_weather, percentage):
        """Set weather and intensity
        :param client: airsim.VehicleClient
               type_weather (str): weather to activate
               percentage (float): intensity of the weather between [0,1]"""
        client.simEnableWeather(True)
        if type_weather == 'rain':
            client.simSetWeatherParameter(airsim.WeatherParameter.Rain, percentage) #AirSim.WeatherParameter.Rain does not work really well
        elif type_weather == 'snow':
            client.simSetWeatherParameter(airsim.WeatherParameter.Snow, percentage)
        elif type_weather == 'fog':
            client.simSetWeatherParameter(airsim.WeatherParameter.Fog, percentage)
        elif type_weather == 'maples':
            client.simSetWeatherParameter(airsim.WeatherParameter.MapleLeaf, percentage)
        elif type_weather == 'dust':
            client.simSetWeatherParameter(airsim.WeatherParameter.Dust, percentage)

    @staticmethod
    def stop_all_weather(client):
        """Stop all weather
        :param client: airsim.VehicleClient"""
        client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0.0)
        client.simSetWeatherParameter(airsim.WeatherParameter.Snow, 0.0)
        client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 0.0)
        client.simSetWeatherParameter(airsim.WeatherParameter.MapleLeaf, 0.0)
        client.simSetWeatherParameter(airsim.WeatherParameter.Dust, 0.0)

        client.simEnableWeather(False)

    @staticmethod
    def stop_weather(client, type_weather):
        """Stop one of the active types of weather
        :param client: airsim.VehicleClient
               type_weather (str): weather to activate"""
        if type_weather == 'rain':
            client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0.0)
        elif type_weather == 'snow':
            client.simSetWeatherParameter(airsim.WeatherParameter.Snow, 0.0)
        elif type_weather == 'fog':
            client.simSetWeatherParameter(airsim.WeatherParameter.Fog, 0.0)
        elif type_weather == 'maples':
            client.simSetWeatherParameter(airsim.WeatherParameter.MapleLeaf, 0.0)
        elif type_weather == 'dust':
            client.simSetWeatherParameter(airsim.WeatherParameter.Dust, 0.0)

    # ---------Day Time------------------
    @staticmethod
    def set_day_time(client, time_of_day):
        """
        time_of_day=%Y-%m-%d %H:%M:%S, e.g: 2018-02-12 15:20:00
        """
        client.simSetTimeOfDay(True, start_datetime=time_of_day)
