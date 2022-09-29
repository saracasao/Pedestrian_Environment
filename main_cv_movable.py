import airsim
import os
import json

from Visualization import visualize
from image_utils import getResponseImages, save_images
from camera_drone import MultiRotor
from settings import Configuration
from pedestrians import get_name_pedestrians, update_gt3d_pedestrian, save_3dgroundTruth, \
    update_gt2d_pedestrian, save_2dgroundTruth


"""
Example ./Documents/AirSim/settings.json for using multiple drone cameras
{
	"SettingsVersion": 1.7,
	"SimMode": "Multirotor",
	"Vehicles": {
		"Drone1": {
		  "VehicleType": "SimpleFlight",
          "X": 1.5, "Y": -1.5, "Z": 0
		}
    }
}
"""


def load_settings_airsim():
    """Load AirSim Settings"""
    path_settings = '/home/scasao/Documents/AirSim/settings.json' #Change path to corresponding ./Documents/AirSim/settings.json
    with open(path_settings, 'r') as f:
        settings = json.load(f)
    return settings


def InitDrones(velocity):
    """ Initialize uavs
    :param velocity (float): speed at which the drones will move
    :return: uavs: Multirotor Class"""
    airsim_settings = load_settings_airsim()
    vehicles = airsim_settings['Vehicles']

    uavs = []
    drone_names = list(vehicles.keys())
    for drone_name in drone_names:
        x_init = vehicles[drone_name]['X']
        y_init = vehicles[drone_name]['Y']
        z_init = vehicles[drone_name]['Z']
        drone = MultiRotor(drone_name, [x_init, y_init, z_init], velocity)
        uavs.append(drone)
    return uavs


def CV_Capture(clients, uavs, config, frames_to_capture):
    drone_names = config.camera_names

    print('-> Drone names defined {} in MODE: {} '.format(drone_names, config.mode))

    # -------------Connect AirSim with Unreal--------------
    client_ref = clients[0]
    for drone in uavs:
        drone.arm(client_ref)

    # -----Taking off----------------
    print('Taking off...')
    task = None
    for drone in uavs:
        task = drone.take_off(client_ref)
    task.join()

    # -------------Create Directory-----------------
    folder_date = config.name_experiment
    path_save = os.path.join(os.getcwd(), (config.mode + '/' + folder_date))
    print('-> Saving info to {}'.format(path_save))

    for d_name in drone_names:
        final_path = path_save + '/' + d_name
        if not os.path.exists(final_path):
            os.makedirs(final_path)
    config.path_save = path_save

    # -------------------Weather----------------------------
    # config.set_weather(client_ref, 'fog', 0.2)

    # -------------Init Pedestrians Info--------------------
    gt3d_pedestrians, gt2d_pedestrians = {}, {}
    struct_ref = 'BP_P_'  # reference structure in Unreal to look for the pedestrians
    name_pedestrians, dict_names = get_name_pedestrians(client_ref, struct_ref, segmentation=config.semantic_segmentation)
    print('-> Pedestrians found in the scene: {}'.format(name_pedestrians))

    for d_name in drone_names:
        gt2d_pedestrians[d_name] = {}
        client_ref.simAddDetectionFilterMeshName(camera_name='0', vehicle_name=d_name, image_type=airsim.ImageType.Scene, mesh_name="BP_P*")

    # --------Get info from cameras-----------------------
    if config.save_mode == 'wait':
        airsim.wait_key('PRES ANY KEY TO GET IMAGES')

    # --------MoveTO------------------
    for drone in uavs:
        next_pos, next_yaw = drone.get_goal_position_and_orientation()
        print('NEXT POS DRONE', next_pos)
        task = drone.moveTOpos(client_ref, next_pos, next_yaw)

    frame_index = 0
    while frame_index < frames_to_capture:
        print(frame_index)
        frame_index_key = str(frame_index)
        frame_index_key = frame_index_key.zfill(4)

        # Get images from external cameras
        images = getResponseImages(clients, config)

        captured_pedestrians = []
        for i, d_name in enumerate(drone_names):
            # Update 2d pedestrians bbox obtained
            gt2d_pedestrians[d_name][frame_index_key] = []
            gt2d_pedestrians, pedestrians_in_frame = update_gt2d_pedestrian(client_ref, d_name, gt2d_pedestrians, frame_index_key, config)
            captured_pedestrians = captured_pedestrians + pedestrians_in_frame

            # Update drone and camera state
            drone = uavs[i]
            drone.update_state_cam_info(client_ref, frame_index)
            drone.update_state_drone_info(client_ref, frame_index)

            # Save images
            save_images(images[d_name], frame_index_key, d_name, config)

        # Update 3d pedestrians position
        captured_pedestrians = set(captured_pedestrians)
        gt3d_pedestrians = update_gt3d_pedestrian(client_ref, captured_pedestrians, gt3d_pedestrians, frame_index_key)

        # Visualize data
        if config.visualize_images and config.vis_pedestrian_2dGT:
            visualize(drone_names, images, frame_index, gt2d_pedestrians, dict_names)
        elif config.visualize_images:
            visualize(drone_names, images, frame_index)
        frame_index += 1

    # Save pedestrians data
    save_3dgroundTruth(gt3d_pedestrians, config.path_save)
    save_2dgroundTruth(gt2d_pedestrians, drone_names, config.path_save)

    # Save sensor data
    for drone in uavs:
        drone.save_info_state(config.path_save)


if __name__ == "__main__":
    # Settings
    save_mode = 'start'  # 'start', 'wait'
    frames_to_capture = 500
    name_experiment = 'MobileCamera'
    img_types = 'RGB'  # 'RGB' , 'RGB-D', 'RGB-DS'
    velocity = 0.5

    # Defining settings
    uavs = InitDrones(velocity)
    config = Configuration(img_types, frames_to_capture, save_mode, name_experiment=name_experiment, external=False, uavs=uavs)

    # Initializing Vehicle of AirSim in Unreal
    clients = [airsim.MultirotorClient() for _ in range(config.number_cameras)]
    client = clients[0]

    # Data capture
    CV_Capture(clients, uavs, config, frames_to_capture)
