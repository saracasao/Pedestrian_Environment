import airsim
import os
import json

from Visualization import visualize
from image_utils import getResponseImages, save_images
from settings import Configuration
from pedestrians import get_name_pedestrians, update_gt3d_pedestrian, save_3dgroundTruth, \
    update_gt2d_pedestrian, save_2dgroundTruth, get_gt3d_pedestrian_info


"""
Example Settings for use only external camera in the simple environment
{
	"SettingsVersion": 1.7, 
	"SimMode": "ComputerVision",
	"ExternalCameras": {
        "cam1": {
            "X": 8.5, "Y": -8.2, "Z": -3,
            "Pitch": 0, "Roll": 0, "Yaw": 135
        },
        "cam2": {
            "X": 8.5, "Y": 13.2, "Z": -3,
            "Pitch": 0, "Roll": 0, "Yaw": -135
        }
    }
}
"""


def save_camera_state(cam, config):
    info_cam = config.camera_states[cam]
    path_save = config.path_save + '/' + cam + '/camera_info.json'
    with open(path_save, 'w') as outfile:
        json.dump(info_cam, outfile)


def CV_Capture(clients, config, frames_to_capture):
    cameras_names = config.camera_names

    print('-> External cameras defined {} in MODE: {} '.format(cameras_names, config.mode))

    # -------------Connect AirSim with Unreal--------------
    client_ref = clients[0]
    client_ref.confirmConnection()

    # -------------Create Directory-----------------
    folder_date = config.name_experiment
    path_save = os.path.join(os.getcwd(), (config.mode + '/' + folder_date))
    print('-> Saving info to {}'.format(path_save))

    for cam_name in config.camera_names:
        final_path = path_save + '/' + cam_name
        if not os.path.exists(final_path):
            os.makedirs(final_path)
    config.path_save = path_save

    config.camera_states = config.get_camera_states(client_ref)
    for cam in cameras_names:
        save_camera_state(cam, config)

    # -------------Init Pedestrians Info--------------------
    gt3d_pedestrians, gt2d_pedestrians = {}, {}
    struct_ref = 'BP_P_'  # reference structure in Unreal to look for the pedestrians
    name_pedestrians, dict_names = get_name_pedestrians(client_ref, struct_ref, segmentation=config.semantic_segmentation)
    print('-> Pedestrians found in the scene: {}'.format(name_pedestrians))

    for cam_name in cameras_names:
        gt2d_pedestrians[cam_name] = {}
        client_ref.simAddDetectionFilterMeshName(camera_name=cam_name, image_type=airsim.ImageType.Scene, mesh_name="BP_P*", external=True)

    # --------Get info from cameras-----------------------
    if config.save_mode == 'wait':
        airsim.wait_key('PRES ANY KEY TO GET IMAGES')

    frame_index = 0
    while frame_index < frames_to_capture:
        print(frame_index)
        frame_index_key = str(frame_index)
        frame_index_key = frame_index_key.zfill(4)

        # Get images from external cameras
        images = getResponseImages(clients, config)

        info_gt3d_pedestrians_scene = get_gt3d_pedestrian_info(client_ref, name_pedestrians)
        captured_pedestrians = []
        for cam in cameras_names:
            # Update 2d pedestrians bbox obtained
            gt2d_pedestrians[cam][frame_index_key] = []
            gt2d_pedestrians, pedestrians_in_frame = update_gt2d_pedestrian(client_ref, cam, gt2d_pedestrians, info_gt3d_pedestrians_scene, images[cam]['rgb'], frame_index_key, config)
            captured_pedestrians = captured_pedestrians + pedestrians_in_frame

            # Save images selected
            save_images(images[cam], frame_index, cam, config)

        # Update 3d pedestrians position
        captured_pedestrians = set(captured_pedestrians)
        gt3d_pedestrians = update_gt3d_pedestrian(info_gt3d_pedestrians_scene, captured_pedestrians, gt3d_pedestrians, frame_index_key)

        # Visualize data
        if config.visualize_images and config.vis_pedestrian_2dGT:
            visualize(cameras_names, images, frame_index, gt2d_pedestrians, dict_names, path_save + '/GT')
        elif config.visualize_images:
            visualize(cameras_names, images, frame_index)
        frame_index += 1

    # Save data
    save_3dgroundTruth(gt3d_pedestrians, config.path_save)
    save_2dgroundTruth(gt2d_pedestrians, cameras_names, config.path_save)


if __name__ == "__main__":
    # Settings
    save_mode = 'start'  # 'start', 'wait'
    frames_to_capture = 500
    name_experiment = 'TEST_PROJECTIONS'
    img_types = 'RGB-D'  # 'RGB-D', 'RGB-DS'

    # Defining settings
    config = Configuration(img_types, frames_to_capture, save_mode, name_experiment, visualize_images=True, vis_pedestrian_2dGT=True, save_camera_state=True)

    # Initializing Vehicle of AirSim in Unreal
    clients = [airsim.VehicleClient() for _ in range(config.number_cameras)]
    client = clients[0]

    # Data capture
    CV_Capture(clients, config, frames_to_capture)
