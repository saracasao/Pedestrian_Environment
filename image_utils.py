import os
import sys
import cv2
import airsim
import numpy as np

from joblib import Parallel, delayed

MIN_DEPTH_METERS = 0
MAX_DEPTH_METERS = 100


def get_responses_rgb_ext(client, cam_name):
    """ Get rgb response from external camera
    :param client: airsim.VehicleClient
           cam_name (str): camera name
    :return: responses (list): airsim.response [airsim.ImageType.Scene]"""
    responses = client.simGetImages([airsim.ImageRequest(cam_name, airsim.ImageType.Scene, False, False)], external=True)
    return responses


def get_responses_rgbd_ext(client, cam_name):
    """ Get rgb and depth response from external camera
    :param client: airsim.VehicleClient
           cam_name (str): camera name
    :return: responses (list): airsim.response [airsim.ImageType.Scene,  airsim.ImageType.DepthPlanar]"""
    responses = client.simGetImages([airsim.ImageRequest(cam_name, airsim.ImageType.Scene, False, False),
                                     airsim.ImageRequest(cam_name, airsim.ImageType.DepthPlanar, True, False)], external=True)
    return responses


def get_responses_seg_ext(client, cam_name):
    """ Get rgb, depth and segmentation response from external camera
    :param client: airsim.VehicleClient
           cam_name (str): camera name
    :return: responses (list): airsim.response [airsim.ImageType.Scene,  airsim.ImageType.DepthPlanar, airsim.ImageType.Segmentation]"""
    responses = client.simGetImages([airsim.ImageRequest(cam_name, airsim.ImageType.Scene, False, False),
                                     airsim.ImageRequest(cam_name, airsim.ImageType.DepthPlanar, True, False),
                                     airsim.ImageRequest(cam_name, airsim.ImageType.Segmentation, False, False)], external=True)
    return responses


def get_responses_rgb(client, vehicle_name):
    """ Get rgb response from drone camera
    :param client: airsim.VehicleClient
           vehicle_name (str): drone name
    :return: responses (list): airsim.response [airsim.ImageType.Scene]"""
    responses = client.simGetImages([airsim.ImageRequest('0', airsim.ImageType.Scene, False, False)], vehicle_name=vehicle_name)
    return responses


def get_responses_rgbd(client, vehicle_name):
    """ Get rgb response from drone camera
    :param client: airsim.VehicleClient
           vehicle_name (str): drone name
    :return: responses (list): airsim.response [airsim.ImageType.Scene,  airsim.ImageType.DepthPlanar]"""
    responses = client.simGetImages([airsim.ImageRequest('0', airsim.ImageType.Scene, False, False),
                                     airsim.ImageRequest('0', airsim.ImageType.DepthPlanar, True, False)], vehicle_name=vehicle_name)
    return responses


def get_responses_seg(client, vehicle_name):
    """ Get rgb response from drone camera
    :param client: airsim.VehicleClient
           vehicle_name (str): drone name
    :return: responses (list): airsim.response [airsim.ImageType.Scene,  airsim.ImageType.DepthPlanar, airsim.ImageType.Segmentation]"""
    responses = client.simGetImages([airsim.ImageRequest('0', airsim.ImageType.Scene, False, False),
                                     airsim.ImageRequest('0', airsim.ImageType.DepthPlanar, True, False),
                                     airsim.ImageRequest('0', airsim.ImageType.Segmentation, False, False)], vehicle_name=vehicle_name)
    return responses


def responseTOrgb(response):
    """ From airsim.response to image in array format H X W X 3
    :param response: airsim.response.ImageType.Scene or airsim.ImageType.Segmentation
    :return: img_rgb: array H X W X 3"""
    img_rgb = np.frombuffer(response.image_data_uint8, dtype=np.uint8).reshape(response.height, response.width, 3)
    return img_rgb


def responseTOdepth(response):
    """ From airsim.response to depth in array format H X W
    :param response: airsim.response.ImageType.DepthPlanar
    :return: depth_img_in_meters: depth matrix in meters, array H X W [0,100]
             depth_img_in_mm: depth matrix in mm, array H X W [0,100000]
             depth_img: depth visualization H X W [0,255]"""
    # Reshape to a 2d array with correct width and height
    depth_img_in_meters = airsim.list_to_2d_float_array(response.image_data_float, response.width, response.height)
    depth_img_in_meters = depth_img_in_meters.reshape(response.height, response.width, 1)

    # Lerp 0..100m to 0..255 gray values
    depth_img = np.interp(depth_img_in_meters, (MIN_DEPTH_METERS, MAX_DEPTH_METERS), (0, 255))

    # Convert depth_img to millimeters to fill out 16bit unsigned int space (0..65535). Also clamp large values (e.g. SkyDome) to 65535
    depth_img_in_millimeters = depth_img_in_meters * 1000
    depth_img_in_mm = np.clip(depth_img_in_millimeters, 0, 65535)
    return depth_img_in_meters, depth_img_in_mm, depth_img


def responseTOimages(responses, camera_names):
    """ Get info from responses and transform format to numpy array
    :param responses (list): [airsim.response cam1, airsim.response cam2, ...]
    :return: images_info[cam_name] = {'timestamp': t, 'rgb': array H X W X 3, 'depth': array H X W  [0,100000], 'segmentation': array H X W X 3}"""
    images_info = {}
    for i, resp in enumerate(responses):
        cam_name = camera_names[i]
        images_info[cam_name] = {}
        images_info[cam_name]['timestamp'] = resp[0].time_stamp
        for j, r in enumerate(resp):
            if r.pixels_as_float:
                depth_matrix = responseTOdepth(r)
                images_info[cam_name]['depth'] = depth_matrix[1]
            else:
                img = responseTOrgb(r)
                if j == 0:
                    images_info[cam_name]['rgb'] = img
                else:
                    images_info[cam_name]['segmentation'] = img
    return images_info


def getResponseImages(clients, config):
    """ Get images from cameras
    :param client: airsim.VehicleClient
           config: Configuration Class
    :return: images_info[cam_name] = {'timestamp': t, 'rgb': array H X W X 3, 'depth': array H X W  [0,100000], 'segmentation': array H X W X 3}"""

    camera_names = config.camera_names

    # Select image response
    if config.image_types == 'RGB' and config.external:
        responses = Parallel(n_jobs=config.number_cameras, backend='threading')(
            delayed(get_responses_rgb_ext)(cl, name) for cl, name in zip(clients, camera_names))
    elif config.image_types == 'RGB' and not config.external:
        responses = Parallel(n_jobs=config.number_cameras, backend='threading')(
            delayed(get_responses_rgb)(cl, name) for cl, name in zip(clients, camera_names))
    elif config.image_types == 'RGB-D' and config.external:
        responses = Parallel(n_jobs=config.number_cameras, backend='threading')(
            delayed(get_responses_rgbd_ext)(cl, name) for cl, name in zip(clients, camera_names))
    elif config.image_types == 'RGB-D' and not config.external:
        responses = Parallel(n_jobs=config.number_cameras, backend='threading')(
            delayed(get_responses_rgbd)(cl, name) for cl, name in zip(clients, camera_names))
    elif config.image_types == 'RGB-DS' and config.external:
        responses = Parallel(n_jobs=config.number_cameras, backend='threading')(
            delayed(get_responses_seg_ext)(cl, name) for cl, name in zip(clients, camera_names))
    elif config.image_types == 'RGB-DS' and not config.external:
        responses = Parallel(n_jobs=config.number_cameras, backend='threading')(
            delayed(get_responses_seg)(cl, name) for cl, name in zip(clients, camera_names))
    else:
        sys.exit('Image types defined is not implemented')

    images_info = responseTOimages(responses, config.camera_names)
    return images_info


def check_path(path):
    """Check if path exists"""
    if not os.path.exists(path):
        os.makedirs(path)


def save_images(images_info, frame_index_key, cam, config):
    """ Get images from cameras
    :param images_info: dict = {'cam1':{'timestamp': t, 'rgb': array H X W X 3, 'depth': array H X W  [0,100000], 'segmentation': array H X W X 3}, ...}
           frame_index_key (str): frame index converted to string, e.g., 0000
           cam (str): camera name
           config: Configuration Class
    """

    time_stamp = str(images_info['timestamp'])
    final_name = frame_index_key + '_' + time_stamp

    path_save = config.path_save + '/' + cam

    if 'RGB' in config.image_types:
        final_path_rgb = path_save + '/Frames'
        check_path(final_path_rgb)

        cv2.imwrite(final_path_rgb + '/' + final_name + '.png', images_info['rgb'])
    if 'D' in config.image_types:
        final_path_depth = path_save + '/Depth'
        check_path(final_path_depth)

        np.save(final_path_depth + '/' + final_name + '.npy', images_info['depth'])
    if 'S' in config.image_types:
        final_path_seg = path_save + '/Segmentation'
        check_path(final_path_seg)

        cv2.imwrite(final_path_seg + '/' + final_name + '.png', images_info['segmentation'])




