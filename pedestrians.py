import json
import pathlib
import airsim


def get_dict_colors():
    """Load color dict for segmentation to assign to each pedestrian
    :return: dict [int] = [r,g,b]"""
    current_path = str(pathlib.Path().resolve())
    file = current_path + '/segmentation_rgb.txt'

    color_dict = {}
    with open(file) as file:
        for line in file:
            key, value = line.split('\t')
            value = value.replace('[', ' ')
            value = value.replace(']', ' ')
            r, g, b = value.split(',')

            color_dict[int(key)] = [int(r), int(g), int(b)]
    return color_dict


def get_name_pedestrians(client, ref_struct, segmentation=False):
    """Get all people models presented in the scene.
       If image segmentation is active, set specific color for each pedestrian and save dict with correspondences
    :param client: airsim.VehicleClient
           ref_struct (str): key structure to find people models in the scene
           segmentation (bool): get image segmentation
    :return: name_pedestrians (list str): list of pedestrians names
             dict_names"""
    name_pedestrians = client.simListSceneObjects(ref_struct + '.*')
    dict_names = {name: i for i, name in enumerate(name_pedestrians)}
    if segmentation:
        colors = get_dict_colors()
        color_to_pedestrian = {}
        for i, name_ped in enumerate(name_pedestrians):
            client.simSetSegmentationObjectID(name_ped, i + 1, True)
            color_to_pedestrian[name_ped] = colors[i + 1]

        with open('color_to_pedestrian.json', 'w') as f:
            json.dump(color_to_pedestrian, f)
    return name_pedestrians, dict_names


def update_gt3d_pedestrian(client, name_pedestrians, info_pedestrians_3d, frame_index_key):
    """ Update 3d ground truth of pedestrians presented in the scene
    :param client: airsim.VehicleClient
           name_pedestrians (list str): list of pedestrians names to save 3d information
           info_pedestrians_3d (dict): info_pedestrians_3d[frame_index_key] = [{info_ped1},{info_ped2},...]
           frame_index_key (str): frame index converted to string, e.g., 0000
    :return: info_pedestrians_3d (dict): info_pedestrians_3d[frame_index_key] = [{info_ped1},{info_ped2},...]"""

    info_pedestrians_3d[frame_index_key] = []
    for name_ped in name_pedestrians:
        pose = client.simGetObjectPose(name_ped)
        pos_x = pose.position.x_val
        pos_y = pose.position.y_val
        pos_z = pose.position.z_val

        orient_w = pose.orientation.w_val
        orient_x = pose.orientation.x_val
        orient_y = pose.orientation.y_val
        orient_z = pose.orientation.z_val

        info = {'id': name_ped,
                'pos_x': pos_x,
                'pos_y': pos_y,
                'pos_z': pos_z,
                'orient_w': orient_w,
                'orient_x': orient_x,
                'orient_y': orient_y,
                'orient_z': orient_z}
        info_pedestrians_3d[frame_index_key].append(info)
    return info_pedestrians_3d


def update_gt2d_pedestrian(client, cam_name, info_pedestrians, frame_index_key, config):
    """ Update 2d ground truth of pedestrians presented in the scene
    :param client: airsim.VehicleClient
           name_pedestrians (list str): list of pedestrians names to save 2d information
           info_pedestrians (dict): info_pedestrians[cam_name][frame_index_key] = [{info_ped1},{info_ped2},...]
           frame_index_key (str): frame index converted to string, e.g., 0000
           config: Configuration Class
    :return: info_pedestrians (dict): info_pedestrians[cam_name][frame_index_key] = [{info_ped1},{info_ped2},...]
             captured_pedestrians (list str): list of pedestrian names detected in the scene"""

    captured_pedestrians = []
    if config.external:
        bboxes = client.simGetDetections(camera_name=cam_name, image_type=airsim.ImageType.Scene, external=True)
    else:
        bboxes = client.simGetDetections(camera_name='0', vehicle_name=cam_name, image_type=airsim.ImageType.Scene)

    for bbox in bboxes:
        name_ped = bbox.name
        xmin = bbox.box2D.min.x_val
        ymin = bbox.box2D.min.y_val
        xmax = bbox.box2D.max.x_val
        ymax = bbox.box2D.max.y_val

        # AirSim bbox get width oversize -> reduce to half
        width = xmax-xmin
        new_xmin = xmin + int(width/4)
        new_xmax = new_xmin + int(width/2)

        info = {'id': name_ped,
                'xmin': new_xmin,
                'ymin': ymin,
                'xmax': new_xmax,
                'ymax': ymax}
        info_pedestrians[cam_name][frame_index_key].append(info)
        captured_pedestrians.append(name_ped)
    return info_pedestrians, captured_pedestrians


def save_3dgroundTruth(info_pedestrians, path_save_info):
    """"Save 3d ground truth"""
    name_file = 'gt3d_pedestrians.json'

    final_path = path_save_info + '/' + name_file
    with open(final_path, 'w') as f:
        json.dump(info_pedestrians, f)


def save_2dgroundTruth(info_pedestrians, camera_names, path_save_info):
    """"Save 2d ground truth"""
    name_file = 'gt2d_pedestrians.json'

    for folder in camera_names:
        final_path = path_save_info + '/' + folder + '/' + name_file
        with open(final_path, 'w') as f:
            json.dump(info_pedestrians[folder], f)

