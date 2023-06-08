import cv2
from image_utils import check_path


def draw_bboxes(info_pedestrians, images, cam, dict_names):
    """ Draw bounding boxes of pedestrian detection in rgb image
    :param info_pedestrians (list): [{'id': name_pedestrian, 'pos_x': x, 'pos_y': y, 'pos_z': z, 'orient_w': o_w, 'orient_x': o_x, 'orient_y': o_y, 'orient_z': o_z}, ...]
           images (dict): {'cam1':{'timestamp': t, 'rgb': array H X W X 3, 'depth': array H X W  [0,100000], 'segmentation': array H X W X 3}, ...}
           cam (str): camera name
           dict_names (dict): dict str pedestrian names to int identity, e.g., {'BP_P_Chris': 0, 'BP_P_Manolo': 1,...}
    :return: images (dict): {'cam1':{'timestamp': t, 'rgb': array H X W X 3, 'depth': array H X W  [0,100000], 'segmentation': array H X W X 3}, ...}
    """

    color_bbox = (255,255,255)
    name = (0, 0, 255)
    for ped in info_pedestrians:
        ped_name = ped['id']
        xmin = int(ped['xmin'])
        ymin = int(ped['ymin'])
        xmax = int(ped['xmax'])
        ymax = int(ped['ymax'])

        width = xmax - xmin
        identity = dict_names[ped_name]

        cv2.rectangle(images[cam]['rgb'], (xmin, ymin), (xmax, ymax), color_bbox, 5)
        # cv2.putText(images[cam]['rgb'], str(identity), (int(xmin + width/2 - 10), ymin),
        #             cv2.FONT_HERSHEY_SIMPLEX, 3, name, 5)
        cv2.putText(images[cam]['rgb'], str(identity), (int(xmin + width/8 - 5), ymin),
                    cv2.FONT_HERSHEY_SIMPLEX, 3, name, 5)
    cv2.putText(images[cam]['rgb'], str(cam), (30, images[cam]['rgb'].shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 0), 5)
    return images


def visualize(cameras, images, frame_index, gt2d_pedestrians=None, dict_names=None, path_to_save=None):
    """ Visualize rgb images captured
    :param cameras (list str): camera names
           frame_index (int)
           images (dict): {'0000': {'cam1':{'timestamp': t, 'rgb': array H X W X 3, 'depth': array H X W  [0,100000], 'segmentation': array H X W X 3}, ...},
                           '0001': {'cam1':{'timestamp': t, 'rgb': array H X W X 3, 'depth': array H X W  [0,100000], 'segmentation': array H X W X 3}, ...},
                           ...}
           gt2d_pedestrians (dict): {'cam1': {'0000': [{'id': name_pedestrian, 'pos_x': x, 'pos_y': y, 'pos_z': z, 'orient_w': o_w, 'orient_x': o_x, 'orient_y': o_y, 'orient_z': o_z}, ...],
                                             '0001': [{'id': name_pedestrian, 'pos_x': x, 'pos_y': y, 'pos_z': z, 'orient_w': o_w, 'orient_x': o_x, 'orient_y': o_y, 'orient_z': o_z},...],
                                             ...}
                                     {'cam2':{'0000': [...], '0001': [...]}}
           dict_names (dict): dict str pedestrian names to int identity, e.g., {'BP_P_Chris': 0, 'BP_P_Manolo': 1,...}
    """
    if len(cameras) == 4:
        row_frames1, row_frames2 = [], []
        for i, cam in enumerate(cameras):
            if gt2d_pedestrians is not None:
                frame_index_key = str(frame_index)
                frame_index_key = frame_index_key.zfill(4)

                info_pedestrians = gt2d_pedestrians[cam][frame_index_key]
                images = draw_bboxes(info_pedestrians, images, cam, dict_names)
            if cam == cameras[0] or cam == cameras[1]:
                row_frames1.append(images[cam]['rgb'])
            else:
                row_frames2.append(images[cam]['rgb'])
        img_list_2h = [row_frames1, row_frames2]
        concatenatedFrames = cv2.vconcat([cv2.hconcat(im_list_h) for im_list_h in img_list_2h])
    elif 1 < len(cameras) < 4:
        if gt2d_pedestrians is not None:
            for i, cam in enumerate(cameras):
                frame_index_key = str(frame_index)
                frame_index_key = frame_index_key.zfill(4)

                info_pedestrians = gt2d_pedestrians[cam][frame_index_key]
                images = draw_bboxes(info_pedestrians, images, cam, dict_names)
        img_list_2h = [images[cam]['rgb'] for cam in cameras]
        concatenatedFrames = cv2.hconcat(img_list_2h)
    else:
        frame_index_key = str(frame_index)
        frame_index_key = frame_index_key.zfill(4)
        if gt2d_pedestrians is not None:
            info_pedestrians = gt2d_pedestrians[cameras[0]][frame_index_key]
            images = draw_bboxes(info_pedestrians, images, cameras[0], dict_names)
        concatenatedFrames = images[cameras[0]]['rgb']
    cv2.putText(concatenatedFrames, str(frame_index), (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.imshow('Frames', concatenatedFrames)

    if path_to_save is not None:
        check_path(path_to_save)
        name = str(frame_index)
        name = name.zfill(4)
        final_path = path_to_save + '/' + name + '_gt.png'
        cv2.imwrite(final_path, concatenatedFrames)

    k = cv2.waitKey(33)
    if k == 32:
        cv2.destroyAllWindows()
    elif k == ord('p'):
        cv2.waitKey(0)