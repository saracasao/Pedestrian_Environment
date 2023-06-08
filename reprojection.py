import numpy as np
from geometry_utils import Point2D, Bbox, Cylinder, Point3D, f_add, f_subtract_ground, f_add_ground, f_euclidian_image


def to3d(depth_px, point, width, height, rot_matrix, translation, f_px):
    # From pixel image coordinates to relative 3d camera coordinates
    coord_img_x, coord_img_y = point.getAsXY()
    coord_3d_y = (coord_img_x - width / 2.) * depth_px / f_px
    coord_3d_z = (coord_img_y - height / 2.) * depth_px / f_px
    coord_3d_x = depth_px

    coord = [[coord_3d_x],
             [coord_3d_y],
             [coord_3d_z],
             [1]]

    # From relative coordinates to global
    transformation_gr = np.append(rot_matrix, translation, axis=1)
    transformation_gr = np.append(transformation_gr, [[0,0,0,1]], axis=0)

    assert np.shape(transformation_gr)[0] == 4 and np.shape(transformation_gr)[1] == 4
    global_coord3d = np.dot(transformation_gr, coord)

    coord_3d_x = global_coord3d[0][0]
    coord_3d_y = global_coord3d[1][0]
    coord_3d_z = global_coord3d[2][0]
    return Point3D(coord_3d_x, coord_3d_y, coord_3d_z)


def from3d(cx, cy, cz, rot_matrix, translation, width, height, f_px):
    # f_px = 320
    transformation_gr = np.append(rot_matrix, translation, axis=1)
    transformation_gr = np.append(transformation_gr, [[0,0,0,1]], axis=0)
    transformation_gr_inv = np.linalg.inv(transformation_gr)
    assert np.shape(transformation_gr)[0] == 4 and np.shape(transformation_gr)[1] == 4

    global_point = [[cx],[cy],[cz], [1]]
    # From 3d global coordinates to relative 3d coordinates
    rel_3dpos = np.dot(transformation_gr_inv, global_point)

    # From relative 3d coordinates to image pixel coordinates
    px_coord_x = rel_3dpos[1][0] / rel_3dpos[0][0]
    px_coord_x = int(px_coord_x * f_px + (width / 2.))

    px_coord_y = rel_3dpos[2][0] / rel_3dpos[0][0]
    px_coord_y = int(px_coord_y * f_px + (height / 2.))
    return Point2D(px_coord_x, px_coord_y), rel_3dpos[0][0]


def from3dCylinder(cam_state, f_px, depth_matrix, cylinder):
    """
    Converts a cylinder in floor plane to a bbox from the specified camera image coordinates
    """
    height_img, width_img = depth_matrix.shape[0], depth_matrix.shape[1]
    translation, rot_matrix = cam_state

    center = cylinder.getCenter()
    # 1. Projection of cylinder center
    cx, cy, cz, cwidth, cheight = cylinder.getXYZWH()
    bottom, depth = from3d(cx, cy, cz, rot_matrix, translation, width_img, height_img, f_px)

    if depth > 0:
        # 2. Add a horizontal vector in the image plane and reproject to 3d
        vector_2d = f_add(bottom, Point2D(10, 0))
        vector_3d = to3d(depth, vector_2d, width_img, height_img, rot_matrix, translation, f_px)

        # 3. Compute 3d normalized vector to the width of the cylinder
        vector_3d_norm = f_subtract_ground(vector_3d, center).normalize(cwidth)
        # 4. Add vector 3d to cylinder center and reproject to image
        wx, wy, wz = f_add_ground(vector_3d_norm, center).getXYZ()
        feet_edge2d, depth = from3d(wx, wy, wz, rot_matrix, translation, width_img, height_img, f_px)

        # 5. Compute distance between bbox bottom and the edge
        width = f_euclidian_image(bottom, feet_edge2d) * 2

        # Compute of the height
        hx, hy, hheight = cylinder.getHair().getXYZ()
        head_point, depth = from3d(hx, hy, hheight, rot_matrix, translation, width_img, height_img, f_px)
        height = bottom.getAsXY()[1] - head_point.getAsXY()[1]
        bbox = Bbox.FeetWH(bottom, width, height)
    else:
        bottom = Point2D(0,0)
        width, height = 1, 1
        bbox = Bbox.FeetWH(bottom, width, height)
    return bbox


def check_bbox(xmin, ymin, xmax, ymax, frame_width, frame_height):
    """Check and adapt ground truth bounding box inside the frame dimensions"""
    check = True
    # check width coordinates
    if xmax < 0:
        check = False
    elif xmin > frame_width:
        check = False
    elif xmax > 0 > xmin:
        xmin = 0
    elif xmin < frame_width < xmax:
        xmax = frame_width

    # check height coordinates
    if ymax < 0:
        check = False
    elif ymin > frame_height:
        check = False
    elif ymax > 0 > ymin:
        ymin = 0
    elif ymin < frame_height < ymax:
        ymax = frame_height

    height = ymax - ymin
    width = xmax - xmin
    if height < 2 or width < 2:
        check = False
    return check, xmin, ymin, xmax, ymax


def gt2d_from_gt3d(cam_name, gt2d_iteration, gt3d_iteration, depth_matrix, config, uav=None):
    """
    :param cam_name (str): camera name
           gt2d_iteration: ground truth capture by AirSim in 2D: list of dict = {'id': name_pedestrian, 'xmin': xmin_bbox, 'ymin': ymin_bbox, 'xmax': xmax_bbox, 'ymax': ymax_bbox}
           gt3d_iteration: ground truth capture by AirSim in 3D: list of dict = {'id': name_pedestrian, 'pos_x': 3d coord x, 'pos_y': 3d coord y, 'pos_z': 3d coord z}
           depth_matrix: array H X W  [0,100000]
           config: Configuration Class
    """
    width = 0.58
    height = 1.75
    if uav is not None:
        cam_state = uav.extrinsics_camera
        f_px = uav.cam_states[-1]['f_fov']
    else:
        cam_state = config.get_pose(cam_name)
        f_px = config.get_focal_length_px(cam_name, depth_matrix)
    frame_height, frame_width = depth_matrix.shape[0], depth_matrix.shape[1]

    ped_scene = [gt3d['id'] for gt3d in gt3d_iteration]
    ped_detected = [gt2d['id'] for gt2d in gt2d_iteration]
    ped_to_detect = list(set(ped_scene) ^ set(ped_detected))

    ped_3d_info = {gt3d['id']: [gt3d['pos_x'], gt3d['pos_y'], gt3d['pos_z']] for gt3d in gt3d_iteration if
                   gt3d['id'] in ped_to_detect}
    info_2d = []
    for ped in ped_to_detect:
        x, y, z = ped_3d_info[ped]
        z = z + (height / 2)

        cylinder = Cylinder.XYZWH(x, y, z, width, height)
        bbox = from3dCylinder(cam_state, f_px, depth_matrix, cylinder)
        xmin, ymin, xmax, ymax = bbox.getAsXmYmXMYM()
        check, xmin, ymin, xmax, ymax = check_bbox(xmin, ymin, xmax, ymax, frame_width, frame_height)

        if check:
            info = {'id': ped,
                    'xmin': int(xmin),
                    'ymin': int(ymin),
                    'xmax': int(xmax),
                    'ymax': int(ymax)}
            info_2d.append(info)
    return info_2d
