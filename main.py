import numpy as np
import open3d as o3d
from plyfile import PlyData


def get_ply_data(input_file):
    plydata = PlyData.read(input_file)
    pc = plydata['vertex'].data
    angles = pc['scalar_ScanAngleRank']
    angles = np.asarray(angles, dtype=np.int)
    labels = pc['scalar_Label']
    labels = np.asarray(labels, dtype=np.ubyte)
    return angles, labels, pc


def filter_by_input_data(data, max_height, min_angle):
    x, y, z, angles = data
    height_mask = z < max_height
    angles_mask = angles > min_angle
    mask = height_mask & angles_mask
    return (x[mask],
            y[mask],
            z[mask],
            angles[mask])


def filter_by_height_diff(data, min_diff):
    x, y, z, angles = data
    height_diff = np.zeros_like(z)
    for i in range(len(z)):
        if i == 0:
            height_diff[i] = 0
        height_diff[i] = z[i] - z[i - 1]
    mask = height_diff > min_diff
    return (x[mask],
            y[mask],
            z[mask],
            angles[mask])


def process_cloud(pc, angles, labels):
    UTM_OFFSET = [627285, 4841948, 0]
    x = pc['x'] - UTM_OFFSET[0]
    y = pc['y'] - UTM_OFFSET[1]
    z = pc['z']
    labels_1 = np.where(labels == 1)

    x = x[labels_1]
    y = y[labels_1]
    z = z[labels_1]
    angles = angles[labels_1]

    max_z = z.min() + 1.2
    min_angle = 15
    x, y, z, angles = filter_by_input_data((x, y, z, angles),
                                           max_z, min_angle)

    min_diff = 0.02
    x, y, z, angles = filter_by_height_diff((x, y, z, angles),
                                            min_diff)

    draw_pc(angles, x, y, z)


def draw_pc(angles, x, y, z):
    ground_array = np.vstack((x, y, z)).transpose()
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(ground_array))
    cloud.colors = o3d.utility.Vector3dVector(np.vstack(((z - 133.8),
                                                         np.zeros(len(angles)),
                                                         np.zeros(len(angles))))
                                              .transpose()
                                              )
    obb = cloud.get_oriented_bounding_box()
    obb.color = (0, 1, 0)
    o3d.visualization.draw_geometries([cloud, obb])


if __name__ == '__main__':
    input_file = "/home/egor/workspace/python_ws/Toronto_3D/L004.ply"

    angles, labels, pc = get_ply_data(input_file)

    process_cloud(pc, angles, labels)
