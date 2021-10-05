from sklearn.neighbors import KDTree
from os.path import join, exists, dirname, abspath
import numpy as np
import os, glob, pickle
import sys

BASE_DIR = dirname(abspath(__file__))
ROOT_DIR = dirname(BASE_DIR)
sys.path.append(BASE_DIR)
sys.path.append(ROOT_DIR)
from plyfile import PlyData, PlyElement
from helper_ply import write_ply
from helper_tool import DataProcessing as DP


################################### UTILS Functions #######################################
def read_ply_with_plyfilelib(filename):
    """convert from a ply file. include the label and the object number"""
    # ---read the ply file--------
    plydata = PlyData.read(filename)
    xyz = np.stack([plydata['vertex'][n] for n in ['x', 'y', 'z']], axis=1)
    try:
        rgb = np.stack([plydata['vertex'][n]
                        for n in ['red', 'green', 'blue']]
                       , axis=1).astype(np.uint8)
    except ValueError:
        rgb = np.stack([plydata['vertex'][n]
                        for n in ['r', 'g', 'b']]
                       , axis=1).astype(np.float32)
    if np.max(rgb) > 1:
        rgb = rgb
    try:
        object_indices = plydata['vertex']['object_index']
        labels = plydata['vertex']['label']
        return xyz, rgb, labels, object_indices
    except ValueError:
        try:
            labels = plydata['vertex']['label']
            return xyz, rgb, labels
        except ValueError:
            return xyz, rgb


grid_size = 0.01
dataset_path = ROOT_DIR + '/data/raw'
original_pc_folder = join(dirname(dataset_path), 'original_ply')
sub_pc_folder = join(dirname(dataset_path), 'input_{:.3f}'.format(grid_size))
os.mkdir(original_pc_folder) if not exists(original_pc_folder) else None
os.mkdir(sub_pc_folder) if not exists(sub_pc_folder) else None
label_statistics = np.zeros((6,), dtype=int)
min_input_points = np.iinfo(np.int32(10)).max
folders = ["train/", "test/", "validate/"]
print("Prepare urbanmesh point cloud ...")
for folder in folders:
    print("=================\n   " + folder + "\n=================")
    data_folder = dataset_path + "/" + folder
    files = glob.glob(data_folder + "*.ply")
    n_files = len(files)
    i_file = 1
    for file in files:
        file_name = os.path.splitext(os.path.basename(file))[0]
        print(str(i_file) + " / " + str(n_files) + "---> " + file_name)
        i_file += 1
        # check if it has already calculated
        if exists(join(sub_pc_folder, file_name + '_KDTree.pkl')):
            continue

        xyz, rgb, rawlabels = read_ply_with_plyfilelib(file)
        labels = np.array([x if x > 0 else x + 1 for x in rawlabels])
        rgb = 255 * rgb  # Now scale by 255
        full_ply_path = join(original_pc_folder, file_name + '.ply')

        #  Subsample to save space, save sub_cloud and KDTree file
        sub_points, sub_colors, sub_labels = DP.grid_sub_sampling(xyz[:, :].astype(np.float32),
                                                                  rgb[:, :].astype(np.uint8), labels.astype(np.uint8), grid_size)

        sub_colors = sub_colors / 255.0
        sub_labels = np.squeeze(sub_labels)
        sub_ply_file = join(sub_pc_folder, file_name + '.ply')
        write_ply(sub_ply_file, [sub_points, sub_colors, sub_labels], ['x', 'y', 'z', 'red', 'green', 'blue', 'class'])
        write_ply(full_ply_path, [sub_points, sub_colors, sub_labels], ['x', 'y', 'z', 'red', 'green', 'blue', 'class'])
        if folder != "test/":
            for sub_l in sub_labels:
                if sub_l != 0:
                    label_statistics[sub_l - 1] += 1
            if len(sub_xyz) < min_input_points:
                min_input_points = len(sub_xyz)

        search_tree = KDTree(sub_xyz, leaf_size=50)
        kd_tree_file = join(sub_pc_folder, file_name + '_KDTree.pkl')
        with open(kd_tree_file, 'wb') as f:
            pickle.dump(search_tree, f)

        proj_idx = np.squeeze(search_tree.query(sub_points, return_distance=False))
        proj_idx = proj_idx.astype(np.int32)
        proj_save = join(sub_pc_folder, file_name + '_proj.pkl')
        with open(proj_save, 'wb') as f:
            pickle.dump([proj_idx, labels], f)

print("Minimum input points per tile: " + str(min_input_points))
print("Num pts per class: " + str(label_statistics))