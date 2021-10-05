import os
import os.path as osp
from itertools import repeat, product
import numpy as np
import h5py
import torch
import random
import glob
from plyfile import PlyData, PlyElement
import logging

from torch_geometric.data import Dataset, Data
from torch_points3d.datasets.base_dataset import BaseDataset
from torch_points3d.metrics.segmentation_tracker import SegmentationTracker

################################### Parameters Setting #######################################
DIR = os.path.dirname(os.path.realpath(__file__))
log = logging.getLogger(__name__)
SPLITS = ["train", "val", "test"]
NUM_CLASSES = 6
CLASS_LABELS = (
    "ground",
    "vegetation",
    "building",
    "water",
    "car",
    "boat",
)
COLOR_MAP = {
    0: (0, 0, 0),# unlabelled .->. black
    1: (170, 85, 0), # 'ground' -> brown
    2: (0, 255, 0),  # 'vegetation' -> green
    3: (255, 255, 0),# 'building' -> yellow
    4: (0, 255, 255),# 'water' -> blue
    5: (255, 0, 255),# 'vehicle'/'car' -> pink
    6: (0, 0, 153),  # 'boat' -> purple
}

################################### UTILS Functions #######################################
def read_ply(filename):
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

def write_ply(filename, xyz, rgb):
    """write into a ply file"""
    prop = [('x', 'f4'), ('y', 'f4'), ('z', 'f4'), ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')]
    vertex_all = np.empty(len(xyz), dtype=prop)
    for i_prop in range(0, 3):
        vertex_all[prop[i_prop][0]] = xyz[:, i_prop]
    for i_prop in range(0, 3):
        vertex_all[prop[i_prop + 3][0]] = rgb[:, i_prop]
    ply = PlyData([PlyElement.describe(vertex_all, 'vertex')], text=True)
    ply.write(filename)


################################### Dataset Class ###################################

class UrbanMesh(Dataset):
    SPLITS = SPLITS
    CLASS_LABELS = CLASS_LABELS
    COLOR_MAP = COLOR_MAP
    num_classes = len(CLASS_LABELS)

    def __init__(self, root, split="train", transform=None, pre_transform=None):
        self.root_dir = root
        self.split = split
        super().__init__(root, transform=transform, pre_transform=pre_transform)

    @property
    def raw_file_names(self):
        rawfile_names = []
        data_folder = self.raw_dir + "/" + self.split + "/"
        rawfile_names.append(glob.glob(data_folder + "*.ply"))
        return rawfile_names

    @property
    def processed_file_names(self):
        processed_file_names = []
        raw_folder = self.raw_dir + "/" + self.split + "/"
        processed_folder = self.processed_dir + "/" + self.split + "/"
        if not os.path.isdir(processed_folder):
            os.mkdir(processed_folder)
        files = glob.glob(raw_folder + "*.ply")
        for file in files:
            file_name = os.path.splitext(os.path.basename(file))[0]
            data_file = processed_folder + file_name + '.pt'
            processed_file_names.append(data_file)
        return processed_file_names

    def len(self):
        return len(self.processed_file_names)

    def get(self, idx):
        data = torch.load(self.processed_paths[idx])
        return data

    #convert *.ply to *.pt (PyG data format)
    def process(self):
        print("=================\n   " + self.split + "\n=================")
        data_folder = self.raw_dir + "/" + self.split + "/"
        files = glob.glob(data_folder + "*.ply")
        n_files = len(files)
        i_file = 0
        path = []
        for file in files:
            i_file += 1
            file_name = os.path.splitext(os.path.basename(file))[0]
            # adapt to your hierarchy. The following 4 files must be defined
            data_file = data_folder + file_name + '.ply'  # or .las
            print(str(i_file) + " / " + str(n_files) + "---> " + file_name)
            xyz, rgb, labels = read_ply(data_file)
            # check for invalid labels
            labels_ = np.array([x if x < 0 else x-1 for x in labels])
            #convert from numpy arrays to torch tensors
            xyz = torch.from_numpy(xyz)
            rgb = torch.from_numpy(rgb)
            labels_ = torch.from_numpy(labels_).long() #torch.tensor(torch.from_numpy(labels_), dtype=torch.long)
            data = Data(pos=xyz, y=labels_, rgb=rgb)

            if self.pre_filter is not None and not self.pre_filter(data):
                continue
            if self.pre_transform is not None:
                data = self.pre_transform(data)

            path = self.processed_dir + "/" + self.split + "/"
            torch.save(data, osp.join(path, '{}.pt'.format(file_name)))

class UrbanMeshDataset(BaseDataset):
    def __init__(self, dataset_opt):
        super().__init__(dataset_opt)

        self.train_dataset = UrbanMesh(
            self._data_path,
            split="train",
            transform=self.val_transform,
            pre_transform=self.pre_transform,
        )

        self.val_dataset = UrbanMesh(
            self._data_path,
            split="validate",
            transform=self.val_transform,
            pre_transform=self.pre_transform,
        )

        self.test_dataset = UrbanMesh(
            self._data_path,
            split="test",
            transform=self.val_transform,
            pre_transform=self.pre_transform,
        )

    def get_tracker(self, wandb_log: bool, tensorboard_log: bool):
        """Factory method for the tracker

        Arguments:
            wandb_log - Log using weight and biases
            tensorboard_log - Log using tensorboard
        Returns:
            [BaseTracker] -- tracker
        """
        return SegmentationTracker(self, wandb_log=wandb_log, use_tensorboard=tensorboard_log)



