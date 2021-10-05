"""
    Large-scale Point Cloud Semantic Segmentation with Superpoint Graphs
    http://arxiv.org/abs/1711.09869
    2017 Loic Landrieu, Martin Simonovsky
"""
from __future__ import division
from __future__ import print_function
from builtins import range

import sys

sys.path.append("./learning")

import random
import numpy as np
import os
import functools
import torch
import torchnet as tnt
import h5py
import spg

from sklearn.linear_model import RANSACRegressor


def get_datasets(args, test_seed_offset=0):
    """build training and testing set"""

    # for a simple train/test organization
    validset = ['validate/' + f for f in os.listdir(args.CUSTOM_SET_PATH + '/superpoint_graphs/validate')]
    trainset = ['train/' + f for f in os.listdir(args.CUSTOM_SET_PATH + '/superpoint_graphs/train')]
    testset = ['test/' + f for f in os.listdir(args.CUSTOM_SET_PATH + '/superpoint_graphs/test')]

    # Load superpoints graphs
    testlist, trainlist, validlist = [], [], []
    for n in trainset:
        trainlist.append(spg.spg_reader(args, args.CUSTOM_SET_PATH + '/superpoint_graphs/' + n, True))
    for n in validset:
        validlist.append(spg.spg_reader(args, args.CUSTOM_SET_PATH + '/superpoint_graphs/' + n, True))
    for n in testset:
        testlist.append(spg.spg_reader(args, args.CUSTOM_SET_PATH + '/superpoint_graphs/' + n, True))

    # Normalize edge features
    if args.spg_attribs01:
        trainlist, testlist, validlist, scaler = spg.scaler01(trainlist, testlist, validlist=validlist)

    return tnt.dataset.ListDataset([spg.spg_to_igraph(*tlist) for tlist in trainlist],
                                    functools.partial(spg.loader, train=True, args=args, db_path=args.CUSTOM_SET_PATH)), \
           tnt.dataset.ListDataset([spg.spg_to_igraph(*tlist) for tlist in testlist],
                                    functools.partial(spg.loader, train=False, args=args, db_path=args.CUSTOM_SET_PATH, test_seed_offset=test_seed_offset)), \
           tnt.dataset.ListDataset([spg.spg_to_igraph(*tlist) for tlist in validlist],
                                    functools.partial(spg.loader, train=False, args=args, db_path=args.CUSTOM_SET_PATH, test_seed_offset=test_seed_offset)), \
            scaler

def get_info(args):
    edge_feats = 0
    for attrib in args.edge_attribs.split(','):
        a = attrib.split('/')[0]
        if a in ['delta_avg', 'delta_std', 'xyz']:
            edge_feats += 3
        else:
            edge_feats += 1

    if args.loss_weights == 'none':
        weights = np.ones((6,), dtype='f4')
        #weights = np.ones((5,), dtype='f4')
    else:
        weights = h5py.File(args.CUSTOM_SET_PATH + "/parsed/class_count.h5")["class_count"][:].astype('f4')
        weights = weights[:, [i for i in range(6) if i != args.cvfold - 1]].sum(1)
        weights = weights.mean() / weights
    if args.loss_weights == 'sqrt':
        weights = np.sqrt(weights)
    weights = torch.from_numpy(weights).cuda() if args.cuda else torch.from_numpy(weights)

    return {
        'node_feats': len(args.pc_attribs),
        'edge_feats': edge_feats,
        'class_weights': weights,
        'classes': 6,  # CHANGE TO YOUR NUMBER OF CLASS
        'inv_class_map': {0: 'ground', 1: 'vegetation', 2: 'building', 3: 'water', 4: 'car', 5: 'boat'},  # etc...
    }


def preprocess_pointclouds(CUSTOM_SET_PATH):
    """ Preprocesses data by splitting them by components and normalizing."""

    for n in ['train', 'test','validate']:
        pathP = '{}/parsed/{}/'.format(CUSTOM_SET_PATH, n)
        pathD = '{}/features/{}/'.format(CUSTOM_SET_PATH, n)
        pathC = '{}/superpoint_graphs/{}/'.format(CUSTOM_SET_PATH, n)
        if not os.path.exists(pathP):
            os.makedirs(pathP)
        random.seed(0)

        for file in os.listdir(pathC):
            print(file)
            if file.endswith(".h5"):
                f = h5py.File(pathD + file, 'r')
                xyz = f['xyz'][:]
                rgb = f['rgb'][:].astype(np.float)
                P = np.zeros(1)
				
                elpsv = np.concatenate((f['xyz'][:, 2][:, None], f['geof'][:]), axis=1)
				# rescale to [-0.5,0.5]; keep xyz
				# warning - to use the trained model, make sure the elevation is comparable
				# to the set they were trained on
				# i.e. ~0 for roads and ~0.2-0.3 for builings for sema3d
				# and -0.5 for floor and 0.5 for ceiling for s3dis
                elpsv[:, 0] /= 100  # (rough guess) #adapt
                elpsv[:, 1:] -= 0.5
                rgb = rgb / 255.0 - 0.5
                P = np.concatenate([xyz, rgb, elpsv], axis=1)
  
                f = h5py.File(pathC + file, 'r')
                numc = len(f['components'].keys())

                with h5py.File(pathP + file, 'w') as hf:
                    for c in range(numc):
                        idx = f['components/{:d}'.format(c)][:].flatten()
                        if idx.size > 10000:  # trim extra large segments, just for speed-up of loading time
                            ii = random.sample(range(idx.size), k=10000)
                            idx = idx[ii]

                        hf.create_dataset(name='{:d}'.format(c), data=P[idx, ...])


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Large-scale Point Cloud Semantic Segmentation with Superpoint Graphs')
    parser.add_argument('--CUSTOM_SET_PATH', default='../datasets/custom_set')
    args = parser.parse_args()
    preprocess_pointclouds(args.CUSTOM_SET_PATH)


