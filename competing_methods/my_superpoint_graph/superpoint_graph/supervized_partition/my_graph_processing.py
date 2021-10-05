import os
import sys
import glob
import numpy as np
import h5py
import random
import torch
import torch.nn as nn
import torch.nn.init as init
import transforms3d
import math
import igraph
import argparse
from timeit import default_timer as timer
import torchnet as tnt
import functools
import argparse
from sklearn.linear_model import RANSACRegressor
from plyfile import PlyData, PlyElement

DIR_PATH = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(DIR_PATH, '..'))
sys.path.insert(0, DIR_PATH)
sys.path.append(os.path.join(DIR_PATH,"../partition/cut-pursuit/build/src"))

import libply_c
import libcp

from learning.spg import augment_cloud
from partition.graphs import *
from partition.my_provider import *

def main():
    parser = argparse.ArgumentParser(description='Large-scale Point Cloud Semantic Segmentation with Superpoint Graphs')
    
    parser.add_argument('--ROOT_PATH', default='../datasets/custom_set')
    parser.add_argument('--dataset', default='custom_dataset')
    #parameters
    parser.add_argument('--compute_geof', default=1, type=int, help='compute hand-crafted features of the local geometry')

    ####################Change param########################## default=40
    parser.add_argument('--k_nn_local', default=45, type=int, help='number of neighbors to describe the local geometry')
    ####################Change param########################## default=5
    parser.add_argument('--k_nn_adj', default=10, type=int, help='number of neighbors for the adjacency graph')

    parser.add_argument('--voxel_width', default=0.01, type=float, help='voxel size when subsampling (in m)')
    parser.add_argument('--plane_model', default=1, type=int, help='uses a simple plane model to derive elevation')
    ####################Change param########################## default=0.0
    parser.add_argument('--use_voronoi', default=0.0, type=float, help='uses the Voronoi graph in combination to knn to build the adjacency graph, useful for sparse aquisitions. If 0., do not use voronoi. If >0, then is the upper length limit for an edge to be kept. ')
    parser.add_argument('--ver_batch', default=5000000, type=int, help='batch size for reading large files')
    args = parser.parse_args()
    
    #path to data
    if args.ROOT_PATH[-1]=='/':
        root = args.ROOT_PATH
    else:
        root = args.ROOT_PATH+'/'
        
    if not os.path.exists(root + 'features_supervision'):
        os.makedirs(root + 'features_supervision')
    
    #list of subfolders to be processed
    folders = ["train/", "test/"]
    n_labels = 5 #number of classes

    pruning = args.voxel_width > 0
    #------------------------------------------------------------------------------
    for folder in folders:
        print("=================\n   "+folder+"\n=================")
        data_folder = root + "data/"              + folder
        str_folder  = root + "features_supervision/"  + folder
        
        if not os.path.isdir(data_folder):
            raise ValueError("%s does not exist" % data_folder)
           # os.mkdir(data_folder)
        if not os.path.isdir(str_folder):
            os.mkdir(str_folder)
            
        files = glob.glob(data_folder + "*.ply")
            
        if (len(files) == 0):
            continue
            #raise ValueError('%s is empty' % data_folder)
        n_files = len(files)
        i_file = 0
        for file in files:
            file_name = os.path.splitext(os.path.basename(file))[0]

            data_file   = data_folder + file_name + ".ply"
            str_file    = str_folder  + file_name + '.h5'

            i_file = i_file + 1
            print(str(i_file) + " / " + str(n_files) + "---> "+file_name)
            if os.path.isfile(str_file):
                print("    graph structure already computed - delete for update...")
            else:
                #--- build the geometric feature file h5 file ---
                print("    computing graph structure...")
                #--- read the data files and compute the labels---
                xyz, rgb, labels, objects = read_ply(data_file)

                    #---compute nn graph-------
                n_ver = xyz.shape[0]    
                print("computing NN structure")
                graph_nn, local_neighbors = compute_graph_nn_2(xyz, args.k_nn_adj, args.k_nn_local, voronoi = args.use_voronoi)

                is_transition = objects[graph_nn["source"]]!=objects[graph_nn["target"]]

                if (args.compute_geof):
                    geof = libply_c.compute_geof(xyz, local_neighbors, args.k_nn_local).astype('float32')
                    geof[:,3] = 2. * geof[:,3]
                else:
                    geof = 0
                
                if args.plane_model: #use a simple palne model to the compute elevation
                    low_points = ((xyz[:,2]-xyz[:,2].min() < 0.5)).nonzero()[0]
                    reg = RANSACRegressor(random_state=0).fit(xyz[low_points,:2], xyz[low_points,2])
                    elevation = xyz[:,2]-reg.predict(xyz[:,:2])
                else:
                    elevation = xyz[:,2] - xyz[:,2].min()
                
                #compute the xy normalized position
                ma, mi = np.max(xyz[:,:2],axis=0,keepdims=True), np.min(xyz[:,:2],axis=0,keepdims=True)
                xyn = (xyz[:,:2] - mi) / (ma - mi + 1e-8) #global position
                    
                write_structure(str_file, xyz, rgb, graph_nn, local_neighbors.reshape([n_ver, args.k_nn_local]), \
                    is_transition, labels, objects, geof, elevation, xyn)
                    
#------------------------------------------------------------------------------

#------------------------------------------------------------------------------
def write_structure(file_name, xyz, rgb, graph_nn, target_local_geometry, is_transition, labels, objects, geof, elevation, xyn):
    """
    save the input point cloud in a format ready for embedding    
    """
    #store transition and non-transition edges in two different contiguous memory blocks
    #n_transition = np.count_nonzero(is_transition)
    #blocks = np.hstack((np.where(is_transition),np.where(is_transition==False)))
    
    data_file = h5py.File(file_name, 'w')
    data_file.create_dataset('xyz', data=xyz, dtype='float32')
    data_file.create_dataset('rgb', data=rgb, dtype='float32')
    data_file.create_dataset('elevation', data=elevation, dtype='float32')
    data_file.create_dataset('xyn', data=xyn, dtype='float32')
    data_file.create_dataset('source', data=graph_nn["source"], dtype='int')
    data_file.create_dataset('target', data=graph_nn["target"], dtype='int')
    data_file.create_dataset('is_transition', data=is_transition, dtype='uint8')
    data_file.create_dataset('target_local_geometry', data=target_local_geometry, dtype='uint32')
    data_file.create_dataset('objects', data=objects, dtype='uint32')
    if (len(geof)>0):        
        data_file.create_dataset('geof', data=geof, dtype='float32')
    if len(labels) > 0 and len(labels.shape)>1 and labels.shape[1]>1:
        data_file.create_dataset('labels', data=labels, dtype='int32')
    else:
        data_file.create_dataset('labels', data=labels, dtype='uint8')

#------------------------------------------------------------------------------
def read_structure(file_name, read_geof):
    """
    read the input point cloud in a format ready for embedding    
    """
    data_file = h5py.File(file_name, 'r')
    xyz = np.array(data_file['xyz'], dtype='float32')
    rgb = np.array(data_file['rgb'], dtype='float32')
    elevation = np.array(data_file['elevation'], dtype='float32')
    xyn = np.array(data_file['xyn'], dtype='float32')
    edg_source = np.array(data_file['source'], dtype='int').squeeze()
    edg_target = np.array(data_file['target'], dtype='int').squeeze()
    is_transition = np.array(data_file['is_transition'])
    objects = np.array(data_file['objects'][()])
    labels = np.array(data_file['labels']).squeeze()
    if len(labels.shape) == 0:#dirty fix
        labels = np.array([0])
    if len(is_transition.shape) == 0:#dirty fix
        is_transition = np.array([0])
    if read_geof: #geometry = geometric features
        local_geometry = np.array(data_file['geof'], dtype='float32')
    else: #geometry = neighborhood structure
        local_geometry = np.array(data_file['target_local_geometry'], dtype='uint32')
    
    return xyz, rgb, edg_source, edg_target, is_transition, local_geometry, labels, objects, elevation, xyn
#------------------------------------------------------------------------------
def get_custom_dataset_info(args):
    #for now, no edge attributes
    return {
        'classes': 5,
        'inv_class_map': {1:'ground', 2:'vegetation', 3:'building', 4:'water', 5:'vehicle'},
    }

def create_custom_datasets(args, test_seed_offset=0):
    """ Gets training and test datasets. """
    #for a simple train/test organization
    # trainset = ['train/' + f for f in os.listdir(args.CUSTOM_SET_PATH + '/features_supervision/train')]
    # testset  = ['test/' + f for f in os.listdir(args.CUSTOM_SET_PATH + '/features_supervision/test')]

    path = '{}/features_supervision/'.format(args.ROOT_PATH)
    # Load superpoints graphs
    testlist, trainlist = [], []
    if args.db_train_name == 'train':
        trainlist = [f for f in  glob.glob(path + 'train/*.h5')]

    if args.db_test_name == 'test':
        testlist = [f for f in  glob.glob(path + 'test/*.h5')]

    return tnt.dataset.ListDataset(trainlist,
                                   functools.partial(graph_loader, train=True, args=args, db_path=args.ROOT_PATH)), \
           tnt.dataset.ListDataset(testlist,
                                   functools.partial(graph_loader, train=False, args=args, db_path=args.ROOT_PATH,
                                                     full_cpu=True))

def subgraph_sampling(n_ver, edg_source, edg_target, max_ver):
    """ Select a subgraph of the input graph of max_ver verices"""
    return libply_c.random_subgraph(n_ver)

def graph_loader(entry, train, args, db_path, test_seed_offset=0, full_cpu = False):
    """ Load the point cloud and the graph structure """
    xyz, rgb, edg_source, edg_target, is_transition, local_geometry\
        , labels, objects, elevation, xyn = read_structure(entry, 'geof' in args.ver_value)
    short_name= entry.split(os.sep)[-2]+'/'+entry.split(os.sep)[-1]

    rgb = rgb/255

    n_ver = np.shape(xyz)[0]
    n_edg = np.shape(edg_source)[0]
    
    selected_ver = np.full((n_ver,), True, dtype='?')
    selected_edg = np.full((n_edg,), True, dtype='?')
    
    if train:
        xyz, rgb = augment_cloud_whole(args, xyz, rgb)
        
    subsample = False
    new_ver_index = []

    if train and (0<args.max_ver_train<n_ver):
        
        subsample = True
            
        selected_edg, selected_ver = libply_c.random_subgraph(n_ver, edg_source.astype('uint32'), edg_target.astype('uint32'), int(args.max_ver_train))
        selected_edg = selected_edg.astype('?')
        selected_ver = selected_ver.astype('?')
            
        new_ver_index = -np.ones((n_ver,), dtype = int)
        new_ver_index[selected_ver.nonzero()] = range(selected_ver.sum())
            
        edg_source = new_ver_index[edg_source[selected_edg.astype('?')]]
        edg_target = new_ver_index[edg_target[selected_edg.astype('?')]]
            
        is_transition = is_transition[selected_edg]
        labels = labels[selected_ver,]
        objects = objects[selected_ver,]
        elevation = elevation[selected_ver]
        xyn = xyn[selected_ver,]
       
    if args.learned_embeddings:
        #we use point nets to embed the point clouds
        nei = local_geometry[selected_ver,:args.k_nn_local].astype('int64')
        
        clouds, clouds_global = [], [] #clouds_global is cloud global features. here, just the diameter + elevation
        
        clouds = xyz[nei,]
        #diameters = np.max(np.max(clouds,axis=1) - np.min(clouds,axis=1), axis = 1)
        diameters = np.sqrt(clouds.var(1).sum(1))
        clouds = (clouds - xyz[selected_ver,np.newaxis,:]) / (diameters[:,np.newaxis,np.newaxis] + 1e-10)
        
        if args.use_rgb:
            clouds = np.concatenate([clouds, rgb[nei,]],axis=2)
        
        clouds = clouds.transpose([0,2,1])
        
        clouds_global = diameters[:,None]
        if 'e' in args.global_feat:
            clouds_global = np.hstack((clouds_global, elevation[:,None]))
        if 'rgb' in args.global_feat:
            clouds_global = np.hstack((clouds_global, rgb[selected_ver,]))
        if 'XY' in args.global_feat:
            clouds_global = np.hstack((clouds_global, xyn))
        if 'xy' in args.global_feat:
            clouds_global = np.hstack((clouds_global, xyz[selected_ver,:2]))
        #clouds_global = np.hstack((diameters[:,None], ((xyz[selected_ver,2] - min_z) / (max_z- min_z)-0.5)[:,None],np.zeros_like(rgb[selected_ver,])))

        #clouds_global = np.vstack((diameters, xyz[selected_ver,2])).T
    elif args.ver_value == 'geofrgb':
        #the embeddings are already computed
        clouds = np.concatenate([local_geometry, rgb[selected_ver,]],axis=1)
        clouds_global = np.array([0])
        nei = np.array([0])
    elif args.ver_value == 'geof':
        #the embeddings are already computed
        clouds = local_geometry
        clouds_global = np.array([0])
        nei = np.array([0])
    
    n_edg_selected = selected_edg.sum()
    
    nei = np.array([0])
    
    xyz = xyz[selected_ver,]
    is_transition = torch.from_numpy(is_transition)
    #labels = torch.from_numpy(labels)
    objects = torch.from_numpy(objects.astype('int64'))
    clouds = torch.from_numpy(clouds)
    clouds_global = torch.from_numpy(clouds_global)
    return short_name, edg_source, edg_target, is_transition, labels, objects, clouds, clouds_global, nei, xyz
#------------------------------------------------------------------------------
#------------------------------------------------------------------------------
def graph_collate(batch):
    """ Collates a list of dataset samples into a single batch
    """
    short_name, edg_source, edg_target, is_transition, labels, objects, clouds, clouds_global, nei, xyz = list(zip(*batch))

    n_batch = len(short_name)
    batch_ver_size_cumsum = np.array([c.shape[0] for c in labels]).cumsum()
    batch_n_edg_cumsum = np.array([c.shape[0] for c in edg_source]).cumsum()
    batch_n_objects_cumsum = np.array([c.max() for c in objects]).cumsum()
    
    
    clouds = torch.cat(clouds, 0)
    clouds_global = torch.cat(clouds_global, 0)
    xyz = np.vstack(xyz)
    #if len(is_transition[0])>1:
    is_transition = torch.cat(is_transition, 0)
    labels = np.vstack(labels)
    
    edg_source = np.hstack(edg_source)
    edg_target = np.hstack(edg_target)
    nei = np.vstack(nei)
    #if len(is_transition[0]>1:
    objects = torch.cat(objects, 0)
    
    for i_batch in range(1,n_batch):
        edg_source[batch_n_edg_cumsum[i_batch-1]:batch_n_edg_cumsum[i_batch]] += int(batch_ver_size_cumsum[i_batch-1])
        edg_target[batch_n_edg_cumsum[i_batch-1]:batch_n_edg_cumsum[i_batch]] += int(batch_ver_size_cumsum[i_batch-1])
        #if len(objects)>1:
        objects[batch_ver_size_cumsum[i_batch-1]:batch_ver_size_cumsum[i_batch],] += int(batch_n_objects_cumsum[i_batch-1])
        non_valid = (nei[batch_ver_size_cumsum[i_batch-1]:batch_ver_size_cumsum[i_batch],]==-1).nonzero()
        nei[batch_ver_size_cumsum[i_batch-1]:batch_ver_size_cumsum[i_batch],] += int(batch_ver_size_cumsum[i_batch-1])
        nei[batch_ver_size_cumsum[i_batch-1]+non_valid[0],non_valid[1]] = -1

    return short_name, edg_source, edg_target, is_transition, labels, objects, (clouds, clouds_global, nei), xyz
#------------------------------------------------------------------------------
def show(clouds,k):
    from mpl_toolkits.mplot3d import Axes3D    
    import matplotlib.pyplot as plt    
    fig = plt.figure()    
    ax = fig.gca(projection='3d') 
    ax.scatter(clouds[k,:,0], clouds[k,:,1], clouds[k,:,2])
    plt.show()

#------------------------------------------------------------------------------
def read_embeddings(file_name):
    """
    read the input point cloud in a format ready for embedding    
    """
    data_file = h5py.File(file_name, 'r')
    if 'embeddings' in data_file:
        embeddings = np.array(data_file['embeddings'], dtype='float32')
    else:
        embeddings = []
    if 'edge_weight' in data_file:
        edge_weight = np.array(data_file['edge_weight'], dtype='float32')
    else:
        edge_weight = []
    return embeddings, edge_weight
#------------------------------------------------------------------------------
def write_embeddings(file_name, args, embeddings, edge_weight=[]):
    """
    save the embeddings and the edge weights 
    """
    folder = args.ROOT_PATH + '/embeddings' + args.suffix + '/' +file_name.split('/')[0]
    if not os.path.isdir(folder):
        os.mkdir(folder)
    file_path = args.ROOT_PATH + '/embeddings' + args.suffix + '/' + file_name
    if os.path.isfile(file_path):
        data_file = h5py.File(file_path, 'r+')
    else:
        data_file = h5py.File(file_path, 'w')
    if len(embeddings)>0  and not 'embeddings' in data_file:
        data_file.create_dataset('embeddings'
                                 , data=embeddings, dtype='float32')
    elif len(embeddings)>0:
            data_file['embeddings'][...] = embeddings
            
    if len(edge_weight)>0 and not 'edge_weight' in data_file:
        data_file.create_dataset('edge_weight', data=edge_weight, dtype='float32')
    elif len(edge_weight)>0:
            data_file['edge_weight'][...] = edge_weight
    data_file.close()
#------------------------------------------------------------------------------
def augment_cloud_batch(clouds, args):
    """" Augmentation on XYZ and jittering of everything """
    if args.pc_augm_rot:
        angle = np.random.uniform(0,2*math.pi, size=clouds.shape[0])
        M = np.array([transforms3d.axangles.axangle2mat([0,0,1],t) for t in angle])
        clouds[:,:,:3] = np.matmul(clouds[:,:,:3], M)

    if args.pc_augm_jitter:
        sigma, clip= 0.001, 0.003 # https://github.com/charlesq34/pointnet/blob/master/provider.py#L74
        #clouds = clouds + np.clip(sigma * np.random.standard_normal(clouds.shape), -1*clip, clip).astype(np.float32)
    return clouds
#------------------------------------------------------------------------------
def augment_cloud_whole(args, xyz, rgb):
    """" rotate the whole graph, add jitter """
    if args.pc_augm_rot:
        ref_point = xyz[np.random.randint(xyz.shape[0]),:3]
        ref_point[2] = 0
        M = transforms3d.axangles.axangle2mat([0,0,1],np.random.uniform(0,2*math.pi)).astype('f4')
        xyz = np.matmul(xyz[:,:3]-ref_point, M)+ref_point
    if args.pc_augm_jitter: #add jitter
        sigma, clip= 0.002, 0.005 # https://github.com/charlesq34/pointnet/blob/master/provider.py#L74
        xyz = xyz + np.clip(sigma * np.random.standard_normal(xyz.shape), -1*clip, clip).astype(np.float32)
        if args.use_rgb:
            rgb = np.clip(rgb + np.clip(sigma * np.random.standard_normal(xyz.shape), -1*clip, clip).astype(np.float32),-1,1)
    return xyz, rgb
#------------------------------------------------------------------------------
class spatialEmbedder():
    """ 
    Hand-crafted embeding of point cloud
    """
    def __init__(self, args):
        self.args = args

    def run_batch(self, model, clouds, *excess):
        """return clouds which should contain the embeddings"""
        if self.args.cuda:
            return (clouds.cuda())
        else:
            return (clouds)
if __name__ == "__main__": 
    main()
