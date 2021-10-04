#
#  Name        : seg_aug.cpp
#  Author      : Weixiao GAO
#  Date        : 15/09/2021
#  Version     : 1.0
#  Description : using SMOTE for training data augmentation
#  Availability:
#  Copyright   : Copyright (C) 2021 by Weixiao GAO (gaoweixiaocuhk@gmail.com)
#                All rights reserved.
#
#                 This file is part of semantic_urban_mesh_segmentation: software
#                 for semantic segmentation of textured urban meshes.
#
#                 semantic_urban_mesh_segmentation is free software; you can
#                 redistribute it and/or modify it under the terms of the GNU
#                 General Public License Version 3 as published by the Free
#                  Software Foundation.
#
#                 semantic_urban_mesh_segmentation is distributed in the hope that
#                 it will be useful, but WITHOUT ANY WARRANTY; without even the
#                 implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#                 PURPOSE. See the GNU General Public License for more details.
#
#                 You should have received a copy of the GNU General Public License
#                 along with this program. If not, see <http://www.gnu.org/licenses/>.

import sys
def segment_feature_augmentation(in_folder, out_folder, labels_name_size, used_k_neighbors):
    import numpy as np
    import os.path
    import glob
    from timeit import default_timer as timer
    from imblearn.over_sampling import SVMSMOTE
    import libpp

    print("================= SMOTE augmentation =================")
    files = glob.glob(in_folder + '*.ply') #
    for file in files:
        print("Processing " + file + ", k_neighbors = " + str(used_k_neighbors))
        file_name = os.path.splitext(os.path.basename(file))[0]
        in_file = in_folder + file_name + '.ply'

        feas, labels = libpp.read_data_for_augmentation(in_file, labels_name_size)
        start = timer()
        if max(labels) - min(labels) > 0:
            used_k_neighbors = int(used_k_neighbors)
            feas_aug, labels_aug = SVMSMOTE(k_neighbors=used_k_neighbors, n_jobs=-1).fit_resample(np.asarray(feas), np.asarray(labels))
        else:
            print("     The input file only contain one label, cannot perform data augmentation")
            feas_aug, labels_aug = feas, labels

        end = timer()
        print("     Augmentation cost %5.1f (s)" % (end - start))

        out_file = out_folder + file_name + '_aug.ply'
        libpp.write_data_for_augmentation(in_file, out_file, feas_aug, labels_aug, labels_name_size)

in_folder = sys.argv[0]
out_folder = sys.argv[1]
labels_name_size = sys.argv[2]
used_k_neighbors = sys.argv[3]
segment_feature_augmentation(in_folder, out_folder, labels_name_size, used_k_neighbors)