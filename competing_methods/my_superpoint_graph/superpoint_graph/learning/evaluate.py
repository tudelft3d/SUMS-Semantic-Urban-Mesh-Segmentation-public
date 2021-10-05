#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul  4 09:22:55 2019

@author: landrieuloic
"""

"""
    Large-scale Point Cloud Semantic Segmentation with Superpoint Graphs
    http://arxiv.org/abs/1711.09869
    2017 Loic Landrieu, Martin Simonovsky
"""
import argparse
import numpy as np
import sys

sys.path.append("./learning")
from metrics import *

parser = argparse.ArgumentParser(description='Evaluation function for S3DIS')

parser.add_argument('--odir', default='../datasets/custom_set/results', help='Directory to store results')
parser.add_argument('--dataset', default='../datasets/custom_set', help='Directory to store results')
parser.add_argument('--cvfold', default='', help='which fold to consider')

args = parser.parse_args()


n_labels = 6
inv_class_map = {0:'ground', 1:'vegetation', 2:'building', 3:'water', 4:'car', 5:'boat'}
base_name = args.odir

C = ConfusionMatrix(n_labels)
C.confusion_matrix = np.zeros((n_labels, n_labels))

cm = ConfusionMatrix(n_labels)
cm.confusion_matrix = np.load(base_name + '/pointwise_cm.npy')
print("\t OA = %3.2f \t mA = %3.2f \t mIoU = %3.2f" % (100 * ConfusionMatrix.get_overall_accuracy(cm), 100 * ConfusionMatrix.get_mean_class_accuracy(cm),100 * ConfusionMatrix.get_average_intersection_union(cm)))
C.confusion_matrix += cm.confusion_matrix

print("\nOverall accuracy : %3.2f %%" % (100 * (ConfusionMatrix.get_overall_accuracy(C))))
print("Mean accuracy    : %3.2f %%" % (100 * (ConfusionMatrix.get_mean_class_accuracy(C))))
print("Mean IoU         : %3.2f %%\n" % (100 * (ConfusionMatrix.get_average_intersection_union(C))))
print("         Classe :   IoU")
for c in range(0, n_labels):
    print("   %12s : %6.2f %% \t %.1e points" % (
    inv_class_map[c], 100 * ConfusionMatrix.get_intersection_union_per_class(C)[c], ConfusionMatrix.count_gt(C, c)))
