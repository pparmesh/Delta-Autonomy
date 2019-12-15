# -*- coding: utf-8 -*-
# @Author: Heethesh Vhavle
# @Date:   Nov 20, 2019
# @Last Modified by:   Heethesh Vhavle
# @Last Modified time: Nov 20, 2019

import numpy as np
import matplotlib.pyplot as plt
from pyclustering.cluster.dbscan import dbscan
from pyclustering.cluster import cluster_visualizer

from features import features

cmap = plt.get_cmap('tab10')

# [0, 1, 2,  3,  4,  5,  6]
# [x, y, r, vx, vy, vr, th]
data = np.c_[features[:, 2], features[:, 6], features[:, 5]] #, features[:, 6]

# Create DBSCAN algorithm.
dbscan_instance = dbscan(data, 0.7, 3)

# Start processing by DBSCAN.
dbscan_instance.process()

# Obtain results of clustering.
clusters = dbscan_instance.get_clusters()
noise = dbscan_instance.get_noise()

labels = np.full_like(features[:, 0], -1).astype('int')
for i, indices in enumerate(clusters): labels[indices] = i
# labels += 1
print(labels)
print(len(clusters))

# cmap = plt.get_cmap('tab10')
# for i, (x, y, r, vx, vy, vr, th) in enumerate(features):
#     plt.scatter(x, y, color=cmap(labels[i]))
# plt.grid()
# plt.show()

targets = np.array([np.mean(features[cluster], axis=0) for cluster in clusters])
print(targets.shape)

max_vel = np.max(features[:, 5])
features[:, 5] = features[:, 5] / max_vel

for (x, y, r, vx, vy, vr, th), label in zip(features, labels):
	if label != -1:
	    plt.scatter(x, y, color=(0.2, 0.2, vr), s=100)
	    plt.scatter(x, y, color=cmap(label), s=20)
plt.scatter(targets[:, 0], targets[:, 1], c='r')
plt.grid()
plt.show()

# Visualize clustering results
# visualizer = cluster_visualizer()
# visualizer.append_clusters(clusters, features[:, :2])
# visualizer.append_cluster(noise, features[:, :2], marker='x')
# visualizer.show()
