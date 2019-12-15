# -*- coding: utf-8 -*-
# @Author: Heethesh Vhavle
# @Date:   Nov 18, 2019
# @Last Modified by:   Heethesh Vhavle
# @Last Modified time: Nov 18, 2019

import numpy as np
import matplotlib.pyplot as plt

rows1 = np.array([29.7, 42.7, 55.6, 68.45, 81.2, 94.4, 107.3, 120.3]) / 100
rows2 = np.array([78.2, 91.1, 104.0, 117.0, 130.0, 142.8]) / 100
rows3 = np.array([161.8, 174.7, 187.7, 200.6, 213.4, 226.4]) / 100

gs = np.mean(np.hstack((np.ediff1d(rows1), np.ediff1d(rows2), np.ediff1d(rows3))))

cols1 = [-gs * 3, -gs * 2, -gs * 1, 0, gs * 1, gs * 2]
cols2 = [-gs * 4, -gs * 3, -gs * 2, -gs * 1, 0, gs * 1, gs * 2, gs * 3]
cols3 = [-gs * 4, -gs * 3, -gs * 2, -gs * 1, 0, gs * 1, gs * 2, gs * 3]

matrix1 = np.array([[(i, j) for i in cols1] for j in rows1])
matrix2 = np.array([[(i, j) for i in cols2] for j in rows2])
matrix3 = np.array([[(i, j) for i in cols3] for j in rows3])

idx1 = np.array([
    np.array([2, 3, 4]),
    np.array([2, 3, 4]),
    np.array([1, 2, 3, 4, 5]),
    np.array([0, 1, 2, 3, 4, 5]),
    np.array([0, 1, 2, 3, 4, 5]),
    np.array([0, 1, 2, 3, 4, 5]),
    np.array([0, 1, 2, 3, 4, 5]),
    np.array([0, 1, 2, 3, 4, 5]),
])

idx2 = np.array([
    np.array([1, 2, 3, 4, 5, 6, 7]),
    np.array([0, 1, 2, 3, 4, 5, 6, 7]),
    np.array([0, 1, 2, 3, 4, 5, 6, 7]),
    np.array([0, 1, 2, 3, 4, 5, 6, 7]),
    np.array([0, 1, 2, 3, 4, 5, 6, 7]),
    np.array([0, 1, 2, 3, 4, 5, 6, 7]),
])

idx3 = np.array([
    np.array([1, 2, 3, 6]),
    np.array([0, 1, 2, 3, 4, 5, 6, 7]),
    np.array([0, 1, 5, 6]),
    np.array([7]),
    np.array([0, 3]),
    np.array([2, 5]),
])

world_points = np.array([
    np.vstack([row[idx] for row, idx in zip(matrix1, idx1)]),
    np.vstack([row[idx] for row, idx in zip(matrix2, idx2)]),
    np.vstack([row[idx] for row, idx in zip(matrix3, idx3)])
])
