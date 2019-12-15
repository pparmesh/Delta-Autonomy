#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
Author  : Heethesh Vhavle
Email   : heethesh@cmu.edu
Version : 1.0.0
Date    : Oct 31, 2019
'''

# Python 2/3 compatibility
from __future__ import print_function, absolute_import, division


def constrain(x, xmin, xmax):
    if x > xmax: return xmax
    elif x < xmin: return xmin
    else: return x


def mapfloat(x, in_min, in_max, out_min, out_max):
    return (x - in_min)  *  (out_max - out_min) / (in_max - in_min) + out_min
