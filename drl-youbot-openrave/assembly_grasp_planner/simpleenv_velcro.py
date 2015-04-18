#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from GraspRegions import GraspRegions
from AssemblyOperation import Part,AssemblyOperation

envfile = 'environments/simple.env.xml'

rightside                  = Part('rightside',is_transferable=True)
back                       = Part('back',is_transferable=True)
cseat                      = Part('cseat',is_transferable=True)
leftside                   = Part('leftside',is_transferable=True)
all_parts = [rightside,back,cseat,leftside]

all_robot_names = ['drc1','drc2','drc3']

all_object_names = ['rightside','back','cseat','leftside']

rightside_and_back = AssemblyOperation(assembly_list        = [rightside,back,cseat],
                                       assembly_poses       = [np.eye(4), 
                                                               np.array([[  1.0           ,   0.0           ,   0.0           ,-0.095],
                                                                         [  0.0           ,   0.0           ,   1.0           , 0.02],
                                                                         [  0.0           ,  -1.0           ,   0.0           , 0.27],
                                                                         [  0.0           ,   0.0           ,   0.0           , 1.0]]),
                                                               np.array([[ 0.     ,  0.     ,  1.    , -0.07255655],
                                                                         [ 0.     ,  1.     ,  0.    ,  0.08884105],
                                                                         [-1.     ,  0.     ,  0.    ,  0.14110599],
                                                                         [ 0.     ,  0.     ,  0.    ,  1.        ]])],
                                       assembly_robot_names = ['drc1','drc3','drc2'],
                                       name                 = 'rightside_and_back',
                                       order = ['cseat','back','rightside'],
                                       robot_count = 1)

chair = AssemblyOperation(assembly_list        = [rightside_and_back,leftside],
                          assembly_poses       = [np.eye(4),
                                                  np.array([[  1.    ,   0.    ,   0.    ,  0.],
                                                            [  0.    ,   1.    ,   0.    ,  0.21 ],
                                                            [  0.    ,   0.    ,   1.    ,  0. ],
                                                            [  0.    ,   0.    ,   0.    ,  1.             ]])],
                          assembly_robot_names = ['drc3','drc1'],
                          name                 = 'chair',
                          order = ['rightside_and_back','leftside'],
                          robot_count = 1)


assembly_operations = [rightside,                   # part grasp
                       back,                        # part grasp
                       cseat,                        # part grasp
                       rightside_and_back,          # complex op
                       leftside,
                       chair]

robot_start_poses = {
'drc1':np.array([[ 1. , 0. , 0. , 1.50],
                 [ 0. , 1. , 0. , 0.35420695],
                 [ 0. , 0. , 1. , 0.        ],
                 [ 0. , 0. , 0. , 1.        ]]),
'drc2':np.array([[ 1. , 0. , 0. ,-2.30931759e+00],
                 [ 0. , 1. , 0. , 1.69879246e+00],
                 [ 0. , 0. , 1. ,-5.57303429e-06],
                 [ 0. , 0. , 0. , 1.00000000e+00]]),
'drc3':np.array([[ 1. , 0. , 0. , 1.50],
                 [ 0. , 1. , 0. , 2.68708372e+00],
                 [ 0. , 0. , 1. ,-7.45058060e-06],
                 [ 0. , 0. , 0. , 1.00000000e+00]]),
'drc4':np.array([[ 1. , 0. , 0. ,-2.12870502e+00],
                 [ 0. , 1. , 0. ,-1.57583475e-01],
                 [ 0. , 0. , 1. ,-1.78813934e-07],
                 [ 0. , 0. , 0. , 1.00000000e+00]])}


