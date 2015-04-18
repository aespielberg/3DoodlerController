#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from GraspRegions import GraspRegions
from AssemblyOperation import Part,AssemblyOperation

envfile = 'environments/chair.env.xml'

rightside                  = Part('rightside',is_transferable=True)
leftside                   = Part('leftside',is_transferable=True)
back                       = Part('back',is_transferable=True)
seat                       = Part('seat',is_transferable=True)
fastener_rightside_to_back = Part('fastener_rightside_to_back',is_transferable=False)
fastener_leftside_to_back  = Part('fastener_leftside_to_back',is_transferable=False)
fastener_rightside_to_seat = Part('fastener_rightside_to_seat',is_transferable=False)
fastener_leftside_to_seat  = Part('fastener_leftside_to_seat',is_transferable=False)
all_parts = [rightside,back,fastener_rightside_to_back,
             seat,fastener_rightside_to_seat, 
             leftside,fastener_leftside_to_seat,
             fastener_leftside_to_back]

#all_robot_names = ['drc1','drc2','drc3','drc4','drc5']
all_robot_names = ['drc1','drc2','drc3','drc4']

all_object_names = ['rightside','back','fastener_rightside_to_back',
                    'seat','fastener_rightside_to_seat', 
                    'leftside','fastener_leftside_to_seat',
                    'fastener_leftside_to_back']

rightside_and_back = AssemblyOperation(assembly_list        = [rightside,back],
                                       assembly_poses       = [np.eye(4), 
                                                               np.array([[ 1.        ,  0.        ,  0.        , -0.09238497],
                                                                         [ 0.        ,  1.        ,  0.        ,  0.105],
                                                                         [ 0.        ,  0.        ,  1.        ,  0.18],
                                                                         [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                                                               ],
                                       assembly_robot_names = ['drc1','drc3'],
                                       name                 = 'rightside_and_back',
                                       robot_count = 1)

rightside_and_back_and_seat =  AssemblyOperation(assembly_list  = [rightside_and_back,seat],
                                                 assembly_poses       = [np.eye(4),
                                                                         np.array([[ 0.     ,  0.     ,  1.    , -0.07255655],
                                                                                   [ 0.     ,  1.     ,  0.    ,  0.105],
                                                                                   [-1.     ,  0.     ,  0.    ,  0.17],
                                                                                   [ 0.     ,  0.     ,  0.    ,  1.        ]]),
                                                                         ],
                                                 assembly_robot_names = ['drc1','drc3'],
                                                 name                 = 'rightside_and_back_and_seat',
                                                 robot_count = 1)

chair = AssemblyOperation(assembly_list        = [rightside_and_back_and_seat,leftside],
                                                             assembly_poses       = [np.eye(4),
                                                                                     np.array([[  1.    ,   0.    ,   0.    ,  0.],
                                                                                               [  0.    ,   1.    ,   0.    ,  0.21 ],
                                                                                               [  0.    ,   0.    ,   1.    ,  0. ],
                                                                                               [  0.    ,   0.    ,   0.    ,  1.             ]]),
                                                                                     np.array([[  1.    ,   0.    ,   0.    , 6.65549785e-02],
                                                                                               [  0.    ,  -1.    ,   0.    , 0.29],
                                                                                               [  0.    ,   0.    ,  -1.    , 1.42406836e-01],
                                                                                               [  0.    ,   0.    ,   0.    , 1.            ]]), 
                                                                                     ],
                                                             assembly_robot_names = ['drc1','drc3'],
                                                             name                 = 'chair',
                                                             robot_count = 1)


assembly_operations = [rightside,                   # part grasp
                       back,                        # part grasp
                       fastener_rightside_to_back,  # part grasp
                       rightside_and_back,          # complex op
                       seat,                        # part grasp
                       fastener_rightside_to_seat,  # part grasp
                       rightside_and_back_and_seat, # complex op
                       leftside,                    # part grasp
                       fastener_leftside_to_back,   # part grasp
                       fastener_leftside_to_seat,   # part grasp
                       chair]                       # complex op

robot_start_poses = {
'drc1':np.array([[ 1. , 0. , 0. , 1.0],
                 [ 0. , 1. , 0. , 0.35420695],
                 [ 0. , 0. , 1. , 0.        ],
                 [ 0. , 0. , 0. , 1.        ]]),
'drc2':np.array([[ 1. , 0. , 0. ,-1.30931759e+00],
                 [ 0. , 1. , 0. , 1.69879246e+00],
                 [ 0. , 0. , 1. ,-5.57303429e-06],
                 [ 0. , 0. , 0. , 1.00000000e+00]]),
'drc3':np.array([[ 1. , 0. , 0. , 1.0],
                 [ 0. , 1. , 0. , 2.68708372e+00],
                 [ 0. , 0. , 1. ,-7.45058060e-06],
                 [ 0. , 0. , 0. , 1.00000000e+00]]),
'drc4':np.array([[ 1. , 0. , 0. ,-1.12870502e+00],
                 [ 0. , 1. , 0. ,-1.57583475e-01],
                 [ 0. , 0. , 1. ,-1.78813934e-07],
                 [ 0. , 0. , 0. , 1.00000000e+00]])}
#'drc5':np.array([[ 1. , 0. , 0. ,-2.31274414e+00],
#                 [ 0. , 1. , 0. , 3.05042672e+00],
#                 [ 0. , 0. , 1. , 3.18884850e-06],
#                 [ 0. , 0. , 0. , 1.00000000e+00]])}


#robot_start_poses = {'drc1':np.array([[ 1.        ,  0.        ,  0.        ,  1.31730339],
#                                      [ 0.        ,  1.        ,  0.        ,  0.49844295],
#                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
#                                      [ 0.        ,  0.        ,  0.        ,  1.        ]]),
#                     'drc2':np.array([[ 1.        ,  0.        ,  0.        , -1.2       ],
#                                      [ 0.        ,  1.        ,  0.        ,  2.57      ],
#                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
#                                      [ 0.        ,  0.        ,  0.        ,  1.        ]]),
#                     'drc3':np.array([[ 1.        ,  0.        ,  0.        ,  0.55      ],
#                                      [ 0.        ,  1.        ,  0.        ,  1.75      ],
#                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
#                                      [ 0.        ,  0.        ,  0.        ,  1.        ]]),
#                     'drc4':np.array([[  1.     ,   0.      ,   0.      , -3.35   ],
#                                      [  0.     ,   1.      ,   0.      ,  0.55   ],
#                                      [  0.     ,   0.      ,   1.      ,  0.     ],
#                                      [  0.     ,   0.      ,   0.      ,  1.     ]]),
#                     'drc5':np.array([[  1.     ,   0.      ,   0.      , -5.35   ],
#                                      [  0.     ,   1.      ,   0.      ,  5.55   ],
#                                      [  0.     ,   0.      ,   1.      ,  0.     ],
#                                      [  0.     ,   0.      ,   0.      ,  1.     ]])
#                    }


