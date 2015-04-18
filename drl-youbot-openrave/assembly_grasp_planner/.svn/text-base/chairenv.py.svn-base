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

all_robot_names = ['drc1','drc2','drc3','drc4','drc5']

all_object_names = ['rightside','back','fastener_rightside_to_back',
                    'seat','fastener_rightside_to_seat', 
                    'leftside','fastener_leftside_to_seat',
                    'fastener_leftside_to_back']

rightside_and_back = AssemblyOperation(assembly_list        = [rightside,back,fastener_rightside_to_back],
                                       assembly_poses       = [np.eye(4), 
                                                               np.array([[ 1.        ,  0.        ,  0.        , -0.09238497],
                                                                         [ 0.        ,  1.        ,  0.        ,  0.08709304],
                                                                         [ 0.        ,  0.        ,  1.        ,  0.15055516],
                                                                         [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                                                               np.array([[  1.    ,   0.     ,   0.     , -8.99258554e-02],
                                                                         [  0.    ,   1.     ,   0.     , -6.59056604e-02],
                                                                         [  0.    ,   0.     ,   1.     ,  2.84104079e-01],
                                                                         [  0.    ,   0.     ,   0.     ,  1.            ]])],
                                       assembly_robot_names = ['drc1','drc2','drc3'],
                                       name                 = 'rightside_and_back',
                                       robot_count = 1)

rightside_and_back_and_seat =  AssemblyOperation(assembly_list  = [rightside_and_back,seat,fastener_rightside_to_seat],
                                                 assembly_poses       = [np.eye(4),
                                                                         np.array([[ 0.     ,  0.     ,  1.    , -0.07255655],
                                                                                   [ 0.     ,  1.     ,  0.    ,  0.08884105],
                                                                                   [-1.     ,  0.     ,  0.    ,  0.14110599],
                                                                                   [ 0.     ,  0.     ,  0.    ,  1.        ]]),
                                                                         np.array([[  1.    ,   0.     ,   0.    ,  6.14305511e-02],
                                                                                   [  0.    ,   1.     ,   0.    , -6.71826154e-02],
                                                                                   [  0.    ,   0.     ,   1.    ,  1.41876787e-01],
                                                                                   [  0.    ,   0.     ,   0.    ,  1.            ]])],
                                                 assembly_robot_names = ['drc1','drc2','drc3'],
                                                 name                 = 'rightside_and_back_and_seat',
                                                 robot_count = 1)

chair = AssemblyOperation(assembly_list        = [rightside_and_back_and_seat,leftside,fastener_leftside_to_seat, fastener_leftside_to_back],
                                                             assembly_poses       = [np.eye(4),
                                                                                     np.array([[  1.    ,   0.    ,   0.    , -3.56287346e-05],
                                                                                               [  0.    ,   1.    ,   0.    ,  1.74892381e-01 ],
                                                                                               [  0.    ,   0.    ,   1.    ,  6.36778772e-04 ],
                                                                                               [  0.    ,   0.    ,   0.    ,  1.             ]]),
                                                                                     np.array([[  1.    ,   0.    ,   0.    , 6.65549785e-02],
                                                                                               [  0.    ,  -1.    ,   0.    , 2.40744516e-01],
                                                                                               [  0.    ,   0.    ,  -1.    , 1.42406836e-01],
                                                                                               [  0.    ,   0.    ,   0.    , 1.            ]]), 
                                                                                     np.array([[  1.    ,   0.    ,   0.    , -9.06435475e-02],
                                                                                                [  0.    ,  -1.    ,   0.    ,  2.38736570e-01],
                                                                                                [  0.    ,   0.    ,  -1.    ,  2.84831405e-01],
                                                                                                [  0.    ,   0.    ,   0.    ,  1.            ]])],
                                                             assembly_robot_names = ['drc1','drc2','drc3', 'drc4'],
                                                             name                 = 'chair',
                                                             robot_count = 1)

assembly_operations = [rightside_and_back,
                       rightside_and_back_and_seat,
                       chair]

robot_start_poses = {'drc1':np.array([[ 1.        ,  0.        ,  0.        ,  0.31730339],
                                      [ 0.        ,  1.        ,  0.        ,  0.49844295],
                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
                                      [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                     'drc2':np.array([[ 1.        ,  0.        ,  0.        , -1.2       ],
                                      [ 0.        ,  1.        ,  0.        ,  2.57      ],
                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
                                      [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                     'drc3':np.array([[ 1.        ,  0.        ,  0.        ,  0.55      ],
                                      [ 0.        ,  1.        ,  0.        ,  1.75      ],
                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
                                      [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                     'drc4':np.array([[  1.     ,   0.      ,   0.      , -3.35   ],
                                      [  0.     ,   1.      ,   0.      ,  0.55   ],
                                      [  0.     ,   0.      ,   1.      ,  0.     ],
                                      [  0.     ,   0.      ,   0.      ,  1.     ]]),
                     'drc5':np.array([[  1.     ,   0.      ,   0.      , -5.35   ],
                                      [  0.     ,   1.      ,   0.      ,  5.55   ],
                                      [  0.     ,   0.      ,   1.      ,  0.     ],
                                      [  0.     ,   0.      ,   0.      ,  1.     ]])
                    }


