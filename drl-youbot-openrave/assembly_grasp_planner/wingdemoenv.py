#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from tsr import GraspTSR,LookAtTSR
from Assembly import Assembly

envfile = 'environments/wingdemo.env.xml'
assembly = Assembly(object_names=['wingskin','fastener','camera'],
                    object_relative_poses={                       
                        'wingskin':  np.array([[ 1.    ,  0.    ,  0.    , -0.35  ],
                                               [ 0.    ,  1.    ,  0.    ,  0.    ],
                                               [ 0.    ,  0.    ,  1.    ,  0.073 ],
                                               [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                        'camera':np.array([[ 1.    ,  0.    ,  0.    , -0.027  ],
                                           [ 0.    ,  0.    , -1.    ,  0.163  ],
                                           [ 0.    ,  1.    ,  0.    ,  0.17  ],
                                           [ 0.    ,  0.    ,  0.    ,  1.     ]]),
                        'fastener': np.array([[ 1.    ,  0.    ,  0.    , -0.022  ],
                                              [ 0.    ,  0.    , -1.    ,  0.168  ],
                                              [ 0.    ,  1.    ,  0.    ,  0.10  ],
                                              [ 0.    ,  0.    ,  0.    ,  1.     ]])
                        },
                    fixed_objects = ['ladder']
                    )

assembly_pose = np.array([[ 0.7009, -0.7133, -0.    , -1.2478],
                          [ 0.7133,  0.7009, -0.    ,  0.3399],
                          [ 0.    , -0.    ,  1.    ,  0.13  ],
                          [ 0.    ,  0.    ,  0.    ,  1.    ]])

task_regions = {'wingskin': GraspTSR(T0_w_obj='wingskin',
                                     Tw_e_list=[
                                               np.array([[ 0.    ,  0.    ,  1.    , -0.35  ],
                                                         [ 1     ,  0.    ,  0.    ,  0.    ],
                                                         [ 0.    ,  1.    ,  0.    ,  0.01  ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 1.    ,  0.    ,  0.    ,  0.    ],
                                                         [ 0.    ,  0.    , -1.    ,  0.17  ],
                                                         [ 0.    ,  1.    ,  0.    ,  0.01  ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]])
                                           ],
                                     Bw_list= [[-0.14, 0.14,    0, 0,    0, 0,    0, 0,    0, 0,    0, 0],
                                               [-0.32, 0.32,    0, 0,    0, 0,    0, 0,    0, 0,    0, 0]]
                                    ),
                'fastener': GraspTSR(T0_w_obj='fastener',
                                     Tw_e_list=[
                                               np.array([[ 0.    ,  1.    ,  0.    ,  0.    ],
                                                         [-1.    ,  0.    ,  0.    ,  0.025 ],
                                                         [ 0.    ,  0.    ,  1.    ,  0.    ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]])
                                           ],
                                     Bw_list= [[-0.025, 0.025,    0, 0,    0, 0,   -np.pi, np.pi,  0, 0,   0, 0] ]
                                    ),
                'camera': GraspTSR(T0_w_obj='camera',
                                   Tw_e_list=[
                                               np.array([[ 0.    ,  1.    ,  0.    ,  0.    ],
                                                         [-1.    ,  0.    ,  0.    ,  0.025 ],
                                                         [ 0.    ,  0.    ,  1.    ,  0.    ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]])
                                           ],
                                   Bw_list= [[-0.025, 0.025,    0, 0,    0, 0,   -np.pi, np.pi,   0, 0,   0, 0]]
                                   )
        }

robots = ['drc1','drc2','drc3']
part_responsibilities = {'wingskin':'drc1', 
                         'fastener':'drc2', 
                         'camera':'drc3'}

# Initial robot poses. Not important, may change between runs.
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
                                      [ 0.        ,  0.        ,  0.        ,  1.        ]])
                    }



