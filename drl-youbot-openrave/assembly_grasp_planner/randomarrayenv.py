#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from GraspRegions import GraspRegions
from AssemblyOperation import AssemblyOperation, Part
import IPython

envfile = 'environments/random_array.env.xml'

num_parts = 3
total_num = num_parts*num_parts*2

vertical_parts = {}
horizontal_parts = {}
all_names = []


all_robot_names = ['drc1','drc2','drc3', 'drc4', 'drc5']
for i in range(total_num):
    vert_name = 'link_vertical' + str(i)
    horizontal_name = 'link_horizontal' + str(i)
    vertical_parts[horizontal_name] = Part(vert_name,is_transferable=True)
    horizontal_parts[vert_name] = Part(horizontal_name,is_transferable=True)
    
    
    
all_parts = vertical_parts.values() + horizontal_parts.values()
all_object_names = vertical_parts.keys() + horizontal_parts.keys()

assembly_operations = []
for part in all_parts:
    assembly_operations.append(part)    


side_length = 0.2

parity = 1
for i in range(len(all_parts) / 4):
    #i is the plus sign number
    idx = i*2
    vert_part_name1 = all_parts[idx]
    vert_part_name2 = all_parts[idx + 1]
    hor_part_name1 = all_parts[idx + total_num ]
    hor_part_name2 = all_parts[idx + total_num + 1]
    
    if i == 0:
        assembly_list = [vert_part_name1, vert_part_name2, hor_part_name1, hor_part_name2]
        assembly_poses = [np.array([[  1.0,  0, 0, 0], [0, 1.0, 0, side_length], [0, 0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]), 
                          np.array([[  1.0,  0, 0, 0], [0, 1.0, 0, -side_length], [0, 0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]),
                          np.array([[  1.0,  0, 0, side_length], [0, 1.0, 0, 0.0], [0, 0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]),
                          np.array([[  1.0,  0, 0, -side_length], [0, 1.0, 0, 0.0], [0, 0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]),
                          ]
        assembly_robot_names = all_robot_names[1:4]
    else:
        assembly_list = [new_op, vert_part_name1, vert_part_name2, hor_part_name1, hor_part_name2]
        if i % num_parts == 0: #stack it vertically
            parity *= -1      
            prev_pose = np.array([[  1.0,  0, 0, 0], [0, 1.0, 0, -side_length * 4 ], [0, 0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        else: #stack it horizontally

            prev_pose = np.array([[  1.0,  0, 0, -side_length * 4 * parity], [0, 1.0, 0, 0.0], [0, 0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
        assembly_poses = [prev_pose, 
                          np.array([[  1.0,  0, 0, 0], [0, 1.0, 0, side_length], [0, 0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]), 
                          np.array([[  1.0,  0, 0, 0], [0, 1.0, 0, -side_length], [0, 0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]),
                          np.array([[  1.0,  0, 0, side_length], [0, 1.0, 0, 0.0], [0, 0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]),
                          np.array([[  1.0,  0, 0, -side_length], [0, 1.0, 0, 0.0], [0, 0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]),
                          ]
        assembly_robot_names = all_robot_names[1:5]
    name = 'plus_sign' + str(i)
    robot_count = 1
    new_op = AssemblyOperation( assembly_list = assembly_list, 
                                                    assembly_poses = assembly_poses, 
                                                assembly_robot_names = assembly_robot_names, 
                                                name = name, 
                                                robot_count = 1)
    assembly_operations.append( new_op )
    
    



robot_start_poses = {'drc1':np.array([[ 1.        ,  0.        ,  0.        ,  -100.0],
                                      [ 0.        ,  1.        ,  0.        ,  -100.0],
                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
                                      [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                     'drc2':np.array([[ 1.        ,  0.        ,  0.        , -100.0 ],
                                      [ 0.        ,  1.        ,  0.        ,  100.0      ],
                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
                                      [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                     'drc3':np.array([[ 1.        ,  0.        ,  0.        ,  100.0      ],
                                      [ 0.        ,  1.        ,  0.        ,  100.0      ],
                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
                                      [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                     'drc4':np.array([[ 1.        ,  0.        ,  0.        ,  100.0      ],
                                      [ 0.        ,  1.        ,  0.        ,  -100.0      ],
                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
                                      [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                     'drc5':np.array([[ 1.        ,  0.        ,  0.        ,  100.0      ],
                                      [ 0.        ,  1.        ,  0.        ,  50.0      ],
                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
                                      [ 0.        ,  0.        ,  0.        ,  1.        ]])
                    }
