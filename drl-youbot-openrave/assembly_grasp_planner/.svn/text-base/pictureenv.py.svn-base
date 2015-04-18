#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from GraspRegions import GraspRegions
from AssemblyOperation import AssemblyOperation, Part

envfile = 'environments/pictureframe.env.xml'
fastener1                  = Part('fastener1',is_transferable=False)
fastener2                  = Part('fastener2',is_transferable=False)
fastener3                  = Part('fastener3',is_transferable=False)
fastener4                  = Part('fastener4',is_transferable=False)
fastener5                  = Part('fastener5',is_transferable=False)
fastener6                  = Part('fastener6',is_transferable=False)
fastener7                  = Part('fastener7',is_transferable=False)
fastener8                  = Part('fastener8',is_transferable=False)
fastener9                  = Part('fastener9',is_transferable=False)
fastener10                  = Part('fastener10',is_transferable=False)
fastener11                  = Part('fastener11',is_transferable=False)
fastener12                  = Part('fastener12',is_transferable=False)
fastener13                  = Part('fastener13',is_transferable=False)
fastener14                  = Part('fastener14',is_transferable=False)
sidelink1                  = Part('sidelink1',is_transferable=True)
sidelink2                  = Part('sidelink2',is_transferable=True)
sidelink3                  = Part('sidelink3',is_transferable=True)
sidelink4                  = Part('sidelink4',is_transferable=True)
sidelink5                  = Part('sidelink5',is_transferable=True)
sidelink6                  = Part('sidelink6',is_transferable=True)
sidelink7                  = Part('sidelink7',is_transferable=True)
sidelink8                  = Part('sidelink8',is_transferable=True)
sidelink9                  = Part('sidelink9',is_transferable=True)
sidelink10                  = Part('sidelink10',is_transferable=True)
cornerlink1                 = Part('cornerlink1',is_transferable=True)
cornerlink2                 = Part('cornerlink2',is_transferable=True)
cornerlink3                 = Part('cornerlink3',is_transferable=True)
cornerlink4                 = Part('cornerlink4',is_transferable=True)

dz = 0.2
dx = 0.5
dy = 0.5





                              




all_parts = [sidelink1, sidelink2, sidelink3, sidelink4, 
                    sidelink5, sidelink6, sidelink7, sidelink8, 
                    sidelink9, sidelink10, cornerlink1, cornerlink2, 
                    cornerlink3, cornerlink4, fastener1, fastener2, 
                    fastener3, fastener4, fastener5, fastener6, 
                    fastener7, fastener8, fastener9, fastener10, 
                    fastener11, fastener12, fastener13, fastener14]

all_robot_names = ['drc1','drc2','drc3', 'drc4','drc5']

all_object_names = ['sidelink1', 'sidelink2', 'sidelink3', 'sidelink4', 
                    'sidelink5', 'sidelink6', 'sidelink7', 'sidelink8', 
                    'sidelink9', 'sidelink10', 'cornerlink1', 'cornerlink2', 
                    'cornerlink3', 'cornerlink4', 'fastener1', 'fastener2', 
                    'fastener3', 'fastener4', 'fastener5', 'fastener6', 
                    'fastener7', 'fastener8', 'fastener9', 'fastener10', 
                    'fastener11', 'fastener12', 'fastener13', 'fastener14']




sidelink1_2 = AssemblyOperation(assembly_list        = [sidelink1, sidelink2, fastener1],
                                         assembly_poses       = [np.eye(4), 
                                                                 np.array([[  1.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                                  2.82281876e-01 * dx],
                                                               [  0.00000000e+00,   1.00000000e+00,   0.00000000e+00,
                                                                 -2.02743756e-03 * dy],
                                                               [  0.00000000e+00,   0.00000000e+00,   1.00000000e+00,
                                                                  1.53854489e-06 * dz],
                                                               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                                  1.00000000e+00]]),
                                                                  np.array([[  1.00000000e+00,   1.34401086e-07,  -1.18237335e-07,
                                                                              0.1875 * dx],
                                                                           [  1.34401115e-07,  -1.27438147e-01,   9.91846520e-01,
                                                                              0.0 * dy],
                                                                           [  1.18237303e-07,  -9.91846520e-01,  -1.27438147e-01,
                                                                              0.35 * dz],
        
                                                                           [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                                              1.00000000e+00]])
                                                                 ],
                                         assembly_robot_names = ['drc1','drc2','drc3'], #TODO: add back robot 3 soon
                                         name                 = 'sidelink1_2',
                                         robot_count = 1)
                                         
                                         
sidelink1_3 =                      AssemblyOperation(assembly_list        = [sidelink1_2, sidelink3, fastener2],
                                         assembly_poses       = [np.eye(4), 
                                                                 np.array([[  1.00000000e+00,   4.37671797e-11,   3.23001750e-09,
                                                                  5.89161277e-01 * dx],
                                                               [  4.37675267e-11,   9.99632852e-01,  -2.70954065e-02,
                                                                 -2.70942040e-03 * dy],
                                                               [ -3.23001750e-09,   2.70954065e-02,   9.99632852e-01,
                                                                  9.84980632e-03 * dz],
                                                               [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                                  1.00000000e+00]]),
                                                                  np.array([[  1.00000000e+00,   1.34401086e-07,  -1.18237335e-07,
                                                                              0.4875 * dx],
                                                                           [  1.34401115e-07,  -1.27438147e-01,   9.91846520e-01,
                                                                              0.0 * dy],
                                                                           [  1.18237303e-07,  -9.91846520e-01,  -1.27438147e-01,
                                                                              0.35 * dz],
                                                                           [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                                              1.00000000e+00]])],
                                         assembly_robot_names = ['drc1','drc2','drc3'], #TODO: add back robot 3 soon
                                         name                 = 'sidelink1_3',
                                         robot_count = 1)
                                         
sidelink1_C1 =                      AssemblyOperation(assembly_list        = [sidelink1_3, cornerlink1, fastener3],
                                         assembly_poses       = [np.eye(4), 
                                                                 np.array([[ 1.        ,  0.        ,  0.        ,  0.81738627 * dx],
                                                   [ 0.        ,  1.        ,  0.        , -0.00933409 * dy],
                                                   [ 0.        ,  0.        ,  1.        ,  0.01240559 * dz],
                                                   [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                                           np.array([[  1.00000000e+00,   1.34401086e-07,  -1.18237335e-07,
                                                      0.7875 * dx],
                                                   [  1.34401115e-07,  -1.27438147e-01,   9.91846520e-01,
                                                      0.0 * dy],
                                                   [  1.18237303e-07,  -9.91846520e-01,  -1.27438147e-01,
                                                      0.35 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),],
                                         assembly_robot_names = ['drc1','drc2','drc3'], #TODO: add back robot 3 soon
                                         name                 = 'sidelink1_C1',
                                         robot_count = 1)
                                         
sidelink1_4 =                      AssemblyOperation(assembly_list        = [sidelink1_C1, sidelink4, fastener4],
                                         assembly_poses       = [np.eye(4), 
                                                                 np.array([[ -2.96602786e-02,  -9.99560037e-01,  -1.19156870e-07,
                                                      9.41893160e-01 * dx],
                                                   [  9.99560037e-01,  -2.96602786e-02,  -1.22745102e-07,
                                                      2.70765513e-01 * dy],
                                                   [  1.19156873e-07,  -1.22745100e-07,   1.00000000e+00,
                                                      1.00196265e-02 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),
                                                      np.array([[  1.00000000e+00,   1.34401086e-07,  -1.18237335e-07,
                                                      0.9375],
                                                   [  1.34401115e-07,  -1.27438147e-01,   9.91846520e-01,
                                                      0.1625],
                                                   [  1.18237303e-07,  -9.91846520e-01,  -1.27438147e-01,
                                                      0.35 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),],
                                         assembly_robot_names = ['drc1','drc2','drc3'], #TODO: add back robot 3 soon
                                         name                 = 'sidelink1_4',
                                         robot_count = 2)
                                         
sidelink1_5 =                       AssemblyOperation(assembly_list        = [sidelink1_4, sidelink5, fastener5],
                                         assembly_poses       = [np.eye(4), 
                                                                 np.array([[ -2.00951091e-02,  -9.99798073e-01,  -1.19185203e-07,
                                                      9.25983250e-01 * dx],
                                                   [  9.99798073e-01,  -2.00951091e-02,  -1.21604788e-07,
                                                      5.71205020e-01 * dy],
                                                   [  1.19185193e-07,  -1.21604798e-07,   1.00000000e+00,
                                                      4.29918418e-06 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),
                                                      np.array([[  1.00000000e+00,   1.34401086e-07,  -1.18237335e-07,
                                                      0.9375 * dx],
                                                   [  1.34401115e-07,  -1.27438147e-01,   9.91846520e-01,
                                                      0.4625 * dy],
                                                   [  1.18237303e-07,  -9.91846520e-01,  -1.27438147e-01,
                                                      0.35 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),],
                                         assembly_robot_names = ['drc1','drc2','drc3'], #TODO: add back robot 3 soon
                                         name                 = 'sidelink1_5',
                                         robot_count = 2)
sidelink1_C2 =                       AssemblyOperation(assembly_list        = [sidelink1_5, cornerlink2, fastener6],
                                         assembly_poses       = [np.eye(4), 
                                                                 np.array([[ -2.99547025e-02,  -9.99551257e-01,  -1.19155763e-07,
                                                      9.28616583e-01 * dx],
                                                   [  9.99551257e-01,  -2.99547025e-02,  -1.22780179e-07,
                                                      8.01629782e-01 * dy],
                                                   [  1.19155807e-07,  -1.22780137e-07,   1.00000000e+00,
                                                     -1.39969541e-03 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),
                                                      np.array([[  1.00000000e+00,   1.34401086e-07,  -1.18237335e-07,
                                                      0.9375 * dx],
                                                   [  1.34401115e-07,  -1.27438147e-01,   9.91846520e-01,
                                                      0.7625 * dy],
                                                   [  1.18237303e-07,  -9.91846520e-01,  -1.27438147e-01,
                                                      0.35 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),],
                                         assembly_robot_names = ['drc1','drc2','drc3'], #TODO: add back robot 3 soon
                                         name                 = 'sidelink1_C2',
                                         robot_count = 2)
                                         
sidelink1_6 =                      AssemblyOperation(assembly_list        = [sidelink1_C2, sidelink6, fastener7],
                                         assembly_poses       = [np.eye(4), 
                                                                 np.array([[ -9.98248424e-01,   5.91615139e-02,   7.05260185e-09,
                                                      6.65896118e-01 * dx],
                                                   [ -5.91615139e-02,  -9.98248424e-01,  -2.38209761e-07,
                                                      9.18332040e-01 * dy],
                                                   [ -7.05260140e-09,  -2.38209761e-07,   1.00000000e+00,
                                                     -1.02435047e-06 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),
                                                   np.array([[  1.00000000e+00,   1.34401086e-07,  -1.18237335e-07,
                                                      0.7875 * dx],
                                                   [  1.34401115e-07,  -1.27438147e-01,   9.91846520e-01,
                                                      0.9125 * dy],
                                                   [  1.18237303e-07,  -9.91846520e-01,  -1.27438147e-01,
                                                      0.35 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),
                                                    ],
                                         assembly_robot_names = ['drc1','drc2','drc3'], #TODO: add back robot 3 soon
                                         name                 = 'sidelink1_6',
                                         robot_count = 2)
                                         
sidelink1_7 =                        AssemblyOperation(assembly_list        = [sidelink1_6, sidelink7, fastener8],
                                         assembly_poses       = [np.eye(4), 
                                                                 np.array([[ -9.99965998e-01,   8.24635185e-03,   9.83040956e-10,
                                                      3.72019023e-01 * dx],
                                                   [ -8.24635185e-03,  -9.99965998e-01,  -2.38414568e-07,
                                                      8.95629585e-01 * dy],
                                                   [ -9.83042888e-10,  -2.38414568e-07,   1.00000000e+00,
                                                     -1.32953525e-02 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),
                                                      np.array([[  1.00000000e+00,   1.34401086e-07,  -1.18237335e-07,
                                                      0.4875 * dx],
                                                   [  1.34401115e-07,  -1.27438147e-01,   9.91846520e-01,
                                                      0.9125 * dy],
                                                   [  1.18237303e-07,  -9.91846520e-01,  -1.27438147e-01,
                                                      0.35 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),
                                                    ],
                                         assembly_robot_names = ['drc1','drc2','drc3'], #TODO: add back robot 3 soon
                                         name                 = 'sidelink1_7',
                                         robot_count = 3)
                                         
sidelink1_8 =                        AssemblyOperation(assembly_list        = [sidelink1_7, sidelink8, fastener9],
                                         assembly_poses       = [np.eye(4), 
                                                                 np.array([[ -9.97984107e-01,   6.34643412e-02,   7.56555941e-09,
                                                      6.84092194e-02 * dx],
                                                   [ -6.34643412e-02,  -9.97984107e-01,  -2.38178194e-07,
                                                      8.88728976e-01 * dy],
                                                   [ -7.56551413e-09,  -2.38178196e-07,   1.00000000e+00,
                                                      2.25624442e-02 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),
                                                      np.array([[  1.00000000e+00,   1.34401086e-07,  -1.18237335e-07,
                                                      0.1875 * dx],
                                                   [  1.34401115e-07,  -1.27438147e-01,   9.91846520e-01,
                                                      0.9125 * dy],
                                                   [  1.18237303e-07,  -9.91846520e-01,  -1.27438147e-01,
                                                      0.35 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),
                                                   ],
                                         assembly_robot_names = ['drc1','drc2','drc3'], #TODO: add back robot 3 soon
                                         name                 = 'sidelink1_8',
                                         robot_count = 3)
                                         
sidelink1_C3 =                          AssemblyOperation(assembly_list        = [sidelink1_8, cornerlink3, fastener10],
                                         assembly_poses       = [np.eye(4), 
                                                                 np.array([[-0.99615691,  0.08736788,  0.00618545, -0.14087725 * dx],
                                                   [-0.08755663, -0.99148376, -0.09640433,  0.878775  * dy],
                                                   [-0.00228987, -0.09657541,  0.99532304,  0.00758945 * dz],
                                                   [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                                                   np.array([[  1.00000000e+00,   1.34401086e-07,  -1.18237335e-07,
                                                      -0.1125 * dx],
                                                   [  1.34401115e-07,  -1.27438147e-01,   9.91846520e-01,
                                                      0.9125 * dy],
                                                   [  1.18237303e-07,  -9.91846520e-01,  -1.27438147e-01,
                                                      0.35 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),
                                                    ],
                                         assembly_robot_names = ['drc1','drc2','drc3'], #TODO: add back robot 3 soon
                                         name                 = 'sidelink1_C3',
                                         robot_count = 3)
                                         
sidelink1_9 =                         AssemblyOperation(assembly_list        = [sidelink1_C3, sidelink9, fastener11],
                                         assembly_poses       = [np.eye(4), 
                                                                 np.array([[  2.98583872e-02,   9.99554139e-01,   1.19156142e-07,
                                                     -2.50123024e-01 * dx],
                                                   [ -9.99554139e-01,   2.98583872e-02,  -1.15649909e-07,
                                                      5.98674715e-01 * dy],
                                                   [ -1.19156156e-07,  -1.15649895e-07,   1.00000000e+00,
                                                      1.27371754e-06 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),
                                                      np.array([[  1.00000000e+00,   1.34401086e-07,  -1.18237335e-07,
                                                      -0.2625 * dx],
                                                   [  1.34401115e-07,  -1.27438147e-01,   9.91846520e-01,
                                                      0.7625 * dy],
                                                   [  1.18237303e-07,  -9.91846520e-01,  -1.27438147e-01,
                                                      0.35 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),
                                                    ],
                                         assembly_robot_names = ['drc1','drc2','drc3'], #TODO: add back robot 3 soon
                                         name                 = 'sidelink1_9',
                                         robot_count = 3)
                                         
sidelink1_10 =                         AssemblyOperation(assembly_list        = [sidelink1_9, sidelink10, fastener12],
                                         assembly_poses       = [np.eye(4), 
                                                                 np.array([[  3.41403774e-02,   9.99417047e-01,   1.19139707e-07,
                                     -2.40922093e-01 * dx],
                                   [ -9.99417047e-01,   3.41403774e-02,  -1.15139511e-07,
                                      2.86472082e-01 * dy],
                                   [ -1.19139865e-07,  -1.15139348e-07,   1.00000000e+00,
                                      3.47708315e-02 * dz],
                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                      1.00000000e+00]]),
                                      np.array([[  1.00000000e+00,   1.34401086e-07,  -1.18237335e-07,
                                                      -0.2625 * dx],
                                                   [  1.34401115e-07,  -1.27438147e-01,   9.91846520e-01,
                                                      0.4625 * dy],
                                                   [  1.18237303e-07,  -9.91846520e-01,  -1.27438147e-01,
                                                      0.35 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]])
                                                    ],
                                         assembly_robot_names = ['drc1','drc2','drc3'], #TODO: add back robot 3 soon
                                         name                 = 'sidelink1_10',
                                         robot_count = 3)
                                         
sidelink1_C4 =                         AssemblyOperation(assembly_list        = [sidelink1_10, cornerlink4, fastener13, fastener14],
                                         assembly_poses       = [np.eye(4), 
                                                                 np.array([[-0.03737369,  0.99624535, -0.07809233, -0.23465842 * dx],
                                   [-0.99930043, -0.03715273,  0.00428095,  0.1235164 * dy],
                                   [ 0.00136353,  0.0781977 ,  0.99693694,  0.01610716 * dz],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                                   np.array([[  1.00000000e+00,   1.34401086e-07,  -1.18237335e-07,
                                                      -0.2625 * dx],
                                                   [  1.34401115e-07,  -1.27438147e-01,   9.91846520e-01,
                                                      0.1625 * dy],
                                                   [  1.18237303e-07,  -9.91846520e-01,  -1.27438147e-01,
                                                      0.35 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),
                                   np.array([[  1.00000000e+00,   1.34401086e-07,  -1.18237335e-07,
                                                      -0.1125 * dx],
                                                   [  1.34401115e-07,  -1.27438147e-01,   9.91846520e-01,
                                                      0.0 * dy],
                                                   [  1.18237303e-07,  -9.91846520e-01,  -1.27438147e-01,
                                                      0.35 * dz],
                                                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,
                                                      1.00000000e+00]]),
                                                        ],
                                         assembly_robot_names = ['drc1','drc2','drc3', 'drc4'], #TODO: add back robot 3 soon
                                         name                 = 'sidelink1_C4',
                                         robot_count = 3)
                                         

                                         #TODO: Add one more fastener in this last one and one more robot to "close the loop"
                                        
"""                                         
"""                                         
assembly_operations = [sidelink1_2, sidelink1_3, sidelink1_C1, sidelink1_4, sidelink1_5, 
                       sidelink1_C2, sidelink1_6, sidelink1_7, sidelink1_8, sidelink1_C3,
                       sidelink1_9, sidelink1_10, sidelink1_C4]
#assembly_operations = [sidelink1_2, sidelink1_3, sidelink1_C1]
 
#assembly_operations = [sidelink1_2, sidelink1_3, sidelink1_C1]                                        

robot_start_poses = {'drc1':np.array([[ 1.        ,  0.        ,  0.        ,  -3.0],
                                      [ 0.        ,  1.        ,  0.        ,  -3.0],
                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
                                      [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                     'drc2':np.array([[ 1.        ,  0.        ,  0.        , -3.0 ],
                                      [ 0.        ,  1.        ,  0.        ,  -3.50      ],
                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
                                      [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                     'drc3':np.array([[ 1.        ,  0.        ,  0.        ,  -4.0      ],
                                      [ 0.        ,  1.        ,  0.        ,  -4.0      ],
                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
                                      [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                     'drc4':np.array([[ 1.        ,  0.        ,  0.        ,  -3.5      ],
                                      [ 0.        ,  1.        ,  0.        ,  -3.0      ],
                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
                                      [ 0.        ,  0.        ,  0.        ,  1.        ]]),
                     'drc5':np.array([[ 1.        ,  0.        ,  0.        ,  3.5      ],
                                      [ 0.        ,  1.        ,  0.        ,  3.0      ],
                                      [ 0.        ,  0.        ,  1.        ,  0.        ],
                                      [ 0.        ,  0.        ,  0.        ,  1.        ]])
                    }




