#!/usr/bin/env python
# -*- coding: utf-8 -*-

# TODO part initial poses and env objects
#     - assembly config search cok uzun suruyo.. neden? bi bak bakalim goalda ortalama kac config ariyoruz. kaci valid. grasp'tan ik mappingi mi sparse yoksa o manifolddaki collide etmeyenler mi sparse? temel soru bu.. bunu anlayabilirsem, ve gercekten bu kadar uzun suruyosa, buna bi cozum bulabilirim..
#     - bir de ik bulunca butun solutionlari mi denesem.. fark eder mi..?
#     - once grasp bulup sonra base bulmaktansa, once base bulup, kesismedikleri durumda grasp ara goal'da. bu muhtemelen baya daha hizli olcak. ama bunu genel bi algoritmaya donusturebilir miyiz bilmiyorum.. buyuk olan parcalari (zor constraintleri) once satisfy etmek tarzi olabilir, ama emin degilim.. bu ogrenilebilir mi?
# TODO generate facedown faceup examples and save in pickle files (put black cylinder in back?)
#
#***
# TODO
# base samplingli yaz. -> gercekten hizli mi gor. yeterince hizli degilse dusun, incele..
#   hizli degil. ik bulamiyo cogunlukla. eski usul, ama bi grasp icin butun baseleri cikarip denemeyi dene..
#   Once bu: chair side icin diger grasplari da yaz, bakalim yeterince hizli olcak mi..
#      - olmadi
#      - butun base pozisyonlarini almali denesem mi?
#   belki: !! her robot icin reachability gibi, minimum occupied volume cikar. buna gore intersection cek edip, arayip bul.
# chair baska objeli env. ve hiz.
# regrasping.
# multistep.

# ross: - minimum occupied volume bi metric olabilir. constraint ordering icin. grasplarin birbirine uzakligi: basei nerelere yerlestiriyolar?
#       - multistepteki ve regraspingdeki butun grasplari hep birlikte optimize edebilir misin? bu ordeingi kullanrak, belki ogrenerek.
# andy: geometric sekillerle approximate edip optimization yapabilir misin?
# ross: eger bi set bi robot icin digerlerinden daha buyukse onu daha fazla sample etmek lazim (epsilon degisiyosa her sample'da, tekrar tekrar sample'a gerek yok). bu metric ne? nasil estimate ederiz? kullanma policysi ne? --> bunu dusun.. dene.. multistep acisindan da dusun..
# ross: goalset chomp kullanabilir miyiz?

import numpy as np

from tsr import GraspTSR,LookAtTSR
from Assembly import Assembly

envfile = 'environments/chair.env.xml'
assembly = Assembly(object_names=['fastener','side','back'],
                    object_relative_poses={                       
                        'fastener': np.array([[ 1.    ,  0.    ,  0.    , -0.09  ],
                                              [ 0.    ,  1.    ,  0.    , -0.07  ],
                                              [ 0.    ,  0.    ,  1.    ,  0.28  ],
                                              [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                        'side':  np.array([[ 1.    ,  0.    ,  0.    ,  0.    ],
                                           [ 0.    ,  1.    ,  0.    ,  0.    ],
                                           [ 0.    ,  0.    ,  1.    ,  0.    ],
                                           [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                        'back':np.array([[ 1.    ,  0.    ,  0.    , -0.09  ],
                                         [ 0.    ,  1.    ,  0.    ,  0.095 ],
                                         [ 0.    ,  0.    ,  1.    ,  0.15  ],
                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                        },
                    fixed_objects = []
                    )

assembly_pose = np.array([[1.0,  0.0,  0.0, -1.50],
                          [0.0,  1.0,  0.0,  0.00],
                          [0.0,  0.0,  1.0,  0.10],
                          [0.0,  0.0,  0.0,  1.0 ]])

task_regions = {'side': GraspTSR(T0_w_obj='side',
                                     Tw_e_list=[
                                               np.array([[ 1.    ,  0.    ,  0.    ,  0.    ],
                                                         [ 0     ,  0.    ,  1.    , -0.03  ],
                                                         [ 0.    , -1.    ,  0.    ,  0.14  ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[-1.    ,  0.    ,  0.    ,  0.    ], # flipped
                                                         [ 0     ,  0.    ,  1.    , -0.03  ],
                                                         [ 0.    ,  1.    ,  0.    ,  0.14  ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),

                                               np.array([[ 1.    ,  0.    ,  0.    ,  0.    ],
                                                         [ 0     ,  0.    , -1.    ,  0.03  ],
                                                         [ 0.    ,  1.    ,  0.    ,  0.14  ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[-1.    ,  0.    ,  0.    ,  0.    ], # flipped
                                                         [ 0     ,  0.    , -1.    ,  0.03  ],
                                                         [ 0.    , -1.    ,  0.    ,  0.14  ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),


                                               np.array([[ 0.    ,  1.    ,  0.    ,  0.09  ],
                                                         [ 0     ,  0.    , -1.    ,  0.03  ],
                                                         [-1.    ,  0.    ,  0.    ,  0.065 ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 0.    ,  1.    ,  0.    , -0.09  ],
                                                         [ 0     ,  0.    , -1.    ,  0.03  ],
                                                         [-1.    ,  0.    ,  0.    ,  0.065 ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 0.    ,  1.    ,  0.    , -0.09  ],
                                                         [ 0     ,  0.    , -1.    ,  0.03  ],
                                                         [-1.    ,  0.    ,  0.    ,  0.225 ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),

                                               np.array([[ 0.    , -1.    ,  0.    ,  0.09  ], # flipped
                                                         [ 0     ,  0.    , -1.    ,  0.03  ],
                                                         [ 1.    ,  0.    ,  0.    ,  0.065 ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 0.    , -1.    ,  0.    , -0.09  ],
                                                         [ 0     ,  0.    , -1.    ,  0.03  ],
                                                         [ 1.    ,  0.    ,  0.    ,  0.065 ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 0.    , -1.    ,  0.    , -0.09  ],
                                                         [ 0     ,  0.    , -1.    ,  0.03  ],
                                                         [ 1.    ,  0.    ,  0.    ,  0.225 ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),

                                               np.array([[ 0.    , -1.    ,  0.    ,  0.09  ],
                                                         [ 0     ,  0.    ,  1.    , -0.03  ],
                                                         [-1.    ,  0.    ,  0.    ,  0.065 ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 0.    , -1.    ,  0.    , -0.09  ],
                                                         [ 0     ,  0.    ,  1.    , -0.03  ],
                                                         [-1.    ,  0.    ,  0.    ,  0.065 ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 0.    , -1.    ,  0.    , -0.09  ],
                                                         [ 0     ,  0.    ,  1.    , -0.03  ],
                                                         [-1.    ,  0.    ,  0.    ,  0.225 ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),

                                               np.array([[ 0.    ,  1.    ,  0.    ,  0.09  ], # flipped
                                                         [ 0     ,  0.    ,  1.    , -0.03  ],
                                                         [ 1.    ,  0.    ,  0.    ,  0.065 ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 0.    ,  1.    ,  0.    , -0.09  ],
                                                         [ 0     ,  0.    ,  1.    , -0.03  ],
                                                         [ 1.    ,  0.    ,  0.    ,  0.065 ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 0.    ,  1.    ,  0.    , -0.09  ],
                                                         [ 0     ,  0.    ,  1.    , -0.03  ],
                                                         [ 1.    ,  0.    ,  0.    ,  0.225 ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),

                                               np.array([[ 0.    ,  0.    ,  1.    , -0.11  ],
                                                         [ 0     ,  1.    ,  0.    ,  0.    ],
                                                         [-1.    ,  0.    ,  0.    ,  0.15  ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 0.    ,  0.    ,  1.    , -0.11  ], # flipped
                                                         [ 0     , -1.    ,  0.    ,  0.    ],
                                                         [ 1.    ,  0.    ,  0.    ,  0.15  ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 0.    ,  0.    , -1.    ,  0.11  ],
                                                         [ 0     , -1.    ,  0.    ,  0.    ],
                                                         [-1.    ,  0.    ,  0.    ,  0.075 ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 0.    ,  0.    , -1.    ,  0.11  ], # flipped
                                                         [ 0     ,  1.    ,  0.    ,  0.    ],
                                                         [ 1.    ,  0.    ,  0.    ,  0.075 ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),

                                               np.array([[ 1.    ,  0.    ,  0.    ,  0.    ],
                                                         [ 0     ,  1.    ,  0.    ,  0.    ],
                                                         [ 0.    ,  0.    ,  1.    ,  0.12  ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[-1.    ,  0.    ,  0.    ,  0.    ], # flipped
                                                         [ 0     , -1.    ,  0.    ,  0.    ],
                                                         [ 0.    ,  0.    ,  1.    ,  0.12  ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 1.    ,  0.    ,  0.    ,  0.    ],
                                                         [ 0     , -1.    ,  0.    ,  0.    ],
                                                         [ 0.    ,  0.    , -1.    ,  0.16  ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[-1.    ,  0.    ,  0.    ,  0.    ], # flipped
                                                         [ 0     ,  1.    ,  0.    ,  0.    ],
                                                         [ 0.    ,  0.    , -1.    ,  0.16  ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                           ],  
                                     Bw_list= [
                                               [-0.06, 0.06,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.06, 0.06,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.06, 0.06,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.06, 0.06,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.05, 0.05,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.05, 0.05,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.05, 0.05,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.05, 0.05,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.05, 0.05,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.05, 0.05,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.05, 0.05,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.05, 0.05,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.05, 0.05,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.05, 0.05,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.05, 0.05,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.05, 0.05,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.15, 0.15,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.15, 0.15,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.06, 0.06,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.06, 0.06,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.06, 0.06,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.06, 0.06,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.06, 0.06,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                               [-0.06, 0.06,    0, 0,    0, 0,    0, 0,    -np.pi/36.0,np.pi/36.0,    0, 0],
                                         ]
                                    ),
                'fastener': GraspTSR(T0_w_obj='fastener',
                                     Tw_e_list=[
                                               np.array([[ 0.    ,  1.    ,  0.    ,  0.    ],
                                                         [-1.    ,  0.    ,  0.    ,  0.025 ],
                                                         [ 0.    ,  0.    ,  1.    ,  0.    ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 0.    ,  0.    ,  1.    ,  0.    ], # flipped
                                                         [ 1.    ,  0.    ,  0.    ,  0.025 ],
                                                         [ 0.    ,  1.    ,  0.    ,  0.    ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),

                                               np.array([[ 0.    , -1.    ,  0.    ,  0.    ],
                                                         [ 0.    ,  0.    ,  1.    , -0.01  ],
                                                         [-1.    ,  0.    ,  0.    ,  0.    ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                               np.array([[ 0.    , -1.    ,  0.    ,  0.    ],
                                                         [ 0.    ,  0.    , -1.    ,  0.06  ],
                                                         [ 1.    ,  0.    ,  0.    ,  0.    ],
                                                         [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                           ],
                                     Bw_list= [
                                               [-0.025, 0.025,    0, 0,    0, 0,   -np.pi, np.pi,  0, 0,   0, 0] ,
                                               [-0.025, 0.025,    0, 0,    0, 0,   -np.pi, np.pi,  0, 0,   0, 0] ,
                                               [ 0., 0.,    0, 0,    0, 0,    0, 0,   0, 0,   -np.pi, np.pi],
                                               [ 0., 0.,    0, 0,    0, 0,    0, 0,   0, 0,   -np.pi, np.pi]
                                              ]
                                    ),
                'back': GraspTSR(T0_w_obj='back',
                                 Tw_e_list=[
                                             np.array([[ 0.    ,  1.    ,  0.    ,  0.    ],
                                                       [ 0.    ,  0.    , -1.    ,  0.085 ],
                                                       [-1.    ,  0.    ,  0.    ,  0.075 ],
                                                       [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                             np.array([[ 0.    , -1.    ,  0.    ,  0.    ], # flipped
                                                       [ 0.    ,  0.    , -1.    ,  0.085 ],
                                                       [ 1.    ,  0.    ,  0.    ,  0.075 ],
                                                       [ 0.    ,  0.    ,  0.    ,  1.    ]]),

                                             np.array([[ 0.    , -1.    ,  0.    ,  0.    ],
                                                       [ 0.    ,  0.    ,  1.    , -0.085 ],
                                                       [-1.    ,  0.    ,  0.    ,  0.075 ],
                                                       [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                             np.array([[ 0.    ,  1.    ,  0.    ,  0.    ], # flipped
                                                       [ 0.    ,  0.    ,  1.    , -0.085 ],
                                                       [ 1.    ,  0.    ,  0.    ,  0.075 ],
                                                       [ 0.    ,  0.    ,  0.    ,  1.    ]]),

                                             np.array([[ 0.    ,  1.    ,  0.    ,  0.    ],
                                                       [ 1.    ,  0.    ,  0.    ,  0.    ],
                                                       [ 0.    ,  0.    , -1.    ,  0.16  ],
                                                       [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                             np.array([[ 0.    , -1.    ,  0.    ,  0.    ], # flipped
                                                       [-1.    ,  0.    ,  0.    ,  0.    ],
                                                       [ 0.    ,  0.    , -1.    ,  0.16  ],
                                                       [ 0.    ,  0.    ,  0.    ,  1.    ]]),

                                             np.array([[ 0.    ,  1.    ,  0.    ,  0.    ],
                                                       [-1.    ,  0.    ,  0.    ,  0.    ],
                                                       [ 0.    ,  0.    ,  1.    , -0.01  ],
                                                       [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                             np.array([[ 0.    , -1.    ,  0.    ,  0.    ], # flipped
                                                       [ 1.    ,  0.    ,  0.    ,  0.    ],
                                                       [ 0.    ,  0.    ,  1.    , -0.01  ],
                                                       [ 0.    ,  0.    ,  0.    ,  1.    ]]),
                                           ],
                                 Bw_list= [
                                            [-0.07, 0.07,    0, 0,    0, 0,    0, 0,    -np.pi/36.0, np.pi/36.0,    0, 0],
                                            [-0.07, 0.07,    0, 0,    0, 0,    0, 0,    -np.pi/36.0, np.pi/36.0,    0, 0], 
                                            [-0.07, 0.07,    0, 0,    0, 0,    0, 0,    -np.pi/36.0, np.pi/36.0,    0, 0], 
                                            [-0.07, 0.07,    0, 0,    0, 0,    0, 0,    -np.pi/36.0, np.pi/36.0,    0, 0], 
                                            [-0.07, 0.07,    0, 0,    0, 0,    0, 0,    -np.pi/36.0, np.pi/36.0,    0, 0], 
                                            [-0.07, 0.07,    0, 0,    0, 0,    0, 0,    -np.pi/36.0, np.pi/36.0,    0, 0], 
                                            [-0.07, 0.07,    0, 0,    0, 0,    0, 0,    -np.pi/36.0, np.pi/36.0,    0, 0], 
                                            [-0.07, 0.07,    0, 0,    0, 0,    0, 0,    -np.pi/36.0, np.pi/36.0,    0, 0], 
                                          ]
                                )
        }

robots = ['drc1','drc2','drc3']
part_responsibilities = {'back':'drc1', 
                         'side':'drc2', 
                         'fastener':'drc3'}

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



