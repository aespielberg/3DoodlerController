#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import random
import IPython
from itertools import izip,product

import openravepy as orpy

import GraspRegions
import Assembly

from allgraspregions import all_grasp_regions

#handtr = np.array([[  1.00000000e+00,   0.00000000e+00,   0.00000000e+00, -1.17930460e+00],
#                   [  0.00000000e+00,   1.00000000e+00,  -6.35298936e-15, 4.11785269e+00],
#                   [  0.00000000e+00,   6.35298936e-15,   1.00000000e+00, 1.38644680e-01],
#                   [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]])
#tr=handtr
#grasp_generator.hand.SetTransform(tr)
#rstr = np.array([[ 1.        ,  0.        ,  0.        , -1.1704365 ],
#                 [ 0.        ,  1.        ,  0.        ,  4.11590052],
#                 [ 0.        ,  0.        ,  1.        ,  0.09999912],
#                 [ 0.        ,  0.        ,  0.        ,  1.        ]])
#env.GetKinBody('rightside').SetTransform(rstr)
#
#hand2 =  env.ReadRobotURI('../models/robots/kuka-youbot-gripper.robot.xml')
#hand2.SetName('hand2')
#env.Add(hand2,True)
#tr[2,3] = tr[2,3] + random.random()*0.05
#tr[1,3] = tr[1,3] + random.random()*0.05
#hand2.SetTransform(tr)
#hand2.SetDOFValues([0.01,0.01])
#
#hand3 =  env.ReadRobotURI('../models/robots/kuka-youbot-gripper.robot.xml')
#hand3.SetName('hand3')
#env.Add(hand3,True)
#tr[2,3] = tr[2,3] + random.random()*0.05
#tr[1,3] = tr[1,3] + random.random()*0.05
#hand3.SetTransform(tr)
#hand3.SetDOFValues([0.01,0.01])
#
#hand4 =  env.ReadRobotURI('../models/robots/kuka-youbot-gripper.robot.xml')
#hand4.SetName('hand4')
#env.Add(hand4,True)
#hand4.SetTransform(tr)
#tr[2,3] = tr[2,3] + random.random()*0.05
#tr[1,3] = tr[1,3] + random.random()*0.05
#hand4.SetDOFValues([0.01,0.01])
#
#hand5 =  env.ReadRobotURI('../models/robots/kuka-youbot-gripper.robot.xml')
#hand5.SetName('hand5')
#env.Add(hand5,True)
#hand5.SetTransform(tr)
#tr[2,3] = tr[2,3] + random.random()*0.05
#tr[1,3] = tr[1,3] + random.random()*0.05
#hand5.SetDOFValues([0.01,0.01])
#
#hand6 =  env.ReadRobotURI('../models/robots/kuka-youbot-gripper.robot.xml')
#hand6.SetName('hand6')
#env.Add(hand6,True)
#hand6.SetTransform(tr)
#tr[2,3] = tr[2,3] + random.random()*0.05
#tr[1,3] = tr[1,3] + random.random()*0.05
#hand6.SetDOFValues([0.01,0.01])
#
#hand7 =  env.ReadRobotURI('../models/robots/kuka-youbot-gripper.robot.xml')
#hand7.SetName('hand7')
#env.Add(hand7,True)
#tr[2,3] = tr[2,3] + random.random()*0.05
#tr[1,3] = tr[1,3] + random.random()*0.05
#hand7.SetTransform(tr)
#hand7.SetDOFValues([0.01,0.01])
#
#In [89]: hs = [grasp_generator.hand,hand2,hand3,hand4,hand5,hand6,hand7]
#
#In [90]: for h in hs:                                                   
#        print h.GetTransform()
#           ....:     
#               [[  1.00000000e+00   0.00000000e+00   0.00000000e+00  -8.79319131e-01]
#                        [  0.00000000e+00   1.00000000e+00  -6.35298936e-15   4.31507587e+00]
#                         [  0.00000000e+00   6.35298936e-15   1.00000000e+00   4.47175086e-01]
#                          [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
#               [[ -9.98597057e-01   1.12027420e-07  -5.29520417e-02  -8.56262386e-01]
#                        [ -5.29038153e-02   4.26673921e-02   9.97687667e-01   4.58731651e+00]
#                         [  2.25943730e-03   9.99089332e-01  -4.26075262e-02   2.41272986e-01]
#                          [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
#               [[  1.00000000e+00   0.00000000e+00   0.00000000e+00  -1.13820732e+00]
#                        [  0.00000000e+00   1.00000000e+00  -2.42405752e-15   4.11041546e+00]
#                         [  0.00000000e+00   2.42405752e-15   1.00000000e+00   1.41889498e-01]
#                          [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
#               [[ -1.89515391e-02   9.82906465e-01   1.83127606e-01  -1.27776623e+00]
#                        [  1.01178341e-01   1.84105624e-01  -9.77685053e-01   4.20349741e+00]
#                         [ -9.94687781e-01  -8.92452364e-08  -1.02937930e-01   1.47334576e-01]
#                          [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
#               [[  1.00000000e+00   0.00000000e+00   0.00000000e+00  -8.65938425e-01]
#                        [  0.00000000e+00   1.00000000e+00  -6.35298936e-15   4.22300863e+00]
#                         [  0.00000000e+00   6.35298936e-15   1.00000000e+00   4.45155531e-01]
#                          [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
#               [[  2.33261884e-02   6.35126079e-15   9.99727907e-01  -1.36461306e+00]
#                        [ -9.00031636e-15   1.00000000e+00  -6.14298917e-15   4.11400986e+00]
#                         [ -9.99727907e-01  -8.85457491e-15   2.33261884e-02   3.56783897e-01]
#                          [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
#               [[ -5.23907638e-02  -6.34426649e-15  -9.98626661e-01  -9.75814700e-01]
#                        [ -1.56930293e-21   1.00000000e+00  -6.35299122e-15   4.11189318e+00]
#                         [  9.98626661e-01  -3.32836495e-16  -5.23907638e-02   1.42811373e-01]
#                          [  0.00000000e+00   0.00000000e+00   0.00000000e+00   1.00000000e+00]]
#

class GraspGenerator(object):

    def __init__(self,env):
        self.env = env
        self.hand = env.ReadRobotURI('../models/robots/kuka-youbot-gripper.robot.xml')
        self.hand.SetName('grasp_generator_hand')
        self.env.Add(self.hand,True)
        self.hand.SetDOFValues([0.01,0.01]) # open gripper
        self.hand.SetVisible(False)
        self.hand.Enable(False)

    #def GenerateAssemblyGrasps(self,assembly):
    #    assembly.part_grasps = []
    #    #print 'Generating grasps for assembly: ',assembly.name
    #    for part,pose in izip(assembly.part_list,assembly.pose_per_part):
    #        part.SetGrasps(self.GeneratePartGrasps(part))
    #        #print '\tusing part ',part.name, ' with ',len(part.grasps), 'grasps.'
    #        if 'fastener' in part.name: # FIXME hack to exclude fastener grasps in assembly grasps. Fix this by defining the output of assembly operation as a different config of parts.
    #            continue
    #        for g in part.grasps:
    #            g_in_asm = np.dot(pose,g)
    #            with self.hand:
    #                self.hand.Enable(True)
    #                self.hand.SetTransform(g_in_asm)
    #                with Assembly.AssemblyPoseSaver(assembly):
    #                    assembly.SetTransform(np.eye(4))
    #                    self.hand.Grab(part.body)
    #                    try:
    #                        if assembly.CheckCollision(self.env,self.hand):
    #                            #print '\tcollision'
    #                            continue
    #                        assembly.part_grasps.append(Assembly.PartGrasp(part.name,g)) 
    #                    finally:
    #                        self.hand.Release(part.body)
    #    #print 'Generated ',len(assembly.part_grasps),' grasps for assembly ',assembly.name

    def GeneratePartGrasps(self,part):
        print 'Generating grasps for ',part.name
        if part.grasps is None:
            translational_resolution = 0.05
            rotational_resolution    = np.pi/9.0
            return all_grasp_regions[part.name].UniformSamples(translational_resolution,rotational_resolution)
        else:
            return part.grasps

    def Visualize(self,assembly_op):
        self.hand.SetVisible(True)
        for part in assembly_op.GetPartList():
            for g in part.grasps:
                part_pose = self.env.GetKinBody(part.name).GetTransform()
                ee = self.hand.GetManipulators()[0].GetEndEffectorTransform()
                hand_in_ee = np.dot(np.linalg.inv(ee),self.hand.GetTransform())
                with self.hand:
                    self.hand.SetTransform(np.dot(np.dot(part_pose,g),hand_in_ee))
                    self.env.UpdatePublishedBodies()
                    time.sleep(0.3)
                    # raw_input('grasp example')
        self.hand.SetVisible(False)

