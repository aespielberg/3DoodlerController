#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import random
from itertools import izip,product

import openravepy as orpy

class GraspRegions(object):
    def __init__(self,Tw_e_list,Bw_list):
        self.Tw_e_list = Tw_e_list
        self.Bw_list = Bw_list

    def SampleUniformGrid(self,start,end,resolution):
        grid = []
        point = start+resolution
        while point <= end:
            grid.append(point)
            point += resolution
        if len(grid) < 1:
            # resolution larger than range. Sample one randomly
            grid.append(random.uniform(start, end))
        return grid

    def UniformSamples(self,translational_resolution,rotational_resolution):
        samples = []
        for Tw_e,Bw in izip(self.Tw_e_list,self.Bw_list):
            Xs = self.SampleUniformGrid(Bw[0],Bw[1],translational_resolution)
            y = 0.
            z = 0.
            #roll = random.uniform(Bw[6], Bw[7])*np.array([1.,0.,0.])
            Rolls = self.SampleUniformGrid(Bw[6],Bw[7],rotational_resolution)
            Pitchs = self.SampleUniformGrid(Bw[8],Bw[9],rotational_resolution)
            #yaw = random.uniform(Bw[10], Bw[11])*np.array([0.,0.,1.])
            Yaws = self.SampleUniformGrid(Bw[10],Bw[11],rotational_resolution)
            for x,pitch,yaw,roll in product(Xs,Pitchs,Yaws,Rolls):
                s = np.eye(4)
                pitch = pitch*np.array([0.,1.,0.])
                yaw = yaw*np.array([0.,0.,1.])
                roll = roll*np.array([1.,0.,0.])
                s[0,3] = x
                s[1,3] = y
                s[2,3] = z
                s[:3,:3] = np.dot(np.dot(orpy.rotationMatrixFromAxisAngle(roll),orpy.rotationMatrixFromAxisAngle(pitch)),orpy.rotationMatrixFromAxisAngle(yaw))
                s = np.dot(Tw_e,s)
                samples.append(s)
        return samples

    def SampleObjectRelative(self):
        # FIXME The sampling is not exactly uniform right now. Different line
        # of grasps can have different lengths.
        n_grasp_lines = len(self.Tw_e_list)
        random_line = random.randint(0,n_grasp_lines-1)
        Tw_e = self.Tw_e_list[random_line]
        Bw = self.Bw_list[random_line]
        randoffset = np.eye(4)
        xoffset = random.uniform(Bw[0], Bw[1])
        yoffset = random.uniform(Bw[2], Bw[3])
        zoffset = random.uniform(Bw[4], Bw[5])
        rolloffset = random.uniform(Bw[6], Bw[7])*np.array([1.,0.,0.])
        pitchoffset = random.uniform(Bw[8], Bw[9])*np.array([0.,1.,0.])
        yawoffset = random.uniform(Bw[10], Bw[11])*np.array([0.,0.,1.])
        randoffset[0,3] = xoffset
        randoffset[1,3] = yoffset
        randoffset[2,3] = zoffset
        randoffset[:3,:3] = np.dot(np.dot(orpy.rotationMatrixFromAxisAngle(rolloffset),orpy.rotationMatrixFromAxisAngle(pitchoffset)),orpy.rotationMatrixFromAxisAngle(yawoffset))
        sample = np.dot(Tw_e,randoffset)
        return sample

    def Visualize(self,kinbody,robot,nsamples=100):
        robotinworld = robot.GetTransform()
        worldinrobot = np.linalg.inv(robotinworld)
        eeinworld = robot.GetManipulators()[0].GetEndEffectorTransform()
        eeinrobot = np.dot(worldinrobot,eeinworld)
        robotinee = np.linalg.inv(eeinrobot)
        for s in range(nsamples):
            eeinw = np.dot(kinbody.GetTransform(),self.SampleObjectRelative())
            robotinw = np.dot(eeinw,robotinee)
            robot.SetTransform(robotinw)
            time.sleep(0.1)

