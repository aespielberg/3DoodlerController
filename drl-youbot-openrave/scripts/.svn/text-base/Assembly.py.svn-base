#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import random

import openravepy as orpy

class Assembly(object):
    def __init__(self, object_names, object_relative_poses,fixed_objects):
        self.object_names = object_names
        self.object_relative_poses = object_relative_poses
        self.fixed_objects = fixed_objects

    def Show(self,env,assembly_pose):
        with env:
            for n in self.object_names:
                pose = np.dot(assembly_pose,self.object_relative_poses[n])
                env.GetKinBody(n).SetTransform(pose)


