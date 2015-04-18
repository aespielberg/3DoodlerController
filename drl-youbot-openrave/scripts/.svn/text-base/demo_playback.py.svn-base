#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import IPython
import yaml
import sys
import random
import cPickle as pickle

import openravepy as orpy

import youbotpy
from youbotpy import youbotik as yik
from tsr import GraspTSR,LookAtTSR
from JointGoalSearch import JointGoalSearch
from wingdemo_misc import XYThetaToMatrix,MatrixToXYTheta
from AssemblyConfigurationPlanner import AssemblyConfigurationPlanner
import MultiYoubotPlanner

def load_pickled_demo(fname):
    f = open(fname,'rb')
    envfile = pickle.load(f)
    start_config = pickle.load(f)
    goal_config = pickle.load(f)
    basetrajs1 = pickle.load(f)
    armtrajs1 = pickle.load(f)
    basetrajs2 = pickle.load(f)
    armtrajs2 = pickle.load(f)
    assembly = pickle.load(f)
    assembly_pose = pickle.load(f)
    robots = pickle.load(f)
    part_responsibilities = pickle.load(f)
    robot_start_poses = pickle.load(f)
    f.close()
    # FIXME create a demo class?
    return envfile, start_config, goal_config, basetrajs1, armtrajs1, basetrajs2, armtrajs2, assembly, assembly_pose, robots, part_responsibilities, robot_start_poses

def run_pickled_demo(fname):
    envfile, start_config, goal_config, basetrajs1, armtrajs1, basetrajs2, armtrajs2, assembly, assembly_pose, robots, part_responsibilities, robot_start_poses = load_pickled_demo(fname)
    youbotenv = youbotpy.YoubotEnv(sim=True,viewer=True,env_xml=envfile, \
                                   youbot_names=robots)
    env = youbotenv.env
    youbots = youbotenv.youbots

    for y in basetrajs1:
        btraj = orpy.RaveCreateTrajectory(env,"")
        btraj.deserialize(basetrajs1[y])
        basetrajs1[y] = btraj
        atraj = orpy.RaveCreateTrajectory(env,"")
        atraj.deserialize(armtrajs1[y])
        armtrajs1[y] = atraj
    for y in basetrajs2:
        btraj = orpy.RaveCreateTrajectory(env,"")
        btraj.deserialize(basetrajs2[y])
        basetrajs2[y] = btraj
        atraj = orpy.RaveCreateTrajectory(env,"")
        atraj.deserialize(armtrajs2[y])
        armtrajs2[y] = atraj

    for name in youbots:
        youbots[name].SetTransform(robot_start_poses[name])
        youbotenv.MoveGripper(name,0.01,0.01) # open grippers

    raw_input('Hit Enter to run.')
    motionplanner = MultiYoubotPlanner.MultiYoubotPlanner(env,youbots)
    motionplanner.Execute(basetrajs1,armtrajs1)
    for obj in assembly.object_names:
        youbots[part_responsibilities[obj]].Grab(env.GetKinBody(obj))
    motionplanner.Execute(basetrajs2,armtrajs2)
    raw_input('Hit Enter to destroy environment.')
    youbotenv.env.Destroy()
    orpy.RaveDestroy()

IPython.embed()
