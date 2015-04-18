#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import IPython
import random
import time

import openravepy as orpy

from youbotpy import youbotik as yik
from tsr import GraspTSR,LookAtTSR

secondgoal = False

class JointGoalSearch:

    def __init__(self,env,youbots):
        self.env = env
        self.youbots = youbots
        yik.init(youbots[youbots.keys()[0]]) # does not matter which robot we use to initialize since all youbots have same kinematics.

    def SampleBaseAndArmGivenEEPose(self,rob, eepose):
        #start = time.time()
        base_poses,arm_configs=yik.FindIKAndBaseSolutions(rob,eepose,returnfirst=True,checkenvcollision=False,randomize=True)
        #print "IK generation took %f sec."%(time.time()-start)
        if base_poses is None or len(base_poses) == 0:
            print 'Failed to find ik.'
            return None,None
        randindex = random.randint(0,len(base_poses)-1)
        base_pose = base_poses[randindex]
        arm_config = np.array(arm_configs[randindex])
        base_config = np.array([0.,0.,0.])
        base_config[:2] = base_pose[:2,3]
        base_config[2] = np.arctan2(base_pose[1,0],base_pose[0,0])
        return base_config,arm_config

    def GetGoal(self,env,params,yname,timeout=100.0):
        starttime = time.time()
        with env:
            basegoal = None
            armgoal = None
            while (time.time() - starttime) < timeout:
               s = self.GetTSRSample(env,params,yname)
               #h = orpy.misc.DrawAxes(env,s,0.25,2.0)
               basegoal,armgoal = self.SampleBaseAndArmGivenEEPose(env.GetRobot(yname),s)
               if basegoal is not None:
                   return np.array(basegoal),np.array(armgoal)
            return None,None

    def GetFeasibleGoal(self,env,params,youbots,grabdict,timeout=100.0):
        starttime = time.time()
        while (time.time() - starttime) < timeout:
            with env:
                basesamples = {}
                armsamples = {}
                feasible = True
                for y in youbots:
                    basesample,armsample = self.GetGoal(env,params,y,timeout)
                    if basesample is None:
                        feasible = False
                        print 'no goal for %s'%(y)
                        break
                    pose = np.eye(4)
                    pose[:3,:3] = orpy.rotationMatrixFromAxisAngle(basesample[2]*np.array([0.,0.,1.]))
                    pose[:2,3] = basesample[:2]
                    youbots[y].SetTransform(pose)
                    youbots[y].SetDOFValues(armsample[:5],[0,1,2,3,4])
                    basesamples[y] = basesample
                    armsamples[y] = armsample
    
                env.UpdatePublishedBodies() # TODO remove for timing
    
                if not feasible:
                    break
            
                try:
                    for y in grabdict:
                        grabobjs = grabdict[y]
                        for grabobj in grabobjs:
                            youbots[y].Grab(grabobj)
                    for y in youbots:
                        collision = env.CheckCollision(youbots[y])
                        if collision:
                            feasible = False
                            #time.sleep(0.1) 
                            break
                finally:
                    for y in grabdict:
                        grabobjs = grabdict[y]
                        for grabobj in grabobjs:
                            youbots[y].Release(grabobj)
            
            if feasible:
                return basesamples,armsamples
        return None,None
     
    
    def GetTSRSample(self, env, params, yname):
        ladder = env.GetKinBody('ladder')
        ladderpose = ladder.GetTransform()
        wingskin_goal = np.dot(ladderpose, params['wingskin_in_ladder']) 
        if secondgoal:
            tsr = GraspTSR(params[yname+'_goal_tsr2']['T0_w_obj'],
                           params[yname+'_goal_tsr2']['Tw_e_list'],
                           params[yname+'_goal_tsr2']['Bw_list'],
                           params[yname+'_goal_tsr2']['grab_obj'])
        else:
            tsr = GraspTSR(params[yname+'_goal_tsr']['T0_w_obj'],
                           params[yname+'_goal_tsr']['Tw_e_list'],
                           params[yname+'_goal_tsr']['Bw_list'],
                           params[yname+'_goal_tsr']['grab_obj'])
        with env:
            env.GetKinBody('wingskin').SetTransform(wingskin_goal)
            s = tsr.Sample(env)
        return s
    
    def GetHardGoal(self, params, yname):
        goal = params[yname+'_goal_arm_config']
        base_goal = [np.array(params[yname+'_goal_base_pose'])[0,3], 
                     np.array(params[yname+'_goal_base_pose'])[1,3], 
                     orpy.axisAngleFromRotationMatrix(np.array(params[yname+'_goal_base_pose'])[:3,:3])[2]]
        goal.extend(base_goal)
        return goal


