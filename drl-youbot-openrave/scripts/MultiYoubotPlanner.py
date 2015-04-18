#!/usr/bin/env python
# -*- coding: utf-8 -*-

import openravepy as orpy
import numpy as np
import time
import IPython

from wingdemo_misc import XYThetaToMatrix

def SetTransformAsDOF(robot,pose):
    x = pose[0,3]
    y = pose[1,3]
    theta = np.arctan2(pose[1,0],pose[0,0])
    robot.SetDOFValues([x,y,theta],[robot.GetJoint('basex').GetDOFIndex(),
                                    robot.GetJoint('basey').GetDOFIndex(),
                                    robot.GetJoint('baserotation').GetDOFIndex()])



class MultiYoubotPlanner(object):

    def __init__(self,env,youbots,parallelplanning=False):
        self.env = env
        self.planner = orpy.RaveCreatePlanner(self.env,'birrt')
        self.youbots = youbots
        self.parallelplanning = parallelplanning
        #self.youbots = {} # TODO remove after debug
        #self.youbots['drc1'] = youbots['drc1'] # TODO remove
        #self.youbots['drc2'] = youbots['drc2'] # TODO remove

        if self.parallelplanning:
            self.awaypose = np.eye(4)
            self.awaypose[:2,3] = np.array([100.0, 100.0])

            self.youbots_base_joints = {}
            for yname in self.youbots:
                print yname
                youbot_base_joints = env.ReadRobotURI('robots/kuka-youbot-baseasjoints.robot.xml')
                youbot_base_joints.SetName(yname+'_baseasjoints')
                self.env.Add(youbot_base_joints,True)
                youbot_base_joints.SetTransform(self.awaypose)
                youbot_base_joints.GetLink('base0').Enable(False)
                youbot_base_joints.GetLink('base1').Enable(False)
                youbot_base_joints.GetLink('base2').Enable(False)
                self.youbots_base_joints[yname] = youbot_base_joints
        else:
            self.planners = {}
            for y in youbots:
                self.planners[y] = orpy.interfaces.BaseManipulation(youbots[y],plannername='BiRRT')


    def Plan(self,basegoals,armgoals):
        if self.parallelplanning:
            return self.PlanParallel(basegoals,armgoals)
        else:
            return self.PlanSequential(basegoals,armgoals)

    def PlanParallel(self,basegoals,armgoals):
        if not self.parallelplanning:
            print 'Error: PlanParallel is called even though planner was initialized not parallel.'
            raise Exception('Error: PlanParallel is called even though planner was initialized not parallel.')
        initial_base_poses = {}
        initial_arm_configs = {}
        grabbed = {}
        for yname in self.youbots:
            initial_base_poses[yname] = self.youbots[yname].GetTransform()
            initial_arm_configs[yname] = self.youbots[yname].GetDOFValues()
            grabbed[yname] = self.youbots[yname].GetGrabbed() 
        try:
            for yname in self.youbots:
                self.youbots[yname].ReleaseAllGrabbed()
                self.youbots[yname].SetTransform(self.awaypose)
            raw_input('Hit enter to bring basej robots')
            for yname in self.youbots:
                self.youbots_base_joints[yname].SetDOFValues([0.,0.,0.],[0,1,2]) # reset the base joints
                self.youbots_base_joints[yname].SetDOFValues(initial_arm_configs[yname],
                                                             np.append(self.youbots_base_joints[yname].GetManipulators()[0].GetArmIndices(),
                                                                       self.youbots_base_joints[yname].GetManipulators()[0].GetGripperIndices()))
                self.youbots_base_joints[yname].SetTransform(np.eye(4)) # FIXME different height? probably not, but check.
                #if yname == 'drc2':
                #    remove=np.eye(4)
                #    remove[0,3] = 5.5
                #    self.youbots_base_joints[yname].SetTransform(remove) # FIXME different height? probably not, but check.
                SetTransformAsDOF(self.youbots_base_joints[yname],initial_base_poses[yname])
                if grabbed[yname] is not None:
                    for o in grabbed[yname]:
                        self.youbots_base_joints[yname].Grab(o)
            raw_input('Hit enter to plan.')
            configspec = orpy.ConfigurationSpecification()
            initialconfig = np.array([])
            goal = np.array([])
            for yname in self.youbots:
                self.youbots_base_joints[yname].SetActiveDOFs([self.youbots_base_joints[yname].GetJoint('basex').GetDOFIndex(),
                                                               self.youbots_base_joints[yname].GetJoint('basey').GetDOFIndex(),
                                                               self.youbots_base_joints[yname].GetJoint('baserotation').GetDOFIndex(),
                                                               self.youbots_base_joints[yname].GetJoint('j0').GetDOFIndex(),
                                                               self.youbots_base_joints[yname].GetJoint('j1').GetDOFIndex(),
                                                               self.youbots_base_joints[yname].GetJoint('j2').GetDOFIndex(),
                                                               self.youbots_base_joints[yname].GetJoint('j3').GetDOFIndex(),
                                                               self.youbots_base_joints[yname].GetJoint('j4').GetDOFIndex()])
                configspec = configspec + self.youbots_base_joints[yname].GetActiveConfigurationSpecification()
                goal = np.append(goal,basegoals[yname])
                goal = np.append(goal,armgoals[yname])
                initialconfig = np.append(initialconfig, self.youbots_base_joints[yname].GetActiveDOFValues())
            params = orpy.Planner.PlannerParameters()
            params.SetExtraParameters("<_fsteplength>0.15</_fsteplength>")
            params = orpy.Planner.PlannerParameters(params)
            params.SetExtraParameters("<_nmaxiterations>10000</_nmaxiterations>") # FIXME check if this takes effect
            params = orpy.Planner.PlannerParameters(params)
            params.SetConfigurationSpecification(self.env, configspec)
            params.SetInitialConfig(initialconfig)
            params.SetGoalConfig(goal)
            #IPython.embed()
            self.planner.InitPlan(None,params)
            traj = orpy.RaveCreateTrajectory(self.env,"")
            status = self.planner.PlanPath(traj)
            for yname in self.youbots:
                self.youbots_base_joints[yname].ReleaseAllGrabbed()
                self.youbots_base_joints[yname].SetTransform(self.awaypose) 
                self.youbots_base_joints[yname].SetDOFValues([0.]*self.youbots_base_joints[yname].GetDOF())
        finally:
            for yname in self.youbots:
                self.youbots[yname].SetTransform(initial_base_poses[yname])
                self.youbots[yname].SetDOFValues(initial_arm_configs[yname])
                if grabbed[yname] is not None:
                    for o in grabbed[yname]:
                        self.youbots[yname].Grab(o)
        IPython.embed()
        # TODO check status?
        # TODO convert traj to base and arm trajs?
        return status,base_trajs,arm_trajs

    def PlanSingleRobot(self,robot,basegoal,armgoal,baseonly=False,armonly=False):
        with self.env:
            if baseonly:
                robot.SetActiveDOFs([],orpy.DOFAffine.X|orpy.DOFAffine.Y|orpy.DOFAffine.RotationAxis,[0,0,1])
                goalconfig = np.array(basegoal) 
            elif armonly:
                robot.SetActiveDOFs(range(5))
                goalconfig = np.array(armgoal) 
            else:
                robot.SetActiveDOFs(range(5),orpy.DOFAffine.X|orpy.DOFAffine.Y|orpy.DOFAffine.RotationAxis,[0,0,1])
                goalconfig = np.array(armgoal) 
                goalconfig = np.append(goalconfig,basegoal)

            traj = self.planners[robot.GetName()].MoveActiveJoints(goal=goalconfig,maxiter=5000,steplength=0.15,maxtries=2,execute=False,outputtrajobj=True)
        #robot.SetActiveDOFs(range(5),orpy.DOFAffine.X|orpy.DOFAffine.Y|orpy.DOFAffine.RotationAxis,[0,0,1])
        #configspec = robot.GetActiveConfigurationSpecification()
        #goalconfig = np.array(armgoal) 
        #goalconfig = np.append(goalconfig,basegoal)
        #initialconfig = np.array(robot.GetActiveDOFValues())
        #params = orpy.Planner.PlannerParameters()
        #params.SetExtraParameters("<_fsteplength>0.15</_fsteplength>")
        #params = orpy.Planner.PlannerParameters(params)
        #params.SetExtraParameters("<_nmaxiterations>10000</_nmaxiterations>") # FIXME check if this takes effect
        #params = orpy.Planner.PlannerParameters(params)
        #params.SetConfigurationSpecification(self.env, configspec)
        #params.SetInitialConfig(initialconfig)
        #params.SetGoalConfig(goalconfig)
        #self.planner.InitPlan(None,params)
        #traj = orpy.RaveCreateTrajectory(self.env,"")
        #status = self.planner.PlanPath(traj)
        return traj 

    def PlanSequential(self,basegoals,armgoals):
        try:
            initial_base,initial_arm = self.SaveCurrentConfig()
            basetrajs = {}
            armtrajs = {}
            with self.env:
                for yname in self.youbots:
                    basetrajs[yname] = self.PlanSingleRobot(self.youbots[yname],basegoals[yname],armgoals[yname],baseonly=True)
                    if basetrajs[yname] is None:
                        print 'Failed to find plan for %s.'%(yname)
                        return None
                    self.youbots[yname].SetTransform(XYThetaToMatrix(basegoals[yname][0],basegoals[yname][1],basegoals[yname][2]))
                    armtrajs[yname] = self.PlanSingleRobot(self.youbots[yname],basegoals[yname],armgoals[yname],armonly=True)
                    if armtrajs[yname] is None:
                        print 'Failed to find plan for %s.'%(yname)
                        return None
                    self.youbots[yname].SetDOFValues(armgoals[yname],range(5))
                    #self.SetRobotConfig(self.youbots[yname],basegoals[yname],armgoals[yname])
            return basetrajs,armtrajs
        finally:
            self.LoadConfig(initial_base,initial_arm)

    def SaveCurrentConfig(self):
        initial_base_poses = {}
        initial_arm_configs = {}
        for yname in self.youbots:
            initial_base_poses[yname] = self.youbots[yname].GetTransform()
            initial_arm_configs[yname] = self.youbots[yname].GetDOFValues()
        return initial_base_poses,initial_arm_configs

    def LoadConfig(self,base_qs,arm_qs):
        for yname in self.youbots:
            self.youbots[yname].SetTransform(base_qs[yname])
            self.youbots[yname].SetDOFValues(arm_qs[yname])

    def SetRobotConfig(self,robot,baseq,armq):
        robot.SetTransform(XYThetaToMatrix(baseq[0],baseq[1],baseq[2]))
        robot.SetDOFValues(armq,range(5))

    def Execute(self,basetrajs,armtrajs):
        for yname in self.youbots:
            self.youbots[yname].GetController().SetPath(basetrajs[yname])
            while not self.youbots[yname].GetController().IsDone():
                time.sleep(0.01)
            self.youbots[yname].GetController().SetPath(armtrajs[yname])
            while not self.youbots[yname].GetController().IsDone():
                time.sleep(0.01)



