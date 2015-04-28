#!/usr/bin/env python 
# -*- coding: utf-8 -*-

import os
import numpy as np
import time
import random

import openravepy as orpy
from tfplugin import TfPlugin

class YoubotEnv:

    def __init__(self, sim=False,viewer=False,env_xml=None,youbot_names=[],registered_objects=[],use_tfplugin=True, youbot_model='kuka-youbot.robot.xml'):
        self.youbotpydir = os.popen('rospack find youbotpy').read()
        print self.youbotpydir
        
        #RaveSetDebugLevel(0) # suppresses printing of non fatal errors
        
        env = orpy.Environment()
        if viewer:
            env.SetViewer('qtcoin')
        if env_xml:
            env.Load(env_xml)
        
        youbots = {}
        youbot_hands = {}
        for youbot_name in youbot_names:
            youbot = env.ReadRobotURI(self.youbotpydir[:-1] + '/../models/robots/' + youbot_model)
            #youbot = env.ReadRobotURI(self.youbotpydir[:-1] + '/../models/robots/kuka-youbot-hires.robot.xml')
            youbot.SetName(youbot_name)
            env.Add(youbot,True)
            youbots[youbot_name] = youbot

        self.basemanips = {}
        for name in youbot_names:
            self.basemanips[name] = orpy.interfaces.BaseManipulation(youbots[name],plannername='BiRRT')

        tfplugin=None
        if not sim:
            if use_tfplugin:
                tfplugin = TfPlugin(env,'map')
            for youbot_name in youbot_names:
                probotcontroller = orpy.RaveCreateController(env,'youbotcontroller')
                youbots[youbot_name].SetController(probotcontroller)
                #tfplugin.RegisterBody(or_body=youbots[youbot_name],tf_id=youbot_name)
                if use_tfplugin:
                    openrave_frame_in_tf_frame = np.eye(4)
                    openrave_frame_in_tf_frame[2,3] = -0.12 # difference betweeen vicon origin and openrave kinbody origin.
                    tfplugin.RegisterBody(or_body=youbots[youbot_name],tf_id=youbot_name,openrave_frame_in_tf_frame=openrave_frame_in_tf_frame)
                    #tfplugin.RegisterBody(or_body=youbots[youbot_name],tf_id=youbot_name,openrave_frame_in_tf_frame=openrave_frame_in_tf_frame,planar_tracking=True)
                    #tfplugin.RegisterRobotHand(or_body=youbots[youbot_name],tf_id=youbot_name+'_arm')
                # Also add youbot hands that are tracked separately.
                detached_hands = True
                if detached_hands:
                    youbot_hand = env.ReadKinBodyURI(self.youbotpydir[:-1] + '/../models/objects/youbothand.kinbody.xml')
                    youbot_hand.SetName(youbot_name+'_arm')
                    env.Add(youbot_hand,True)
                    youbot_hand.Enable(False)
                    openrave_frame_in_tf_frame = np.eye(4)
                    openrave_frame_in_tf_frame[2,3] = 0.013 # difference betweeen vicon origin and openrave kinbody origin.
                    tfplugin.RegisterBody(or_body=youbot_hand,tf_id=youbot_hand.GetName(),openrave_frame_in_tf_frame=openrave_frame_in_tf_frame)
                    youbot_hands[youbot_name] = youbot_hand
            if use_tfplugin:
                if not (registered_objects is None):
                    for oname in registered_objects: 
                        tfplugin.RegisterBody(or_body=env.GetKinBody(oname),tf_id=oname)

        if sim and viewer:
            # nudge robots a little, b/c clicking on trimeshes overlapping perfectly causes openrave to crash.
            for name in youbots:
                randpose = np.eye(4)
                randpose[:2,3] = np.array([random.random(), random.random()])
                youbots[name].SetTransform(randpose)

        self.env = env
        self.tfplugin = tfplugin
        self.sim =sim
        self.youbots = youbots
        self.youbot_hands = youbot_hands
        self.arm_joint_offsets = np.array([2.96, 1.70, -0.55, 0.80, 4.58])


    def PlanArmTo(self,robot,goal,planner=None):
        if planner is None:
            planner = self.basemanips[robot.GetName()]
        with self.env:
            robot.SetActiveDOFs(range(5))
            traj = planner.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.15,maxtries=2,outputtrajobj=True)
            #print traj.serialize().replace("\\n","\n")
        while not robot.GetController().IsDone():
            time.sleep(0.01)
        return traj
    

    def PlanBaseTo(self,robot,goal,planner=None):
        if planner is None:
            planner = self.basemanips[robot.GetName()]
        xyyaw = np.array([goal[0,3],goal[1,3],np.arctan2(goal[1,0],goal[0,0])])
        return self.PlanBaseToXYYaw(robot,xyyaw,planner)

    
    def PlanBaseToXYYaw(self,robot,xyyaw,planner):
        if planner is None:
            planner = self.basemanips[robot.GetName()]
        current = robot.GetTransform()
        currentxyyaw = np.array([current[0,3],current[1,3],np.arctan2(current[1,0],current[0,0])])
        if np.linalg.norm(currentxyyaw-xyyaw) < 0.0001:
            print robot.GetName(),' already at goal. Not moving.'
            return 
        with self.env:
            robot.SetActiveDOFs([],orpy.DOFAffine.X|orpy.DOFAffine.Y|orpy.DOFAffine.RotationAxis,[0,0,1])
            traj = planner.MoveActiveJoints(goal=xyyaw,maxiter=5000,steplength=0.15,maxtries=2,execute=False,outputtrajobj=True)
            robot.GetController().SetPath(traj)
        while not robot.GetController().IsDone():
            time.sleep(0.01)
        return traj


    def GetRealStartPoses(self,youbot_names,objects):
        # Register with tfplugin
        idealcontrollers = {}
        if self.tfplugin is None:
            self.tfplugin = TfPlugin(self.env,'map')
        for youbot_name in youbot_names:
            idealcontrollers[youbot_name] = self.youbots[youbot_name].GetController()
            probotcontroller = orpy.RaveCreateController(self.env,'youbotcontroller')
            self.youbots[youbot_name].SetController(probotcontroller)
            self.tfplugin.RegisterRobotHand(or_body=self.youbots[youbot_name],tf_id=youbot_name+'_arm')
        for oname in objects: 
            self.tfplugin.RegisterBody(or_body=self.env.GetKinBody(oname),tf_id=oname)
        #raw_input('Hit enter if start poses are updated from vicon.')
        # Unregister with tfplugin
        time.sleep(3.0)
        for youbot_name in youbot_names:
            self.tfplugin.UnregisterBody(or_body=self.youbots[youbot_name])
            # set back to idealcontroller
            self.youbots[youbot_name].SetController(idealcontrollers[youbot_name])
        for oname in objects: 
            self.tfplugin.UnregisterBody(or_body=self.env.GetKinBody(oname))

    def GetRobot(self,name):
        return self.youbots[name]

    def MoveArm(self,name,joint_values):
        robot = self.youbots[name]
        if self.sim:
            robot.SetDOFValues(joint_values, robot.GetActiveManipulator().GetArmIndices())
        else:
            # send to the real robot through ros 
            assert len(robot.GetActiveManipulator().GetArmIndices()) == len(joint_values),"joint values must equal number of arms"
            joint_values = list(joint_values) # In case it is a numpy array, convert to list.
            command = str(joint_values)[1:-1].replace(' ','')
            return robot.GetController().SendCommand("MoveArm "+ command, True)
            # brics_actuator/JointPositions /drcX/arm_1/arm_controller/position_command

    def Grab(self, name, part, move_gripper=True):
        if move_gripper:
            self.MoveGripper(name, 0.0, 0.0)
        self.youbots[name].Grab(part)
        
    def Release(self, name, part, move_gripper=True):
        if move_gripper:
            self.MoveGripper(name, 0.01, 0.01)
        self.youbots[name].Release(part)

    # TODO: testing
    def MoveBase(self,name,pose):
        #Needs testing
        robot = self.youbots[name]
        if self.sim:  
            robot.SetTransform(pose)  
        else:
            command = str(pose)[1:-1].replace(' ','')
            return robot.GetController().SendCommand("MoveBase "+ command, False) == 'True'
            # This is a service!!: assembly_common/BasePose /drcX/robot_base_command
       

    def MoveGripper(self,name,left, right):
        robot = self.youbots[name]
        if self.sim:
          robot.SetDOFValues([left, right], [5,6])          
          #self.youbots[name].SetDOFValues(joint_values)
        else:
           robot.GetController().SendCommand("MoveGripper "+str(left)+"," +str(right), False) == 'True'
           time.sleep(3.0)
           return
           # brics_actuator/JointPositions /drcX/arm_1/gripper_controller/position_command

    def ExecuteTrajectory(self,name,traj,start_index=0,end_index=None):
        print "Warning: youbotpy::ExecuteTrajectory ignores base positions for now."
        if end_index is None:
            end_index = traj.GetNumWaypoints()-1

        for ind in range(start_index, end_index+1):
            q=traj.GetWaypoint(ind)[:5]
            p=traj.GetWaypoint(ind)[5:8]
            # FIXME  Use p to set the base pose
            self.MoveArm(name,q)
            #if not self.sim:
            #    while True:
            #        self.youbots[name].GetController().IsDone() 
            #        time.sleep(0.1)
            time.sleep(0.5)

    def MoveHandStraight(self,name,direction,distance,step_size=0.005):
        if step_size <= 0.00001 or distance <= 0.00001:
            return
        eepose = self.youbots[name].GetManipulators()[0].GetEndEffectorTransform()
        dist_moved = 0.0
        while dist_moved < distance:
            new_ee = eepose.copy()
            dist_moved = dist_moved + step_size
            new_ee[:3,3] = eepose[:3,3] + direction*dist_moved
            #h = orpy.misc.DrawAxes(youbotenv.env,new_ee,0.25,2.0)
            with self.youbots[name]:
                arm_configs = self.yik.FindIKSolutions(self.youbots[name], new_ee)
            if arm_configs is None or len(arm_configs) == 0:
                print 'Warning: No ik found. Moved %f m.'%(dist_moved)
                break
            close_arm_config = None
            for ac in arm_configs:
                if np.linalg.norm(ac-self.youbots[name].GetDOFValues()[:5]) < 0.2:
                    close_arm_config = ac
                    break
            if close_arm_config is None:
                print 'Warning: No ik found. Moved %f m.'%(dist_moved)
                break
            self.MoveArm(name,close_arm_config)
            time.sleep(0.2)

    def PauseController(self,name):
        if self.sim:
            print 'Ignoring PauseController in sim mode.'
        else:
            robot = self.youbots[name]
            return robot.GetController().SendCommand("Pause") == 'True'

    def ResumeController(self,name):
        if self.sim:
            print 'Ignoring ResumeController in sim mode.'
        else:
            robot = self.youbots[name]
            return robot.GetController().SendCommand("Resume") == 'True'

    # This function returns the vertical and horizontal forces at the end-effector using the
    # J1,J2,J3 torques and the pseudo-inverse of the jacobian transpose.
    def GetEndEffectorForces(self,name):
        robot = self.youbots[name]
        torques = -1.*robot.GetDOFTorqueLimits()[1:4]
        # TODO remove torques due to links' mass.
        with self.env:
            with robot: 
                robot.SetDOFValues([0.],[0]) # Set j0 to zero to align arm with x-z plane.
                robot.SetTransform(np.eye(4)) # set robot orientation to align arm with x-z plane.
                j = robot.GetManipulators()[0].CalculateJacobian()
                jt = j.transpose()[1:4,[0,2]] # only get the X and Z component for j1,j2,j3
                try:
                    jtinv = np.linalg.pinv(jt)
                except np.linalg.linalg.LinAlgError:
                    print("Warning: Pseudo-inverse of Jacobian-transpose failed.")  
                    forces = np.array([0.,0.])
                else:
                    forces = np.dot(jtinv,torques)
                return forces

    def ReplaceWithHiResYoubot(self,name):
        youbot_hires = self.env.ReadRobotURI(self.youbotpydir[:-1] + '/../models/robots/kuka-youbot-hires.robot.xml')
        youbot_hires.SetName(name+'_hires')
        self.env.Add(youbot_hires,True)
        robot = self.youbots[name]
        youbot_hires.SetTransform(robot.GetTransform())
        youbot_hires.SetDOFValues(robot.GetDOFValues())
        robot.Enable(False)
        robot.SetVisible(False)
        
    def ReplaceWithHiResYoubots(self):
        for k in self.youbots.keys():
            self.ReplaceWithHiResYoubot(k)

       
class simpletestsuit:
    def __init__(self):
        self.ye =YoubotEnv(False, True, None, ["kuka"])
    
    def testGripper(self):
        return self.ye.MoveGripper("kuka", 0.4,1)
    def testArm(self):
        return self.ye.MoveArm("kuka",[1,2,3,4,5])

#test1 = simpletestsuit()
#print test1.testArm()
#print test1.testGripper()
