#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import IPython
import yaml
import sys
import random

import openravepy as orpy
import youbotpy
import youbotik as yik

from tsr import GraspTSR,LookAtTSR

#np.set_printoptions(suppress=True)
#np.set_printoptions(precision=4)

awaypose = np.eye(4)
awaypose[:2,3] = np.array([100.0, 100.0])

# TODO 
#      x!! lookattsr'i da bitir, ekle. (simdilik grasp olarak impl ettim)
#      sorular:
#           X* ne kadar hizli bulunuyo sonuc bak. ilginc bi sey var mi?
#              - yok gibi. simdilik hizli buabiliyorum gibi. daha sonra hem delik olayini gercekci yaptigimda hem de chair icin yaptigimda tekrar denerim, ama bulunacak gibi cok inanilmaz zor bi problem degilse. 
#           * teker teker planda feasible sonuclarin hepsi ulasilabilir mi? zaman farki var mi arada? 
#              * robotlarin planlama sirasini degistirince zaman farki var mi?
#              * joint planning'e gecis yapilabilir mi? buyuk obje tasiyan robot once gecsin tarzi.. joint planning zaten bunu bulmaz mi direk..
#           * joint planlama birbirlerinin icinden geciyo...?
#           * ilk bastaki grasp goal'da secilenle ayni degil. hallet.
#           * genel bi framework olarak yaz, ki degisik seyler icin de planlayabil.  
#           * iki robotla tasimayi nasil halledebilirim..?
#           * gercek robot implementasyonu..
#           * secilen grasp'a gidis.. 
#      genel direction:
#           (X)* planlamayi yaz, joint planlama.
#                - threerobot yaratarak yaptim planlamayi
#           * degisik constraintleri hazirla objelerle
#           * objelerin degisik grasp olayini yeni objelerle ve chairle de yap
#           * robotlarda implement et
#           * sorun cikarsa guzel bi cozumu var mi bak
#             - synchronization isini mesele etsem.. synchronizationi hesaba katmaya calisan planning diye...
#           * cozulurse hizla devam etmeye calis.
#      diger isler
#          - openrave planning birbirlerinin icinden geciyo.. neden?
#          - camera ve fastener ekle
#          - lookattsr impl et
#          - drc2,3 icin yeni lookattsr'lar ekle yaml'a
#          - yeni deliklerin pozisyonlarini koy
#          - chair parcalari icin de implement et
#  
#      daha sonra bunu bi de chair icin yap mesela. genellestirmeye calis ki, gercekten general sistemin requirementlari ne olacak anlayalim.

secondgoal = False
justplanning = False

def SampleBaseAndArmGivenEEPose(rob, eepose):
    #start = time.time()
    base_poses,arm_configs=yik.FindIKAndBaseSolutions(rob,eepose,returnfirst=True,checkenvcollision=False,randomize=True)
    #print "IK generation took %f sec."%(time.time()-start)
    if base_poses is None or len(base_poses) == 0:
        print 'Failed to find ik.'
        return None
    randindex = random.randint(0,len(base_poses)-1)
    base_pose = base_poses[randindex]
    arm_config = np.array(arm_configs[randindex])
    sample = np.array([0.,0.,0.,0.,0.,0.,0.,0.])
    sample[:5] = arm_config
    sample[5:7] = base_pose[:2,3]
    sample[7] = np.arctan2(base_pose[1,0],base_pose[0,0])
    return sample

def GetGoal(env,params, yname):
    with env:
        goal = None
        for i in range(1000):
           s = GetTSRSample(env,params,yname)
           #print 'yname: ',yname
           #print 's: ',s
           #h = orpy.misc.DrawAxes(env,s,0.25,2.0)
           #raw_input('enter')
           goal = SampleBaseAndArmGivenEEPose(env.GetRobot(yname),s)
           if goal is not None:
               return np.array(goal)
        return None

def GetTSRSample(env, params, yname):
    ladderpose = ladder.GetTransform()
    wingskin_goal = np.dot(ladderpose, params['wingskin_in_ladder']) 
    if secondgoal:
        tsr = GraspTSR(env, 
                       params[yname+'_goal_tsr2']['T0_w_obj'],
                       params[yname+'_goal_tsr2']['Tw_e_list'],
                       params[yname+'_goal_tsr2']['Bw_list'],
                       params[yname+'_goal_tsr2']['grab_obj'])
    else:
        tsr = GraspTSR(env, 
                       params[yname+'_goal_tsr']['T0_w_obj'],
                       params[yname+'_goal_tsr']['Tw_e_list'],
                       params[yname+'_goal_tsr']['Bw_list'],
                       params[yname+'_goal_tsr']['grab_obj'])
    with env:
        env.GetKinBody('wingskin').SetTransform(wingskin_goal)
        s = tsr.Sample()
    return s

def GetHardGoal(params, yname):
    goal = params[yname+'_goal_arm_config']
    base_goal = [np.array(params[yname+'_goal_base_pose'])[0,3], 
                 np.array(params[yname+'_goal_base_pose'])[1,3], 
                 orpy.axisAngleFromRotationMatrix(np.array(params[yname+'_goal_base_pose'])[:3,:3])[2]]
    goal.extend(base_goal)
    return goal

def visualizeTSR(env,params,yname,nsamples=20):
    # This iterates only for drc1
    for i in range(nsamples):
        sample = GetGoal(env,params,yname)
        if sample is None:
            print 'No valid sample.'
            return
        env.GetRobot(yname).GetController().Reset(0)
        pose = np.eye(4)
        pose[:3,:3] = orpy.rotationMatrixFromAxisAngle(sample[7]*np.array([0.,0.,1.]))
        pose[:2,3] = sample[5:7]
        env.GetRobot(yname).SetTransform(pose)
        env.GetRobot(yname).SetDOFValues(sample[:5], [0,1,2,3,4])
        #IPython.embed()
        time.sleep(0.01)


env,youbots,tf_plugin = youbotpy.init(sim=True,viewer=True,env_xml='environments/wingdemo.env.xml', \
        youbot_names=['drc1','drc2','drc3'])

drc1 = youbots['drc1']
drc2 = youbots['drc2']
drc3 = youbots['drc3']
for y in youbots:
    with youbots[y].GetEnv():
        manip = youbots[y].GetManipulators()[0]
        youbots[y].SetActiveManipulator(manip)

yik.init(drc1) # does not matter which robot we use to initialize since all youbots have same kinematics.


ladder = env.GetKinBody('ladder')
rack = env.GetKinBody('rack')
wingskin = env.GetKinBody('wingskin')

params = yaml.load(open('./wingdemo_sim_parameters.yaml'))
drc1.SetTransform(np.array(params['drc1_start_pose']))
drc2.SetTransform(np.array(params['drc2_start_pose']))
drc3.SetTransform(np.array(params['drc3_start_pose']))
wingskin.SetTransform(np.array(params['wingskin_start_pose']))
ladder.SetTransform(params['ladder_pose'])
rack.SetTransform(params['rack_pose'])


if not justplanning:
    #raw_input('Hit enter to visualize goals for drc1')
    #visualizeTSR(env,params,'drc1',nsamples=20)
    #drc1.SetTransform(np.array(params['drc1_start_pose']))
    #raw_input('Hit enter to visualize goals for drc2')
    #visualizeTSR(env,params,'drc2',nsamples=20)
    #drc2.SetTransform(np.array(params['drc2_start_pose']))
    #raw_input('Hit enter to visualize goals for drc3')
    #visualizeTSR(env,params,'drc3',nsamples=20)
    #drc3.SetTransform(np.array(params['drc3_start_pose']))
    
    #IPython.embed()
    raw_input('Hit enter to find feasible joint goal.')
    
    # This tests for all robots
    goals = {} 
    wingskin_goal = np.dot(ladder.GetTransform(), params['wingskin_in_ladder']) 
    wingskin.SetTransform(wingskin_goal)
    starttime = time.time()
    for i in range(1000):
        with env:
            samples = {}
            feasible = True
            for y in youbots:
               sample = GetGoal(env,params,y)
               if sample is None:
                   feasible = False
                   print 'no goal for %s'%(y)
                   break
               pose = np.eye(4)
               pose[:3,:3] = orpy.rotationMatrixFromAxisAngle(sample[7]*np.array([0.,0.,1.]))
               pose[:2,3] = sample[5:7]
               youbots[y].SetTransform(pose)
               youbots[y].SetDOFValues(sample[:5],[0,1,2,3,4])
               samples[y] = sample

            env.UpdatePublishedBodies()

            if not feasible:
                break
    
            try:
               drc1.Grab(wingskin)
               for y in youbots:
                   collision = env.CheckCollision(youbots[y])
                   if collision:
                       feasible = False
                       #time.sleep(0.1) 
                       break
            finally:
                drc1.Release(wingskin)
    
        if feasible:
            print 'Total time: %f'%(time.time()-starttime)
            inp = raw_input('Found a feasible goal! Find another (a), plan (p), or quit (any other character)?')
            if inp is 'a': 
                continue
            elif inp is 'p':
                goals = samples
                break
            else:
                sys.exit()


if len(goals) == 0:
    print 'No goal found'
    IPython.embed()

# Planning
params = yaml.load(open('./wingdemo_sim_parameters.yaml'))
drc1.SetTransform(np.array(params['drc1_start_pose']))
drc2.SetTransform(np.array(params['drc2_start_pose']))
drc3.SetTransform(np.array(params['drc3_start_pose']))
wingskin.SetTransform(np.array(params['wingskin_start_pose']))
ladder.SetTransform(params['ladder_pose'])
rack.SetTransform(params['rack_pose'])

drc1.SetDOFValues(np.array([ 0.  ,  0.7 ,  1.5 , -0.6 ,  1.57, -0.  ,  0.  ]))
drc1.Grab(wingskin)

#raw_input('Hit enter for planning.')

# load the ThreeYoubots.robot.xml
threeyoubot = env.ReadRobotURI('robots/ThreeYoubots.robot.xml')
threeyoubot.SetName('threeyoubot')
env.Add(threeyoubot,True)

# set subyoubots to where the current youbots are.
threeyoubot_start_q = np.zeros(3*8) # excluding the gripper joints
indices = []
joffset = 0
threeyoubot_start_q[joffset:joffset+2] = drc1.GetTransform()[:2,3]
threeyoubot_start_q[joffset+2] = np.arctan2(drc1.GetTransform()[1,0],drc1.GetTransform()[0,0])
threeyoubot_start_q[joffset+3:joffset+8] = drc1.GetDOFValues()[:5] 
indices.extend(range(joffset,joffset+8))
joffset = 8
threeyoubot_start_q[joffset:joffset+2] = drc2.GetTransform()[:2,3]
threeyoubot_start_q[joffset+2] = np.arctan2(drc2.GetTransform()[1,0],drc2.GetTransform()[0,0])
threeyoubot_start_q[joffset+3:joffset+8] = drc2.GetDOFValues()[:5] 
indices.extend(range(joffset+2,joffset+8+2))
joffset = 16
threeyoubot_start_q[joffset:joffset+2] = drc3.GetTransform()[:2,3]
threeyoubot_start_q[joffset+2] = np.arctan2(drc3.GetTransform()[1,0],drc3.GetTransform()[0,0])
threeyoubot_start_q[joffset+3:joffset+8] = drc3.GetDOFValues()[:5] 
indices.extend(range(joffset+4,joffset+8+4))


threeyoubot.SetDOFValues(threeyoubot_start_q,indices)

with env:
    drc1.Release(wingskin)
    threeyoubot.Grab(wingskin)
drc1.SetTransform(awaypose)
drc2.SetTransform(awaypose)
drc3.SetTransform(awaypose)

raw_input('Enter to plan.')
threeyoubot_goal_q = np.zeros(3*8)
indices = []
joffset = 0
threeyoubot_goal_q[joffset:joffset+2] = goals['drc1'][5:7]
threeyoubot_goal_q[joffset+2] = goals['drc1'][7]
threeyoubot_goal_q[joffset+3:joffset+8] = goals['drc1'][0:5]
indices.extend(range(joffset,joffset+8))
joffset = 8
threeyoubot_goal_q[joffset:joffset+2] = goals['drc2'][5:7]
threeyoubot_goal_q[joffset+2] = goals['drc2'][7]
threeyoubot_goal_q[joffset+3:joffset+8] = goals['drc2'][0:5]
indices.extend(range(joffset+2,joffset+8+2))
joffset = 16
threeyoubot_goal_q[joffset:joffset+2] = goals['drc3'][5:7]
threeyoubot_goal_q[joffset+2] = goals['drc3'][7]
threeyoubot_goal_q[joffset+3:joffset+8] = goals['drc3'][0:5]
indices.extend(range(joffset+4,joffset+8+4))

start_time = time.time()
threeyoubot_planners = orpy.interfaces.BaseManipulation(threeyoubot,plannername='BiRRT')
with env:
    threeyoubot.SetActiveDOFs(indices)
    traj = threeyoubot_planners.MoveActiveJoints(goal=threeyoubot_goal_q,maxiter=10000,steplength=0.15,maxtries=2)
    print 'Planning took %f sec.'%(time.time()-start_time)
while not threeyoubot.GetController().IsDone():
    time.sleep(0.01)


#planners = {}
#for y in youbots:
#    planners[y] = orpy.interfaces.BaseManipulation(youbots[y],plannername='BiRRT')
#
#for y in youbots: # TODO permute this randomly?
#    print 'youbot: ',y
#    with env:
#        youbots[y].SetActiveDOFs(range(5),orpy.DOFAffine.X|orpy.DOFAffine.Y|orpy.DOFAffine.RotationAxis,[0,0,1])
#        if justplanning:
#            goal = GetHardGoal(params,y)
#        else:
#            goal = goals[y]
#        planners[y].MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.15,maxtries=2)
#    while not youbots[y].GetController().IsDone():
#        time.sleep(0.01)

# TODO start wingskin at a different pose, then look at the difference between the orders of plans.
# TODO joint planning for a given goal -> is it also fast like sequential planning?
# TODO goal search -> how long does it take. do every feasible goal work for the sequential/joint planner.

# TODO youbotik el tam yana bakarken bi fail ediyo. neden? youbotikde mi sorun yoksa ikfast'te mi? tam yana degilm cok alcak oldugunda mi yoksa..

# Goal
#drc1.SetTransform(np.array(params['drc1_goal_base_pose']))
#drc2.SetTransform(np.array(params['drc2_goal_base_pose']))
#drc3.SetTransform(np.array(params['drc3_goal_base_pose']))
#drc1.SetDOFValues(params['drc1_goal_arm_config'], range(len(params['drc1_goal_arm_config'])))
#drc2.SetDOFValues(params['drc2_goal_arm_config'], range(len(params['drc2_goal_arm_config'])))
#drc3.SetDOFValues(params['drc3_goal_arm_config'], range(len(params['drc3_goal_arm_config'])))

IPython.embed()


