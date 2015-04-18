#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import IPython
import random
import time
import copy
import operator
from itertools import izip,product,combinations
from operator import itemgetter

import openravepy as orpy

from youbotpy import youbotik as yik
from tsr import GraspTSR,LookAtTSR
from wingdemo_misc import XYThetaToMatrix,WorldConfig,RobotsConfig,Grabbed,VisualizeAssemblyOperation,MakeRobotTransparentExceptHand,VisualizeAssemblyGrasp
from scipy.spatial.distance import cdist,pdist
import AssemblyGraspCSP

class AssemblyConfigurationPlanner(object):

    def __init__(self,env,youbots):
        self.env = env
        self.youbots = youbots
        yik.init(youbots[youbots.keys()[0]]) # does not matter which robot we use to initialize since all youbots have same kinematics.
        self.iktimes = [] # TODO remove
        self.collchecktimes = [] # TODO remove
        self.ngraspstried = 0
        self.nconfigstried = 0
        self.asm_grasps = []
        self.asm_grasp_scores = []
        self.grasp_grid_translational_resolution = 0.05
        self.grasp_grid_rotational_resolution = np.pi/9.0

    def SampleGrasps(self,assembly,task_regions):
        grasps = {}
        for each in assembly.object_names:
            grasps[each] = task_regions[each].SampleObjectRelative(self.env)
        return grasps

    def ComputeGraspDistance(self,g1,g2):
        return np.sqrt(((g1-g2[:3,3])**2).sum())

    def ComputeAssemblyGraspScore(self,asm_grasp):
        asm_grasp = [g[:3,3] for g in asm_grasp]
        #m=pdist(asm_grasp,metric=self.ComputeGraspDistance) 
        m=pdist(asm_grasp,metric='euclidean') 
        return np.min(m)

    def ComputeGraspScores(self,assembly,task_regions):
        grasps = {}
        grasps_in_asm = {}
        #n_grasps_for_obj = {}
        for each in assembly.object_names:
            grasps[each] = []
            grasps_in_asm[each] = []
            #n_grasps_for_obj[each] = []
            grasps[each] = task_regions[each].UniformSamples(self.env, self.grasp_grid_translational_resolution,self.grasp_grid_rotational_resolution)
            for i in range(len(grasps[each])):
                grasps_in_asm[each].append(np.dot(assembly.object_relative_poses[each],grasps[each][i]))
            #for i in range(10):
            #    g = task_regions[each].SampleObjectRelative(self.env)
            #    #grasps[each].append(g)
            #    grasps[each].append(np.dot(assembly.object_relative_poses[each],g))
        #print grasps
        self.grasp_grid_translational_resolution /= 2.0
        self.grasp_grid_rotational_resolution /= 2.0
        asm_grasps = []
        asm_grasp_scores = []
        grasps_values = []
        grasps_in_asm_values = []
        for i in range(len(assembly.object_names)):
            grasps_values.append(grasps[assembly.object_names[i]])
            grasps_in_asm_values.append(grasps_in_asm[assembly.object_names[i]])
        for grasp,asm_grasp in izip(product(*grasps_values),product(*grasps_in_asm_values)):
            score = self.ComputeAssemblyGraspScore(asm_grasp)
            asm_grasps.append(grasp)
            asm_grasp_scores.append(score)
        # To use heuristic
        self.asm_grasp_scores,self.asm_grasps = map(list, zip(*[(s,g) for (s,g) in sorted(zip(asm_grasp_scores,asm_grasps),key=itemgetter(0))]))
        #IPython.embed()
        # To *not* use the heuristic
        #self.asm_grasps = asm_grasps
        #sc = [s for (s,g) in sorted(zip(asm_grasp_scores,asm_grasps),key=itemgetter(0))] # TODO comment out
        #IPython.embed() # TODO comment out

    def SampleGraspsHeuristic(self,assembly,task_regions):
        if len(self.asm_grasps) < 1:
            stt = time.time()
            self.ComputeGraspScores(assembly,task_regions)
            print 'computing grasp scores took %f sec.'%(time.time()-stt)
        grasps = {}
        for i in range(len(assembly.object_names)):
            grasps[assembly.object_names[i]] = self.asm_grasps[-1][i]
        del self.asm_grasps[-1] # remove this grasp from the list of grasps to be tried in the future.
        return grasps

    def CreateIndexedSpace(self,axes,element):
        if len(axes) == 0:
            return element 
        else:
            return [self.CreateIndexedSpace(axes[1:], element+(axes[0][i],)) for i in range(len(axes[0]))]

    # dims is a list of ints showing the size of each of the dimensions
    def GetMultiDimensionalIndex(self,dims,flat_index):
        reverse_dims = copy.copy(dims)
        reverse_dims.reverse()
        multidim_index = []
        for dim in reverse_dims:
            multidim_index.append(flat_index%dim)
            flat_index = flat_index/dim
        multidim_index.reverse()
        return multidim_index

    def GetItemWithIndex(self,multidim_list,ndims,indexes):
        if ndims == 0:
            return multidim_list
        else:
            return self.GetItemWithIndex(multidim_list[indexes[0]],ndims-1,indexes[1:])

    def ComputeRegraspSpace(self, n_grasps_per_object, object_indexes):
        product_list = []
        for ind in object_indexes:
            product_list.append(range(n_grasps_per_object[ind]))
        return product(*product_list)

    # Returns a list of tuples of two lists (indexes of grasps at start and goal)
    #      e.g. [([0,1],[0,1]),([0,1],[0,2])]
    def GetIndexesForAllGraspPlans(self,n_grasps_per_object,n_regrasps):
        grasp_plans = []
        flatind = 0
        n_objects = len(n_grasps_per_object)
        while flatind < reduce(operator.mul, n_grasps_per_object, 1): # while we do not exceed the number of items in the whole set of grasps
            index=self.GetMultiDimensionalIndex(n_grasps_per_object,flatind)
            flatind += 1
            for regrasp_objects in combinations(range(n_objects),n_regrasps):
                combined_space_of_regrasps = self.ComputeRegraspSpace(n_grasps_per_object,regrasp_objects)
                for regrasp in combined_space_of_regrasps:
                    other_grasp_index = copy.copy(index)
                    for obj,ind in izip(regrasp_objects,regrasp):
                        other_grasp_index[obj] = ind
                    grasp_plans.append((index,other_grasp_index))
        return grasp_plans

    def GetIndexesForMatchingGrasps(self,indexes,n_grasps_per_object,n_regrasps):
        matching_indexes_list = []
        n_objects = len(n_grasps_per_object)
        for regrasp_objects in combinations(range(n_objects),n_regrasps):
            combined_space_of_regrasps = self.ComputeRegraspSpace(n_grasps_per_object,regrasp_objects)
            for regrasp in combined_space_of_regrasps:
                other_grasp_index = copy.copy(indexes)
                for obj,ind in izip(regrasp_objects,regrasp):
                    other_grasp_index[obj] = ind
                matching_indexes_list.append(other_grasp_index)
        return matching_indexes_list

    # Returns a list of lists (indexes) of grasps.
    def SortGrasps(self,grasps,n_grasps_per_object,assembly):
        flatind = 0
        list_of_indexes = []
        list_of_scores = []
        while flatind < reduce(operator.mul, n_grasps_per_object, 1): # while we do not exceed the number of items in the whole set of grasps
            indexes=self.GetMultiDimensionalIndex(n_grasps_per_object,flatind)
            flatind += 1
            g = self.GetItemWithIndex(grasps,len(n_grasps_per_object),indexes)
            list_of_indexes.append(indexes)
            asm_g = []
            object_relative_poses_values = []
            for i in range(len(assembly.object_names)):
                object_relative_poses_values.append(assembly.object_relative_poses[assembly.object_names[i]])
            for objg,objpose in izip(g,object_relative_poses_values):
                asm_g.append(np.dot(objpose,objg))
            score = self.ComputeAssemblyGraspScore(asm_g)
            list_of_scores.append(score)
        sorted_scores_list,sorted_indexes_list = map(list, zip(*[(s,g) for (s,g) in sorted(zip(list_of_scores,list_of_indexes),key=itemgetter(0))]))
        sorted_indexes_list.reverse()
        return sorted_indexes_list

    def PlanWithRegrasp(self, assembly, assembly_pose, part_responsibilities, task_regions, timeout=30.0): 
        worlds = self.GetAssemblyWorlds(assembly, assembly_pose)
        # gridify grasps on each object
        grasps = []
        n_grasps_per_object = []
        for each in assembly.object_names:
            obj_grasp_samples = task_regions[each].UniformSamples(self.env, self.grasp_grid_translational_resolution,self.grasp_grid_rotational_resolution)
            grasps.append(obj_grasp_samples)
            print 'n_grasps_per_object: ', len(obj_grasp_samples) 
            n_grasps_per_object.append(len(obj_grasp_samples))
        # Update grid resolution for next step. # FIXME better to not do these here if we are not looping here. or maybe loop here.
        self.grasp_grid_translational_resolution /= 2.0
        self.grasp_grid_rotational_resolution /= 2.0
        time_before_creating_space = time.time()
        print 'Creating indexed space.'
        indexed_asm_grasps = self.CreateIndexedSpace(grasps,())
        print 'Creating indexed space took %f sec.'%(time.time()-time_before_creating_space)
        print 'Sorting grasps with heuristic.'
        time_before_sort = time.time()
        sorted_indexes_list = self.SortGrasps(indexed_asm_grasps,n_grasps_per_object,assembly)
        print 'Sorting grasps took %f sec.'%(time.time()-time_before_sort)
        max_regrasps = len(assembly.object_names)
        for i in range(max_regrasps+1):
            time_before = time.time()
            print 'Searching assembly grasps with %d regrasps.'%(i)
            for indexes in sorted_indexes_list: 
                goal_asm_grasp = self.GetItemWithIndex(indexed_asm_grasps,len(assembly.object_names),indexes)
                goal_asm_grasp_named = {}
                for j in range(len(assembly.object_names)):
                    goal_asm_grasp_named[assembly.object_names[j]] = goal_asm_grasp[j]
                goal_config = {}
                goal_config['base'],goal_config['arm'] = self.CheckConstraint(worlds[1],goal_asm_grasp_named,part_responsibilities)
                if goal_config['base'] is None:
                    continue
                matching_indexes_list = self.GetIndexesForMatchingGrasps(indexes,n_grasps_per_object,n_regrasps=i)
                for matching_indexes in matching_indexes_list:
                    start_asm_grasp = self.GetItemWithIndex(indexed_asm_grasps,len(assembly.object_names),matching_indexes)
                    start_asm_grasp_named = {}
                    for j in range(len(assembly.object_names)):
                        start_asm_grasp_named[assembly.object_names[j]] = start_asm_grasp[j]
                    start_config = {}
                    start_config['base'],start_config['arm'] = self.CheckConstraint(worlds[0],start_asm_grasp_named,part_responsibilities)
                    if start_config['base'] is not None:
                        return start_config,goal_config
            print 'Searching assembly grasps with %d regrasps took %f sec.'%(i,time.time()-time_before)
        return None,None

    def Plan(self, assembly, assembly_pose, part_responsibilities, task_regions, timeout=30.0): 
        # Need to compute the configurations the world will go through
        worlds = self.GetAssemblyWorlds(assembly, assembly_pose)
        # FIXME keep trying.. but until when? when do we decide to just grasp at start and regrasp in the middle? and how to structure this so we will be able to make changes to accomodate that and maybe future assembly steps..?
        starttime = time.time()
        while time.time()-starttime < timeout:
            # Sample grasp configurations for all objects
            grasps = self.SampleGraspsHeuristic(assembly,task_regions)
            ## TODO remove block starts
            #for each in grasps:
            #    MakeRobotTransparentExceptHand(self.env,self.env.GetRobot(part_responsibilities[each]))
            #assembly.Show(self.env,assembly_pose)
            #VisualizeAssemblyGrasp(self.env,grasps,part_responsibilities)
            #raw_input('hit enter')
            #continue
            ## TODO remove block ends
            #grasps = self.SampleGrasps()
            # Check if satisfies at the end
            #grasps_in_world = {}
            #for each in grasps:
            #    grasps_in_world[each] = np.dot(assembly_pose, grasps[each])
            goal_config = {}
            goal_config['base'],goal_config['arm'] = self.CheckConstraint(worlds[1],grasps,part_responsibilities)
            #if goal_config['base'] is not None:
            #    return start_config,goal_config
            if goal_config['base'] is None:
                #print 'failed to satisfy goal assembly config'
                continue
            # Check if satisfies at start
            start_config = {}
            start_config['base'],start_config['arm'] = self.CheckConstraint(worlds[0],grasps,part_responsibilities)
            #start_config['base'] = 'REMOVE'  # TODO remove
            #if start_config['base'] is None:
            #    continue
            if start_config['base'] is not None:
                return start_config,goal_config
            #else:
            #    print 'failed to satisfy start assembly config'
        return None,None

    def SampleBaseConfig(self,center_xy,radius=1.0):
        # Sample uniformly from a circle.
        xy = np.array([1.0,1.0])
        while np.linalg.norm(xy) >= 1.0:
            xy = np.array([random.uniform(-1.0,1.0), random.uniform(-1.0,1.0)])
        xy += center_xy
        theta = random.uniform(-np.pi,np.pi)
        return XYThetaToMatrix(xy[0],xy[1],theta)

    def IsConfigFeasible(self,base_configs,arm_configs,part_responsibilities=[]):
        with self.env:
            with RobotsConfig(self.env,base_configs,arm_configs,range(5)):
                #self.env.UpdatePublishedBodies() # TODO remove for timing
                #time.sleep(1.0)
                with Grabbed(self.env,part_responsibilities):
                    for each in base_configs:
                        stt=time.time() # TODO remove
                        is_valid = not self.env.CheckCollision(self.youbots[each])
                        self.collchecktimes.append(time.time()-stt) # TODO remove
                        if not is_valid:
                            return False
        return True

    def CheckConstraint2(self,world_config,grasps,part_responsibilities):
        self.ngraspstried += 1 # TODO remove
        with WorldConfig(self.env,world_config):
            base_configs={}
            arm_configs={}
            for part in part_responsibilities:
                robot_name = part_responsibilities[part]
                base_configs[robot_name] = self.SampleBaseConfig(self.env.GetKinBody(part).GetTransform()[:2,3])
                arm_configs[robot_name] = np.zeros(5)
            if not self.IsConfigFeasible(base_configs,arm_configs):
                print 'Bases collide!'
                return None,None
            self.nconfigstried += 1 # TODO remove
            with RobotsConfig(self.env,base_configs,arm_configs,range(5)):
                for each in grasps:
                    robot_name = part_responsibilities[each]
                    eepose = np.dot(self.env.GetKinBody(each).GetTransform(),grasps[each])
                    #eepose = grasps[each]
                    stt=time.time() # TODO remove
                    iksolns = yik.FindIKSolutions(self.youbots[robot_name],eepose,checkenvcollision=False)
                    self.iktimes.append(time.time()-stt) # TODO remove
                    if len(iksolns) < 1:  
                        print 'No ik soln'
                        return None,None 
                    else:
                        arm_configs[robot_name] = iksolns[random.randint(0,len(iksolns)-1)] 
            if not self.IsConfigFeasible(base_configs,arm_configs,part_responsibilities):
                print 'ik soln collide'
                return None,None
        return base_configs,arm_configs

    def CheckConstraint(self,world_config,grasps,part_responsibilities):
        with WorldConfig(self.env,world_config):
            base_configs = {}
            arm_configs = {}
            self.ngraspstried += 1 # TODO remove
            for each in grasps:
                robot_name = part_responsibilities[each]
                #print 'rob: ',robot_name
                eepose = np.dot(self.env.GetKinBody(each).GetTransform(),grasps[each])
                #h = orpy.misc.DrawAxes(self.env,eepose,0.25,2.0)
                #IPython.embed()
                #eepose = grasps[each]
                stt=time.time() # TODO remove
                base_configs[robot_name],arm_configs[robot_name] = self.SampleBaseAndArmGivenEEPose(self.youbots[robot_name],eepose)
                self.iktimes.append(time.time()-stt) # TODO remove
                if base_configs[robot_name] is None:
                    #print 'no base pose for ee pose.'
                    return None,None
            #print 'base pose found'
            self.nconfigstried += 1 # TODO remove
            if not self.IsConfigFeasible(base_configs,arm_configs,part_responsibilities):
                #print 'config not feasible'
                return None,None
        return base_configs,arm_configs

    # iterates over base poses for the same grasp
    def CheckConstraint3(self,world_config,grasps,part_responsibilities):
        with WorldConfig(self.env,world_config):
            base_configs = {}
            arm_configs = {}
            self.ngraspstried += 1 # TODO remove
            configs = {}
            for each in grasps:
                robot_name = part_responsibilities[each]
                #print 'rob: ',robot_name
                eepose = np.dot(self.env.GetKinBody(each).GetTransform(),grasps[each])
                #eepose = grasps[each]
                stt=time.time() # TODO remove
                base_configs[robot_name],arm_configs[robot_name] = self.SampleBaseAndArmGivenEEPose(self.youbots[robot_name],eepose,returnfirst=False,rotationresolution=0.5,translationresolution=0.15)
                self.iktimes.append(time.time()-stt) # TODO remove
                if base_configs[robot_name] is None < 1:
                    #print 'no base pose for ee pose.'
                    return None,None
                configs[robot_name] = []
                for i in range(len(base_configs[robot_name])):
                    fullconfig = {'base':base_configs[robot_name][i], 'arm':arm_configs[robot_name][i]}
                    configs[robot_name].append(fullconfig)
            for c in product(*configs.values()):# FIXME values() might be dangerous if order is important
                basec = {}
                armc = {}
                for i in range(len(grasps.keys())): # FIXME keys() might be dangerous if order is important
                    robot_name = part_responsibilities[grasps.keys()[i]]# FIXME keys() might be dangerous if order is important
                    basec[robot_name] = c[i]['base']
                    armc[robot_name] = c[i]['arm']
                self.nconfigstried += 1 # TODO remove
                #print 'basec: ',basec
                #print 'armc: ',armc
                if self.IsConfigFeasible(basec,armc,part_responsibilities):
                    return basec,armc
        return None,None

    def GetAssemblyWorlds(self,assembly,assembly_pose):
        worlds = []
        # FIXME For now only two. In the future, maybe more.
        start_world = {'object_names':assembly.object_names,'object_poses':{},'fixed_objects':assembly.fixed_objects}
        for each in assembly.object_names:
            start_world['object_poses'][each] = self.env.GetKinBody(each).GetTransform()
        worlds.append(start_world)
        goal_world = {'object_names':assembly.object_names,'object_poses':{},'fixed_objects':assembly.fixed_objects}
        for each in assembly.object_names:
            goal_world['object_poses'][each] = np.dot(assembly_pose,assembly.object_relative_poses[each])
        worlds.append(goal_world)
        return worlds

    def SampleBaseAndArmGivenEEPose(self, rob, eepose,returnfirst=True,rotationresolution=0.05,translationresolution=0.01):
        #start = time.time()
        #print 'starting ik generation'
        base_poses,arm_configs=yik.FindIKAndBaseSolutions(rob,eepose,returnfirst=returnfirst,checkenvcollision=False,randomize=True,rotationresolution=rotationresolution,translationresolution=translationresolution)
        #print "IK generation took %f sec."%(time.time()-start)
        if base_poses is None or len(base_poses) == 0:
            #print 'Failed to find ik.'
            return None,None
        if returnfirst:
            randindex = random.randint(0,len(base_poses)-1)
            base_pose = base_poses[randindex]
            arm_config = np.array(arm_configs[randindex])
            return base_pose,arm_config
        else:
            return base_poses,arm_configs

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
        tsr = GraspTSR(params[yname+'_goal_tsr']['T0_w_obj'],
                       params[yname+'_goal_tsr']['Tw_e_list'],
                       params[yname+'_goal_tsr']['Bw_list'],
                       params[yname+'_goal_tsr']['grab_obj'])
        with env:
            env.GetKinBody('wingskin').SetTransform(wingskin_goal)
            s = tsr.Sample(env)
        return s

    ######################################
    # Functions after CSP implementation #
    ######################################

    def ConvertGraspsToWorldEEPoses(self,obj_relative_grasps,object_name,assembly,assembly_pose):
        world_ee_poses = []
        for grasp in obj_relative_grasps:
            world_ee_poses.append(np.dot(assembly_pose,np.dot(assembly.object_relative_poses[object_name],grasp)))
        return world_ee_poses

    def GetRobotGraspConfigsGivenEEPoses(self,robot_name,object_name,ee_poses):
        robot_grasp_configs = []
        for ee_pose in ee_poses:
            base_configs,arm_configs = self.SampleBaseAndArmGivenEEPose(self.env.GetRobot(robot_name),ee_pose,returnfirst=False,rotationresolution=0.5,translationresolution=0.2)
            if base_configs is not None:
                for bc,ac in izip(base_configs,arm_configs):
                    robot_grasp_configs.append(AssemblyGraspCSP.RobotGraspConfig(robot_name,object_name,bc,ac))
        return robot_grasp_configs

    def RemoveInconsistentRobotConfigs(self,robot_name,robot_configs,constraint):
        new_robot_configs = []
        print robot_name,': starts with ',len(robot_configs),' configs'
        for q in robot_configs:
            if constraint.IsConsistentUnary(robot_name,q):
                new_robot_configs.append(q)
        print robot_name,': returns with ',len(new_robot_configs),' configs'
        return new_robot_configs

    def PlanCSP(self, assembly, assembly_pose, part_responsibilities, task_regions, timeout=30.0): 
        worlds = self.GetAssemblyWorlds(assembly, assembly_pose)
        starttime = time.time()
        constraints = AssemblyGraspCSP.AssemblyConstraints(self.env,worlds[1]) # FIXME only for assembly pose for now.
        #while time.time()-starttime < timeout:
        for i in range(1):
            robot_configs = {}
            robot_names = []
            for each in assembly.object_names:
                robot_name = part_responsibilities[each]
                robot_names.append(robot_name)
                obj_relative_grasps = task_regions[each].UniformSamples(self.env, self.grasp_grid_translational_resolution,self.grasp_grid_rotational_resolution)
                world_relative_ee_poses = self.ConvertGraspsToWorldEEPoses(obj_relative_grasps,each,assembly,assembly_pose)
                all_robot_configs = self.GetRobotGraspConfigsGivenEEPoses(robot_name,each,world_relative_ee_poses)
                robot_configs[robot_name] = self.RemoveInconsistentRobotConfigs(robot_name,all_robot_configs,constraints)
            self.grasp_grid_translational_resolution /= 2.0
            self.grasp_grid_rotational_resolution /= 2.0
            result = AssemblyGraspCSP.BacktrackingSearch(robot_names,robot_configs,constraints)
            print 'took {0} sec'.format(time.time()-starttime)
            if result is not None:
                assembly.Show(self.env,assembly_pose)
                for robot_name,robot_config in result:
                    self.env.GetRobot(robot_name).SetTransform(robot_config.base_config)
                    self.env.GetRobot(robot_name).SetDOFValues(robot_config.arm_config,range(5))
                IPython.embed()
                return None # FIXME
        return None



