#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('velocity_control_ros')
import rospy
import numpy as np
import time
import sys
import pickle
import random
import copy
import IPython
from itertools import izip,product,combinations,chain
import datetime
import euclid

import openravepy as orpy
import youbotpy
from youbotpy import youbotik as yik
import GraspGenerator
import AnytimePlanner
import CollisionConstraints
import copy
import tfplugin
from brics_actuator.msg import JointVelocities, JointValue
from geometry_msgs.msg import Vector3 as Vector3_g
from std_msgs.msg import Bool
import sys
from assembly_common.srv import BasePose,ArmCommand
import tf.transformations as tr

from vicon_utils import *



BACKUP_AMT = 0.05
LIFT_AMT = 0.3
MOVE_AMT = 0.1
MAX_VEL = 0.1
THRESH = 0.001
BIG_THRESH = 0.004
BAD_ITERS = 10
SPEED_FACTOR = 0.25 #empirical guesstimate at transforming joint to cartesian speed
CLAMP_VALUE = 0.0
pos_acc = 0.005 #ideally 0.001
ang_acc = 0.05 #ideally 0.02

kd = 0.
ki = 0.
kp = 1.
#ki = 1.6
#kd = 0.0002 #Been finding this really bad, maybe get rid of it
ki = 0.5
#kd = 0.125





sim = False
use_vicon_for_start_poses = True

arm_names = []
for i in range(5):
    arm_names.append('arm_joint_' + str(i + 1))
    
unit = 's^-1 rad'

all_robot_names = ['drc1']
r = all_robot_names[0]
envfile = 'environments/floor.env.xml'

youbotenv = youbotpy.YoubotEnv(sim=sim,viewer=True,env_xml=envfile, \
                               youbot_names=all_robot_names, registered_objects=None, youbot_model='kuka-youbot-doodler.robot.xml')

env = youbotenv.env
youbots = youbotenv.youbots
if sim:
    for name in youbots:
        youbotenv.MoveGripper(name,0.01,0.01) # open grippers

yik.init(youbots[youbots.keys()[0]]) #   does not matter which robot
                                     # we use to initialize since 
                                     # all youbots have same kinematics.
grasp_generator = GraspGenerator.GraspGenerator(env)

offset = np.array([2.950, 1.1345, -2.5482, 1.7890, 2.9234])


robot = youbots['drc1']
manip = robot.GetManipulators()[0]
"""
print 'make ikmodel'
ikmodel = orpy.databases.inversekinematics.InverseKinematicsModel(robot,iktype=orpy.IkParameterization.Type.TranslationZAxisAngle4D)
print 'load it or autogenerate it'
if not ikmodel.load():
    ikmodel.autogenerate()
"""




robot_base_homes = {}
for r in all_robot_names:
    robot_base_homes[r] = youbots[r].GetTransform()
    
planners = {}
for r in all_robot_names:
    planners[r] = orpy.interfaces.BaseManipulation(youbots[r],plannername='BiRRT')





is_ready = False


def ready_callback(msg):
    is_ready = msg.data

rospy.init_node('velocity_controller')
feedback_service = '/drc1/arm_feedback_command'
arm_feedback = rospy.ServiceProxy(feedback_service, BasePose)
pub = rospy.Publisher('/' + all_robot_names[0] + '/arm_1/arm_controller/velocity_command', JointVelocities, queue_size=1)
pos_pub = rospy.Publisher('/end_effector_pose', Vector3_g, queue_size=1)
pos_pub_goal = rospy.Publisher('/end_effector_goal', Vector3_g, queue_size=1)
sub = rospy.Subscriber('ready', Bool, ready_callback)

extrude_pub_fast = rospy.Publisher('fast', Bool, queue_size=1)
extrude_pub_slow = rospy.Publisher('slow', Bool, queue_size=1)

def GetClosestArm(cur_arm, sol):
    best_dist = sys.maxint
    ret_i = 0
    for (i, v) in enumerate(sol):
        dist = np.linalg.norm(v - cur_arm, np.inf )
        if dist < best_dist:
            best_dist = dist
            ret_i = i
    #IPython.embed()
    return sol[ret_i]
    
def MoveBaseTo(robot,xyyaw,planner,skip_waypoints=False):
    try:
        orpy.RaveSetDebugLevel(orpy.DebugLevel.Verbose)
        robot.GetEnv().GetKinBody('floor').Enable(False)
        current = robot.GetTransform()
        currentxyyaw = np.array([current[0,3],current[1,3],np.arctan2(current[1,0],current[0,0])])
        if np.linalg.norm(currentxyyaw-xyyaw) < 0.0001:
            print robot.GetName(),' already at goal. Not moving.'
            return 
        with env:
            robot.SetActiveDOFs([],orpy.DOFAffine.X|orpy.DOFAffine.Y|orpy.DOFAffine.RotationAxis,[0,0,1])
            traj = planner.MoveActiveJoints(goal=xyyaw,maxiter=5000,steplength=0.15,maxtries=2,execute=False,outputtrajobj=True, jitter=-0.01)
            if skip_waypoints:
                traj.Remove(1,traj.GetNumWaypoints()-1)
            robot.GetController().SetPath(traj)
        while not robot.GetController().IsDone():
            time.sleep(0.01)
    except Exception, e:
        print str(e)
        c = raw_input('IPython shell?(y/n)')
        if c == 'y':
            IPython.embed()
        raise e
    finally:
        robot.GetEnv().GetKinBody('floor').Enable(True)
        orpy.RaveSetDebugLevel(orpy.DebugLevel.Info)

def MoveArmTo(robot,goal,planner):
    try:
        orpy.RaveSetDebugLevel(orpy.DebugLevel.Verbose)
        with env:
            robot.SetActiveDOFs(range(5))
            traj = planner.MoveActiveJoints(goal=goal,maxiter=5000,steplength=0.001,maxtries=2,execute=False,outputtrajobj=True, jitter=-0.00025)
            robot.GetController().SetPath(traj)
        while not robot.GetController().IsDone():
            time.sleep(0.01)
    finally:
        orpy.RaveSetDebugLevel(orpy.DebugLevel.Info)
        
def get_relative_pose(mat):
    pose = PoseStamped()
    pose.header.frame_id = '/map'
    pose.pose.position.x = mat[0, 3]
    pose.pose.position.y = mat[1, 3]
    pose.pose.position.z = mat[2, 3]
    tr_pose = transform_by_subjects(pose, '/' + all_robot_names[0])
    mat2 = copy.deepcopy(mat)
    mat2[0, 3] = tr_pose.pose.position.x
    mat2[1, 3] = tr_pose.pose.position.y
    mat2[2, 3] = tr_pose.pose.position.z
    return mat2
    
def get_global_pose(mat):
    pose = PoseStamped()
    pose.header.frame_id = '/' + all_robot_names[0]
    pose.pose.position.x = mat[0, 3]
    pose.pose.position.y = mat[1, 3]
    pose.pose.position.z = mat[2, 3]
    tr_pose = transform_by_subjects(pose, '/map')
    mat2 = copy.deepcopy(mat)
    mat2[0, 3] = tr_pose.pose.position.x
    mat2[1, 3] = tr_pose.pose.position.y
    mat2[2, 3] = tr_pose.pose.position.z
    return mat2
        
def getEndEffector():
    return youbots[r].GetManipulators()[0].GetEndEffectorTransform()
    
def createVelocity(vels):
    for i in range(len(vels)):
        vels[i] = float(vels[i])
    vels = np.array(vels)
    #clamp
    if np.linalg.norm(vels, 2) > MAX_VEL:
        vels /= np.linalg.norm(vels, 2)
        vels *=  MAX_VEL
    
    v = JointVelocities()
    for i in range(5):
        v.velocities.append(JointValue())
        v.velocities[i].joint_uri = arm_names[i]
        v.velocities[i].unit = unit
        v.velocities[i].value = float(vels[i])
    return v
    
def stop():
    v = createVelocity([0, 0, 0, 0, 0])
    pub.publish(v)

#start_config = np.array([2.9499957785497077, 1.334502240891718, -1.2181996753192461, 1.789004272867827, 2.9234068314087893]) - offset #outright
start_config = np.array([2.9499957785497077, 1.44502240891718, -1.2181996753192461, 2.6, 2.9234068314087893]) - offset #angled
#start_config = np.array([2.9499957785497077, 1.24502240891718, -1.2181996753192461, 3.2, 2.9234068314087893]) - offset #upright
#start_config = np.array([2.9499957785497077, 0.84502240891718, -0.8181996753192461, 2.6, 2.9234068314087893]) - offset #angled different

print start_config
time.sleep(3.0)
robot = youbots[r]
MoveArmTo(robot,start_config,planners[r])
print 'init done'
default_z = getEndEffector()[3, 2]


#Testing
"""
target=ikmodel.manip.GetTransform()[0:3,3]
#direction = np.random.rand(3)-0.5

parameterization = orpy.IkParameterization()
parameterization.SetTranslationZAxisAngle4D(target, -np.pi/3)
IPython.embed()
sol = manip.FindIKSolutions(parameterization, orpy.IkFilterOptions.CheckEnvCollisions)
MoveArmTo(robot, sol, planners[r])
"""
#sol = manip.FindIKSolutions(orpy.IkParameterization().SetTranslationYAxisAngle4D(target,0.0), orpy.IkFilterOptions.CheckEnvCollisions)


#add a small cylinder to the environment
def addGeometryToEnvironment(endpoint1, endpoint2):
    #TODO: How do we add in the doodler?
    radius = 0.0005
    with env:
        prism = orpy.RaveCreateKinBody(env, '')
        prism.SetName(str(np.random.rand())) #give it a random name - we'll never reference it
        points_diff = (endpoint2 - endpoint1)[:-1, 3] #difference vector
        offset = endpoint1[:-1, 3]
        norm = np.linalg.norm(points_diff, 2)
        prism.InitFromBoxes(np.array([[0., 0., 0., radius*2., radius*2., norm]]), True)
        z = np.array([0., 0., norm])
        cross = np.cross(z, points_diff)
        angle = np.arccos(np.dot(z, points_diff) / (norm * norm) )
        tr_matrix = tr.rotation_matrix(angle, cross)
        tr_matrix[:-1, 3] = endpoint1[:-1, 3]
        env.Add(prism, True)
        prism.SetTransform(tr_matrix)
        

#FOR TESTING
def moveArmWrapper():
    pose = get_relative_pose(getEndEffector())
    #pose[0, 3] += 0.01
    arm = yik.FindIKSolutions(robot, get_global_pose(pose))
    print arm
    MoveArmTo(robot, arm, planners[r])
    
    


def startExtruding(fast):
    stopExtruding() #first, stop both fast and slow commands
     #then start the appropriate command
    if fast:
        extrude_pub_fast.publish(Bool(data=True))
    else:
        extrude_pub_slow.publish(Bool(data=True))

    
def stopExtruding():
    extrude_pub_fast.publish(Bool(data=False))
    extrude_pub_slow.publish(Bool(data=False))
    rospy.sleep(2.0)

def waitForReady():
    while not is_ready:
        pass #spin
        

    
    
        
def arm_feedback_wrapper(xx, yy, theta, pos_acc, ang_acc, base_offset, frame):
    print "arm_feedback(%f, %f, %f, %f, %f, %s, %s)" % (xx, yy, theta, pos_acc, ang_acc, base_offset, frame)
    success = False
    while not success:
        success = True
        try:
            arm_feedback(xx, yy, theta, pos_acc, ang_acc, base_offset, frame)
        except Exception:
            success = False
            print "Arm feedback threw an exception. Trying again..."
            rospy.sleep(0.5)
            
def move(xx, yy, theta):
    pose = PoseStamped()
    pose.header.frame_id = '/drc1_arm'
    pose.pose.position.x = -xx
    pose.pose.position.y = -yy
    quat = tr.quaternion_from_euler(0., 0., -theta, axes='sxyz')
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    
    tr_pose = transform_by_subjects(pose, '/map')
    
    target_euler = tr.euler_from_quaternion(np.array([tr_pose.pose.orientation.x, tr_pose.pose.orientation.y, tr_pose.pose.orientation.z, tr_pose.pose.orientation.w]), axes='sxyz')
    
    target_x = tr_pose.pose.position.x
    target_y = tr_pose.pose.position.y
    target_theta = target_euler[2]
    
    
    arm_feedback_wrapper(target_x, target_y, target_theta - np.pi, pos_acc, ang_acc, False, '/map')
    
    
def projectPoint(p, line, point):
    #projects p onto the line in direction line protruding from point
    num = np.dot(p - point, line) #remove the offset from p, then add it back at the end
    dem = np.dot(line, line)
    return num / dem * line + point
    
    

def MoveStraight(velocity_factor, rel_diff, horiz=True):
    """
    Moves the end effector in a straight line.
    Velocity_factor - how fast to move the joints
    rel_diff - how far to move the end effector relative to its current location
    """
    velocity_factor *= SPEED_FACTOR
    
    #transform is start
    transform = getEndEffector()
    
    #For debugging with rqt_plot:
    

    step = robot.GetTransform()[:-1, :-1].dot(rel_diff)
    """
    target_x = transform[0, 3] + rel_diff[0]
    target_y = transform[1, 3] + rel_diff[1]
    target_z = transform[2, 3] + rel_diff[2] #Drc1's base frame
    """
    
    #target is goal
    target = copy.deepcopy(transform)
    target[:-1, 3] += step
    """
    target[0, 3] = target_x
    target[1, 3] = target_y
    target[2, 3] = target_z
    """
    
    
    
    #line is between start and finish
    
    line = target[:3, 3] - transform[:3, 3]
    

    #initialize:
    diff = np.array([0., 0., 0., 0., 0.])
    integ = np.array([0., 0., 0., 0., 0.])
    itera = 0
    
    best_distance = sys.maxint
    
    
    #waitForReady()
    startExtruding(fast=False)
    rospy.sleep(2.0)
    loop_count = 0
    while np.linalg.norm(getEndEffector()[:-1, 3] - target[:-1, 3], 2) > THRESH:
        
        timestamp = time.time()
        
        
        
        cart_dist = np.linalg.norm(getEndEffector()[:-1, 3] - target[:-1, 3], 2)
        if cart_dist < best_distance:
            best_distance = cart_dist
        elif cart_dist < BIG_THRESH:
            itera += 1
        
        
        """
        print 'end effector:'
        print getEndEffector()
        print 'target:'
        print target
        """
        print 'dist is: '
        print np.linalg.norm(getEndEffector()[:-1, 3] - target[:-1, 3], 2)
        
        
        current_arm = robot.GetDOFValues()[0:5]
        
        sub_target = copy.deepcopy(getEndEffector()) #in drc1's frame
        
        #0.03 for horizontal
        #0.01 for vertical
        if horiz:
            horizon = 0.01
            #horizon = 0.001
        else:
            horizon = 0.01

        pos_pub.publish(Vector3_g(x=sub_target[0, 3], y=sub_target[1, 3], z=sub_target[2, 3]))
        
        
        
        


        
        #project our current location onto the target line to get the direction we need to go.
        sub_target[:3, 3] = projectPoint(sub_target[:-1, 3], line, transform[:-1, 3]) #TODO: wrap this function
        #sub_target[:3, :3] = transform[:3, :3]
        
        direc = (target[:3, 3] - sub_target[:3, 3])
        direc_norm = np.linalg.norm(direc, 2)
        
        sub_target[:3, 3] += direc * horizon / direc_norm #in drc1's frame
        
        pos_pub_goal.publish(Vector3_g(x=sub_target[0, 3], y=sub_target[1, 3], z=sub_target[2, 3]))

        
        
        print sub_target
        sol = yik.FindIKSolutions(robot, sub_target) #convert it back to the global frame and find IK solutions in global frame
        if not sol:
            break
            print "COULD NOT REACH TARGET"
        closest_arm = GetClosestArm(current_arm, sol)
        
        """
        MoveArmTo(robot,closest_arm,planners[r])
        rospy.sleep(0.01)
        continue
        """
        
        old_error = diff
        
        
        
        
        #P-Term in PID controller
        
        diff = closest_arm - current_arm
        
        
        
        
        
        if itera > BAD_ITERS: #if we're getting worse than the best for BAD_ITERS consecutive cycles and not a million miles away
            break
        
        
        dt = time.time() - timestamp
        print 'dt'
        print dt
        
        #I-Term in PID controller
        integ += diff * dt
        
        #D-Term in PID controller
        
        
        
        deriv = (diff - old_error) / dt

        vel = kp * diff + ki * integ + kd * deriv
        
        
        #TESTING - keep in plane
        vel[0] = 0.
        vel[4] = 0.
        
        
        norm=np.linalg.norm(vel)
        print 'norm is ' + str(norm)
        if norm != 0:
            vel /= norm
        
            
        for i in range(len(vel)):
            if abs(vel[i]) < CLAMP_VALUE:
                vel[i] = 0.0

    
        print diff
        #IPython.embed()
        
        loop_count += 1
        print loop_count
        vel_fac = np.min([velocity_factor, velocity_factor * loop_count / 200.])
        
        print 'vel fac is '
        print vel_fac
        
        v = createVelocity(vel*vel_fac)
        #IPython.embed()
        pub.publish(v)
        rospy.sleep(0.01)
        

        #MoveArmTo(robot, closest_arm, planners[r])

    stop()
    stopExtruding()
    print 'finished segment'
    

    addGeometryToEnvironment(transform, getEndEffector())
    
    rospy.sleep(0.5)
    
    
#MoveStraight(0.75, np.array([0.0, 0., 0.02]))

def point_from_string(s):
    return np.array([float(x) for x in s.split(' ')])

def file_to_commands(filename):
    idx = 0
    with open(filename) as f:
        for line in f:
            idx += 1
            if line == '-':
                #End of spline
                idx = 0
                continue
                
            point = point_from_string(line)
            if idx == 1:
                #move base to position
                #TODO: Raise arm
                MoveArmTo(robot, np.array([0., 0., 0., 0., 0.]), planners[r])
                
                #TODO: who cares about theta?
                current = robot.GetTransform()
                
                yaw = np.arctan2(current[1,0],current[0,0])
                
                
                MoveBaseTo(robot,np.array([point[0], point[1], yaw]),planners[r],skip_waypoints=False)
                
                #lower_arm
                MoveArmTo(robot,start_config,planners[r])
                
                #Finally, move the arm to the appropriate height
                
                current_pose = get_relative_pose(getEndEffector())
                current_pose[3, 2] += point[2]
                sols = yik.FindIKSolutions(robot, get_global_pose(current_pose))
                current_arm = robot.GetDOFValues()[0:5]
                sol = GetClosestArm(current_arm, sols)
                
                
                MoveArmTo(robot, sol, planners[r])
                
                
                
                
            else:
                #First, rotate to point
                point = point_from_string(line)
                robot_pos = robot.GetTransform()[0:2, -1]
                robot_dir = robot.GetTransform()[0:2, 0]
                robot_dir_self = [1., 0.]
                xy1 = robot_dir
                xy2 = point[0:-1] - prev_point[0:-1]
                angle = np.arccos( np.dot(xy1, xy2) / (np.linalg.norm(xy1, 2) * np.linalg.norm(xy2, 2)) )
                move(0., 0., angle)
                
                angle2 = np.arccos( np.dot(robot_dir_self, xy2) / (np.linalg.norm(robot_dir_self, 2) * np.linalg.norm(xy2, 2)) )
                
                #now we need to rotate this onto the xz plane
                rot_mat = tr.rotation_matrix(angle2, np.array([0., 0., 1.]))[:-1, :-1] #just get the rotation part
                
                xz_diff = rot_mat.dot(point - prev_point)
                
                print xz_diff
                
                xz_diff[1] = 0. #clamp away numerical error
                
                
                #And now move the arm by this amount
                
                MoveStraight(0.3, xz_diff)
                
                
                #Back up by the amount we moved and move the arm back into position
                dist = np.linalg.norm(point - prev_point, 2)
                
                
                
                move(-dist, 0., 0.) #finally, back up
                
                #Arm back into correct x position:
                
                current_pose = get_relative_pose(getEndEffector())
                current_pose[0, 3] += dist
                sols = yik.FindIKSolutions(robot, get_global_pose(current_pose))
                current_arm = robot.GetDOFValues()[0:5]
                sol = GetClosestArm(start_config, sols)
    
                MoveArmTo(robot, sol, planners[r])
                
                
                
            prev_point = point #bookkeeping


#move(0.0, 0., np.pi)

IPython.embed()
file_to_commands('output.txt')


#MoveStraight(0.3, np.array([0., 0., 0.02]), horiz=False)
#exit()
#move(-0.04, 0., 0.)

#MoveStraight(0.1, np.array([-0.008, 0.008, 0.0]))
#MoveStraight(0.1, np.array([0., 0., 0.01]))
#MoveStraight(0.9, np.array([0., 0., 0.02]))

#MoveStraight(0.3, np.array([-0.02, 0., 0.]), horiz=True)

#MoveStraight(0.3, np.array([-0.01, 0., 0.]), horiz=True)

MoveStraight(0.3, np.array([0., 0., 0.01]), horiz=False)

move(0., -0.04, 0.)

MoveStraight(0.3, np.array([0., 0., 0.01]), horiz=False)

move(0., 0., np.pi/32.)

MoveStraight(0.3, np.array([0., 0., 0.01]), horiz=False)

#MoveStraight(0.3, np.array([-0.01, 0., 0.01]), horiz=True)

#MoveStraight(0.5, np.array([-0.02, 0., -0.02]), horiz=True)


#MoveStraight(0.3, np.array([0., 0., 0.02]))
#MoveStraight(0.3, np.array([-0.02, 0., 0.0]))
#MoveStraight(0.1, np.array([-0.014, 0., 0.014]))
#MoveStraight(0.3, np.array([-0.02, 0., 0.]))

"""
MoveStraight(0.1, np.array([0., 0., 0.01]))
MoveStraight(0.1, np.array([0., 0., 0.01]))
MoveStraight(0.1, np.array([0., 0., 0.01]))
MoveStraight(0.1, np.array([0., 0., 0.01]))
MoveStraight(0.1, np.array([0., 0., 0.01]))
MoveStraight(0.1, np.array([0., 0., 0.01]))
MoveStraight(0.1, np.array([0., 0., 0.01]))
MoveStraight(0.1, np.array([0., 0., 0.01]))
MoveStraight(0.1, np.array([0., 0., 0.01]))
"""
IPython.embed()


#MoveStraight(0.1, np.array([-0.015, 0.0, 0.0]))
#MoveStraight(0.1, np.array([0., 0.015, 0.0]))
#MoveStraight(0.1, np.array([0.015, 0.0, 0.0]))
#MoveStraight(0.1, np.array([0., -0.015, 0.0]))

#MoveStraight(0.1, np.array([0.0, 0., 0.03]))

#MoveStraight(0.3, np.array([-0.02, -0.02, 0]))
#MoveStraight(speed, np.array([0., 0, dist]))
#MoveStraight(speed, np.array([-dist, 0., 0]))
stopExtruding()
#MoveStraight(speed, np.array([0., -dist, 0]))
#startExtruding(fast=True)
#MoveStraight(speed, np.array([0., dist, 0]))





"""
def MoveEEStraight(velocity_factor,target,step):
    #vel_factor = 1.5
    while np.linalg.norm([target[2, 3] - getEndEffector()[2, 3]], 2) > THRESH:
        print np.linalg.norm([target[2, 3] - getEndEffector()[2, 3]], 2)
        current_arm = robot.GetDOFValues()[0:5]
        sub_target_z = getEndEffector()[2, 3] + step
        sub_target = copy.deepcopy(target)
        sub_target[2, 3] = sub_target_z
        sol = yik.FindIKSolutions(robot, sub_target)
        closest_arm = GetClosestArm(current_arm, sol)
        diff = closest_arm - current_arm
        v = createVelocity(velocity_factor*diff)
        v.velocities[0].value = 0.0
        pub.publish(v)
        rospy.sleep(0.1)
    stop()
    
def MoveClecoDownVel(velocity_factor,length=0.08):
    transform = getEndEffector()
    target = copy.deepcopy(transform)
    target[2, 3] -= length
    MoveEEStraight(velocity_factor,target,-0.005)
    
MoveClecoDownVel(6.0,0.04)
"""


rospy.spin()

