from openravepy import *
import numpy
env = Environment() # create openrave environment
env.SetViewer('qtcoin') # attach viewer (optional)

robot = env.ReadRobotXMLFile('../models/robots/kuka-youbot.robot.xml') #loads the robot

env.Add(robot,True) # add the robot to the environment

# this binds the youbot controller to the robot
probotcontroller = RaveCreateController(env,'youbotcontroller')
robot.SetController(probotcontroller)

#Once the robot is loaded run roscore in the terminal and run rosbag play [bagfile]
