import time
import numpy as np
import openravepy as orpy

class WorldConfig(object):
    def __init__(self,env,world_config):
        self.new_wc = world_config
        self.env = env
        self.saved_wc = {}
    def __enter__(self):
        self.Save(self.new_wc['object_names'])
        if 'object_poses' in self.new_wc.keys():
            self.Load(self.new_wc)
    def __exit__(self, type, value, traceback):
        self.Load(self.saved_wc)
    def Save(self,object_names):
        self.saved_wc = {}
        self.saved_wc['object_names'] = object_names
        self.saved_wc['object_poses'] = {}
        for each in object_names:
            self.saved_wc['object_poses'][each] = self.env.GetKinBody(each).GetTransform()
    def Load(self,wc):
        for each in wc['object_names']:
            self.env.GetKinBody(each).SetTransform(wc['object_poses'][each])

class RobotsConfig(object):
    def __init__(self,env,base_configs,arm_configs,armindices):
        self.new_bc = base_configs
        self.new_ac = arm_configs
        self.armindices = armindices
        self.env = env
        self.saved_bc = {}
        self.saved_ac = {}
    def __enter__(self):
        self.Save()
        self.Load(self.new_bc,self.new_ac)
    def __exit__(self, type, value, traceback):
        self.Load(self.saved_bc,self.saved_ac)
    def Save(self):
        self.saved_bc = {}
        self.saved_ac = {}
        for each in self.new_bc:
            self.saved_bc[each] = self.env.GetRobot(each).GetTransform()
            self.saved_ac[each] = self.env.GetRobot(each).GetDOFValues(self.armindices)
    def Load(self,bc,ac):
        for each in bc:
            self.env.GetRobot(each).SetTransform(bc[each])
            self.env.GetRobot(each).SetDOFValues(ac[each],self.armindices)

class Grabbed(object):
    def __init__(self,env,grabbed):
        self.new_grabbed = grabbed
        self.env = env
        self.saved_grabbed = {}
    def __enter__(self):
        self.Save()
        self.Grab(self.new_grabbed)
    def __exit__(self, type, value, traceback):
        self.Restore()
    def Save(self):
        self.saved_grabbed = {}
        for each in self.new_grabbed:
            if self.env.GetRobot(self.new_grabbed[each]).IsGrabbing(self.env.GetKinBody(each)) is not None:
                self.saved_grabbed[each] = self.new_grabbed[each]
    def Grab(self,grabbed):
        for each in grabbed:
            self.env.GetRobot(grabbed[each]).Grab(self.env.GetKinBody(each))
    def Restore(self):
        for each in self.new_grabbed:
            self.env.GetRobot(self.new_grabbed[each]).Release(self.env.GetKinBody(each))
        self.Grab(self.saved_grabbed)


def visualizeTSR(env,params,yname,jgs,nsamples=20):
    # This iterates only for drc1
    for i in range(nsamples):
        basesample,armsample = jgs.GetGoal(env,params,yname)
        if basesample is None:
            print 'No valid sample.'
            return
        env.GetRobot(yname).GetController().Reset(0)
        pose = np.eye(4)
        pose[:3,:3] = orpy.rotationMatrixFromAxisAngle(basesample[2]*np.array([0.,0.,1.]))
        pose[:2,3] = basesample[:2]
        env.GetRobot(yname).SetTransform(pose)
        env.GetRobot(yname).SetDOFValues(armsample[:5], [0,1,2,3,4])
        #IPython.embed()
        time.sleep(0.01)

def GoalSearchTiming(env,params,youbots,grabdict,nruns=100):
    basegoals = {} 
    armgoals = {} 
    durations = []
    for i in range(nruns):
        starttime = time.time()
        basesamples,armsamples = jgs.GetFeasibleGoal(env,params,youbots,grabdict,timeout=100.0)
        durations.append(time.time() - starttime)
        if basesamples is not None:
            basegoals = basesamples
            armgoals = armsamples
        else:
            print 'TIMEOUT!'
            sys.exit()
    durations = np.array(durations)
    print durations
    print 'Mean time: ',durations.mean()


def XYThetaToMatrix(x,y,theta):
    pose = np.eye(4) 
    pose[:3,:3] = orpy.rotationMatrixFromAxisAngle(theta*np.array([0.,0.,1.]))
    pose[0,3] = x
    pose[1,3] = y
    return pose

def MatrixToXYTheta(mat):
    xytheta = np.array([0.,0.,0.])
    xytheta[:2] = mat[:2,3]
    xytheta[2] = np.arctan2(mat[1,0],mat[0,0])
    return xytheta

def VisualizeAssemblyOperation(env,assembly,assembly_pose,start_config,goal_config):
    with RobotsConfig(env,start_config['base'],start_config['arm'],range(5)):
        raw_input('Hit enter to visualize goal.')
    with WorldConfig(env,{'object_names':assembly.object_names}):
        with RobotsConfig(env,goal_config['base'],goal_config['arm'],range(5)):
            raw_input('Hit enter to visualize goal.')

def VisualizeGrasp(env,robot_name,grasp,obj_name):
    robot = env.GetRobot(robot_name)
    robotinworld = robot.GetTransform()
    worldinrobot = np.linalg.inv(robotinworld)
    eeinworld = robot.GetManipulators()[0].GetEndEffectorTransform()
    eeinrobot = np.dot(worldinrobot,eeinworld)
    robotinee = np.linalg.inv(eeinrobot)
    eeinw = np.dot(env.GetKinBody(obj_name).GetTransform(),grasp)
    robotinw = np.dot(eeinw,robotinee)
    robot.SetTransform(robotinw)

def VisualizeAssemblyGrasp(env,grasps,part_responsibilities):
    for each in grasps:
        VisualizeGrasp(env,part_responsibilities[each],grasps[each],each)

def MakeRobotTransparentExceptHand(env,robot,hand_link_names=['link5','leftgripper','rightgripper']):
    links = robot.GetLinks()
    for l in links:
        if l.GetName() not in hand_link_names:
            l.SetVisible(False)

