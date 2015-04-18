import numpy as np
import openravepy as orpy
import time


class Team:

    def __init__(self, robots):
        self.robots = robots

    def PlanGrasp(self, part, goalpose):
        # FIXME for now, works with single robot teams
        if len(self.robots) > 1:
            raise Exception('Team grasp planning works only for single-robot teams for now.')

    def Grasp(self, grasp, part):
        pass
        # TODO get grasping robots and grasp locations on object
        # TODO compute ik? with base?
        # TODO move robots there 
        # TODO move arms there
        # TODO grasp

    def MoveGraspedPartTo(self, pose):
        pass
        # TODO


