#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy

import openravepy as orpy
import IPython

class TfPlugin:

    def __init__(self, env, world_tf_id):
        self.env = env
        self.world_tf_id = world_tf_id

        self.tfplugin = orpy.RaveCreateSensor(self.env,'tfplugin tfplugin '+self.world_tf_id)
        env.Add(self.tfplugin)

    def RegisterBody(self, or_body, tf_id, openrave_frame_in_tf_frame=None, planar_tracking=False):
        if openrave_frame_in_tf_frame is None:
            self.tfplugin.SendCommand('RegisterBody ' + or_body.GetName() + ' ' + tf_id)
        else:
            x = openrave_frame_in_tf_frame[0,3]
            y = openrave_frame_in_tf_frame[1,3]
            z = openrave_frame_in_tf_frame[2,3]
            quat = orpy.quatFromRotationMatrix(openrave_frame_in_tf_frame[:3,:3])
            cmd = 'RegisterBody ' + or_body.GetName() + ' ' + tf_id + ' openrave_frame_in_tf_frame ' + str(x) + ' ' + str(y) + ' ' + str(z) + ' ' + str(quat[0]) + ' ' + str(quat[1]) + ' ' + str(quat[2]) + ' ' + str(quat[3])
            if planar_tracking:
                cmd = cmd + ' planar_tracking'
            self.tfplugin.SendCommand(cmd)

    def RegisterRobotHand(self, or_body, tf_id):
        self.tfplugin.SendCommand('RegisterRobotHand ' + or_body.GetName() + ' ' + tf_id)

    def UnregisterBody(self, or_body):
        self.tfplugin.SendCommand('UnregisterBody ' + or_body.GetName())

    def Pause(self):
        self.tfplugin.SendCommand('Pause')

    def Resume(self):
        self.tfplugin.SendCommand('Resume')

    def Clear(self):
        self.tfplugin.SendCommand('Clear')
