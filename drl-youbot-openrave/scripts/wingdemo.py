#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time

import IPython
ipshell = IPython.embed 

import openravepy as orpy
import youbotpy

np.set_printoptions(suppress=True)
np.set_printoptions(precision=4)

env,youbots,tf_plugin = youbotpy.init(sim=False,viewer=True,env_xml='environments/wingdemo.env.xml', \
        youbot_names=['drc1','drc3'])

tf_plugin.RegisterBody(env.GetKinBody('rack'),'rack')
tf_plugin.RegisterBody(env.GetKinBody('ladder'),'ladder')
tf_plugin.RegisterBody(env.GetKinBody('drc1_arm'),'drc1_arm')
tf_plugin.RegisterBody(env.GetKinBody('drc3_arm'),'drc3_arm')

ipshell()

