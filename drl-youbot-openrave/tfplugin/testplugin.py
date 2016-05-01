#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('lib/tfplugin')
env=Environment()
env.Load('environments/sundvik.env.xml')
env.SetViewer('qtcoin')

tfplugin = env.GetSensor('tfplugin')
tfplugin.SendCommand("RegisterBody SundvikChairBack brown_sundvik_back")
tfplugin.SendCommand("RegisterBody SundvikChairRightSide brown_sundvik_rightside")



