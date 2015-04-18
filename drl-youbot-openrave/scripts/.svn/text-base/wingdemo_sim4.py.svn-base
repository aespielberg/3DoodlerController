#!/usr/bin/env python
# -*- coding: utf-8 -*-

# TODO su andaki tek asm planning kismini csp olarak yaz
# TODO boyle yazinca heuristicin etkisi ne kadar bak, etkisi yoksa nregraspi value secim heuristigi olarak kullan
# TODO simdilik initial graspi eger ortak grasp varsa ortaklardan biri olmaya zorlayarak, yosa tumuyle serbest birakarak imp et. (yani robot-robot conflicti yuzunden regrasp olmayacagini assue ederek)
# TODO bunun ustune multi-step'i (baslangicta iki stepi) csp-optimization olarak formule edebilir misin bak. csp/optimizasyon oku kitaptan.
# ***
# TODO plan regrasping actions
# TODO 1-regraspli guzel ornek cikar..
# TODO notes'daki gibi csp formulasyonu olayina bak, regrasp'i da csp olarak formule edebilir miyiz dusun. multi-step olur mu dusun.. paperi yaz.
#**
# TODO csp bi oku, formulasyon olarak belki kafa acici bi seyler vardir, belki de direk correspond eden bi sey hemen buluruz.. bi sey cikmazsa, onceden dusundugum gibi n-regraspi onceden sadece env ile tesbit edip, implement etmek..
# TODO csp problemlerini oku. sadece goal'da sonuc bulmak problemini csp olarak ifade edebilir miyiz dependencylerle (isgal edilen volume uzerinden) birlikte objeler/grasplar arasinda? regrasping icin benzer bi sey yapabilir miyiz? multistep planning icin yapabilir miyiz?  
# TODO identify regrasp using env, and then gradually include others (yada her graspin ait oldugu set, sonra ararken sadece hem start hem goalda olanlarla baslayip, sonra yoksa sadece goalda ve sadece start'ta olanlardan iki tane alan seklinde, regrasp sayisini kendisi cikaran bi sey yapabilir miyiz?  
# TODO generate a few example videos
# TODO focus on multistep if all goes well
# TODO paperi yaz eger multisteple beraber bi butunlugu olursa.. (yada min occupied volume intersection .. ile de paper yazilabilir, eger rob-rob onemliyse gercekten..)
#***
# TODO regrasp identifier checks (we have two now: env and robt-robt, but make rob-rob for n-robots, this may also bring cool theory) 
# TODO new assembly configurations where rob-rob regrasping constraints may matter?
# TODO start writing (o makaleyi oku yazmakla ilgili olan)
# TODO going over 0 regrasps takes too long. can we find a way to quickly detect if 0 regrasps won't work? can we do somehting else about searching for higher number of regrasps? why is this so costly? (if I were to present this, how would I explain and justify that this is a problem that is intrinsically costly, and so we should look at non-complete solutions)? --> identify regrasp objects using only environment. we can also check for 
# TODO heuristic score'un min'i aliniyo su anda. onu nasil yapmak gerekir dusun..
# TODO figure out why motion planning crashes somtimes even with 0 regrasps
# TODO think about the heuristic.. orientationi da icine katacak sekilde daha robust/principled bi heuristic cikar. why sorusuna guzel cevap ver.. function mapping ee pose to occluded volume?
# TODO combined grasp planning'de robotlar/objeler birbirlerinden cok uzak iseler teker teker check yapabilmek. (bunu coll checker zaten yapiyo mu? gercekten fark eder mi?)
# ***
# zamanlama nasil? (20saniye wing demoda ilk grasplar free iken.)
# TODO ilk grasplari zorlastir wing demo icin: camera ve pini bi table'in ustune koy (sonra esasen regraspingin isin icine girecegi bir ornek dusun)
# TODO svn'e commit et bunlari
# TODO regraspingi ayni sampling icinde beraber represent edip, biondan bi ondan sample etcek sekilde, bunu da o benim yapmak istedigim optimizasyonu kendisi yapcak sekilde bi algoritma olabilir mi? (aslinda paralel run etme bunu direk yapmiyo mu?) tam degil sanirim, eger gercekten olasiliklari represent edebilirsem.. bi de regrasping de kolay olmayabilir, etraf kalabalik, tumuyle free ortam bulmak zor olabilir..
# TODO bir de genel olarak bu problem nasil assumptionlar yapsam cozmesi kolay hale gelir dusuniyim. eger oyle bi sey bulabilirsem ordan bile paper cikabilir.

# ***

# TODO
#      Once biraz daha dusun:
#         1. regrasping alg multi-step assemblyde usefule olur mu. oluyosa bunu yapmaya calis wafr icin, buna odaklan.
#             - fikir extend edilebilir gibi. yine de herseyi tam clear dusunmedim. ama multi step bi sey dusunulebilir. assemblydeki order freedomini kullanmak da enteresan olabilir. bunun efektif oldugu ornekler cikarmak (aslinda cok regrasp gerektirmeyecen ama bulmanin da cok kolay olmadigi) lazim.
#             - kendi multistep alg'imi daha clear yazmaya caliyim bi ornekle. assembly freedomi da katarak belki.
#             - eger graspxgrasp lookup table kullanilirsa (belli bi regrasp pozisyon/geometrisi icin) o zaman regrasp her zaman daha kolay karli olabilir: basta ve sonda calisan grasp bul. biri digerinde de calisiyosa regrasp yapma, calismiyosa regrasp ile bagla. (tlp paperi da boyle bisey. belli bi regrasp noktasi assume edebiliriz, orda da hesaplayabiliriz calisan regrasplari diyo).

import numpy as np
import time
import IPython
import yaml
import sys
import random
import cPickle as pickle

import openravepy as orpy

import youbotpy
from youbotpy import youbotik as yik
from tsr import GraspTSR,LookAtTSR
from JointGoalSearch import JointGoalSearch
from wingdemo_misc import XYThetaToMatrix,MatrixToXYTheta
from AssemblyConfigurationPlanner import AssemblyConfigurationPlanner
import MultiYoubotPlanner

#from wingdemoenv import *
from chairenv import *

times = []
configs = []

#for i in range(30):
youbotenv = youbotpy.YoubotEnv(sim=True,viewer=True,env_xml=envfile, \
                               youbot_names=robots)
env = youbotenv.env
youbots = youbotenv.youbots
for name in youbots:
    youbots[name].SetTransform(robot_start_poses[name])
    youbotenv.MoveGripper(name,0.01,0.01) # open grippers

#IPython.embed()

asm_planner = AssemblyConfigurationPlanner(env, youbots) 
starttime = time.time()
start_config,goal_config = asm_planner.PlanCSP(assembly, assembly_pose, part_responsibilities, task_regions, timeout=3.0)
#start_config,goal_config = asm_planner.Plan(assembly, assembly_pose, part_responsibilities, task_regions, timeout=300.0)
#start_config,goal_config = asm_planner.PlanWithRegrasp(assembly, assembly_pose, part_responsibilities, task_regions, timeout=300.0)
graspplanningtime = time.time()-starttime
print 'assembly config search took: ',graspplanningtime,' sec'
times.append(graspplanningtime)
configs.append((start_config,goal_config))
if start_config is None:
    print 'assembly config plan failed.'
else:
    print 'assembly configs found! Now planning motion.'

#IPython.embed()

# Following is just motion planning and playback.
for name in start_config['base']:
    start_config['base'][name] = MatrixToXYTheta(start_config['base'][name])
for name in goal_config['base']:
    goal_config['base'][name] = MatrixToXYTheta(goal_config['base'][name])
motionplanner = MultiYoubotPlanner.MultiYoubotPlanner(env,youbots)
starttime = time.time()
basetrajs1,armtrajs1 = motionplanner.Plan(start_config['base'],start_config['arm'])
print 'planning motion to go to start pose took: ',time.time()-starttime,' sec'
motionplanner.Execute(basetrajs1,armtrajs1)
for obj in assembly.object_names:
    youbots[part_responsibilities[obj]].Grab(env.GetKinBody(obj))
starttime = time.time()
basetrajs2,armtrajs2 = motionplanner.Plan(goal_config['base'],goal_config['arm'])
print 'planning motion to go to assembly pose took: ',time.time()-starttime,' sec'
motionplanner.Execute(basetrajs2,armtrajs2)
 
raw_input('Hit enter to destroy environment.')

youbotenv.env.Destroy()

def pickle_this(fname, envfile, start_config, goal_config, basetrajs1, armtrajs1, basetrajs2, armtrajs2, assembly, assembly_pose, robots, part_responsibilities, robot_start_poses):
    f = open(fname,'wb')
    pickle.dump(envfile,f)
    pickle.dump(start_config,f)
    pickle.dump(goal_config,f)
    for y in basetrajs1:
        basetrajs1[y] = basetrajs1[y].serialize()
        armtrajs1[y] = armtrajs1[y].serialize()
    for y in basetrajs2:
        basetrajs2[y] = basetrajs2[y].serialize()
        armtrajs2[y] = armtrajs2[y].serialize()
    pickle.dump(basetrajs1,f)
    pickle.dump(armtrajs1,f)
    pickle.dump(basetrajs2,f)
    pickle.dump(armtrajs2,f)
    pickle.dump(assembly,f)
    pickle.dump(assembly_pose,f)
    pickle.dump(robots,f)
    pickle.dump(part_responsibilities,f)
    pickle.dump(robot_start_poses,f)
    f.close()

IPython.embed()


