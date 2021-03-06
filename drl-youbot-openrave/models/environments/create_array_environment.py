#!/usr/bin/env python
# -*- coding: utf-8 -*-


import random
import os

num_elems = 3
size = 0.2
os.remove('random_array.env.xml')
f = open('random_array.env.xml','a')

f.write('<Environment>\n')
f.write('<camtrans>1.418 -1.234 2.963</camtrans>\n')
f.write('<camrotaxis>0.724 0.302 0.619 68</camrotaxis>\n')
scale = size * num_elems * 50 #hopefully this won't collide

scale = size * 16
print scale
for i in range(num_elems): #hor
    for j in range(num_elems): #vert
        #create a random side_link in the environment
        #create a random vertical_link in the environment
        
        pos_hor1 = [(i + 1)* scale, (j + 1) * scale, 0.2]
        pos_hor2 = [(-i - 1) * scale, (j + 1) * scale, 0.2]
        pos_vert1 = [(i + 1) * scale, (-j - 1) * scale, 0.2]
        pos_vert2 = [(-i - 1) * scale, (-j - 1) * scale, 0.2]
        index = i * num_elems + j
        
        name_hor1 = 'link_horizontal' + str(index)
        name_hor2 = 'link_horizontal' + str(index + num_elems*num_elems)
        name_vert1 = 'link_vertical' + str(index)
        name_vert2 = 'link_vertical' + str(index + num_elems*num_elems)
        
        f.write('<KinBody file="../objects/array/link_horizontal.xml" name="' + name_hor1 +'">\n')
        f.write('<rotationmat> 1. 0. 0.   0. 1. 0.    0. 0. 1.</rotationmat>\n')
        f.write('<Translation>' + str(pos_hor1[0]) +' ' + str(pos_hor1[1]) + ' ' + str(pos_hor1[2]) + '</Translation>\n')
        f.write('</KinBody>\n')
        
        f.write('<KinBody file="../objects/array/link_horizontal.xml" name="' + name_hor2 +'">\n')
        f.write('<rotationmat> 1. 0. 0.   0. 1. 0.    0. 0. 1.</rotationmat>\n')
        f.write('<Translation>' + str(pos_hor2[0]) +' ' + str(pos_hor2[1]) + ' ' + str(pos_hor2[2]) + '</Translation>\n')
        f.write('</KinBody>\n')
        
        f.write('<KinBody file="../objects/array/link_vertical.xml" name="' + name_vert1 +'">\n')
        f.write('<rotationmat> 1. 0. 0.   0. 1. 0.    0. 0. 1.</rotationmat>\n')
        f.write('<Translation>' + str(pos_vert1[0]) +' ' + str(pos_vert1[1]) + ' ' + str(pos_vert1[2]) + '</Translation>\n')
        f.write('</KinBody>\n')
        
        f.write('<KinBody file="../objects/array/link_vertical.xml" name="' + name_vert2 +'">\n')
        f.write('<rotationmat> 1. 0. 0.   0. 1. 0.    0. 0. 1.</rotationmat>\n')
        f.write('<Translation>' + str(pos_vert2[0]) +' ' + str(pos_vert2[1]) + ' ' + str(pos_vert2[2]) + '</Translation>\n')
        f.write('</KinBody>\n')
        
        
        
#floor:
f.write('<KinBody name="floor">\n')
f.write('<Body type="static">\n')
f.write('<Geom type="box">\n')
f.write('<extents>100 100 0.005</extents>\n')
f.write('<diffuseColor>.8 1 .8</diffuseColor>\n')
f.write('<ambientColor>0.6 0.6 0.6</ambientColor>\n')
f.write('</Geom>\n')
f.write('</Body>\n')
f.write('<Translation>0.0 0.0 -0.1</Translation>\n')
f.write('</KinBody>\n')
f.write('</Environment>\n')
      
f.close()
        
        
      
    
    
  
  


