#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import time
import random

import openravepy as orpy


#TODO: type checking

class AOTNode:
    """
    An AOT node just has pointers to children (one more level of disassembly of a subassembly)
    as well as the subassembly in this level of the tree.  Note that the subassemblies
    themselves take care of the details as to how the individual pieces fit together.
    Parts are the atomic units that make up the tree
    """
    
    def __init__(self, children, subassembly, parts):
        self.children = children
        self.subassembly = subassembly
        self.parts = parts #TODO: necessary?
        
        
class Grasps:
    """
    Fixed grasps is a list of grasp positions
    Unfixed grasps is a dictionary of part names to a list of potential grasp positions
    """
    
    def __init__(self, fixed_grasps, unfixed_grasps):
        self.fixed_grasps = fixed_grasps
        self.unfixed_grasps = unfixed_grasps
        
    def no_conflicts(self):
        #If the map has a list for all the unfixed grasps that's not empty, and if none of the fixed_grasps are None, then this is graspable
        is_graspable = True
        for grasp in self.fixed_grasps:
            is_graspable = is_graspable and (not grasp is None)
            if not is_graspable:
                return False #for short circuting
        
        for grasp in self.unfixed_grasps:
            is_graspable = is_graspable and (len(grasp) > 0)
            if not is_graspable:
                return False #for short circuting
                
        return is_graspable
        
    def fix_grasps(self, part, grasp):
        self.unfixed_grasps[part] = []
        self.fixed_grapss[part] = grasp
        
            
        

def search(aot_root, known_grasps):
    #instead of just returning satisfiability, return actual grasps
    #In order to do this, replace the boolean return value with  a tuple:
    #Boolean return, grasp
    #Return the grasp if it's of type Part
    # or if we had any returned
    #Otherwise, just set the grasp as None
    #In the final loop where we do the recursive call, save the grasp if it's not None (if it's a part)
    #and return it as the second value

    """
    Performs the backtrack search on the subassembly and attempts to recover valid part lists
    If we can make it all the way back to the children, then this returns true.  Otherwise, it returns false.
    Note that the AOT tree is fixed - we're searching over that tree to see what grasps exist
    """
    
    #1. is this piece assmbleable?
    #2. iterate over all the children
    #3. recurse - return true if all children are assemblable, else return false
    
    #Base condition:
    if type(aot_root.subassembly) is Part:
        name = aot_root.subassembly.name
        return (True, {name : known_grasps.fixed_grasps[name] } ) #Of course leaves are graspable - you only need one robot!
    
    #TODO: get this
    grasps = self_assemblable([child for child in aot_root.children], known_grasps) #TODO: get this function!
    if not grasps.no_conflicts() #if there are no grasps...
        return (False, None) # :-(
    
    #This is assemblable, so now let's make sure all the subassemblies are assemblable
    
    satisfiable_grasps = True
    map_of_grasps = {}
    for part in grasps.unfixed_grasps: #TODO: do we want to randomize this in some way or take turns on the parts?
        for grasp in grasps.unfixed_grasps[part]:
            grasps_copy = grasps.copy() #This copy is our current choice
            grasps_copy.fix_grasp(part, grasp)
            

            for child in aot_root.children: #See if that grasp works
                search_result = search(child, grasps_copy)
                satisfiable_grasps = satisfiable_grasps and search_result[0]
                map_of_grasps.update( search_result[1] )
    
            if statisfiable_grasps:
                return (satisfiable_grasps, grasps)
                
    return (False, None) #we've exhausted all grasps and thus this doesn't work :-(
    
    
    
    
def convert_to_networkx(aot_root):
    #May be a useful utility in the future
    pass
