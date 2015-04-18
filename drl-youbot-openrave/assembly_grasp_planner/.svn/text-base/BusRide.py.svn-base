#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import networkx as nx
import itertools as it
import matplotlib.pyplot as plt

from GraspRegions import GraspRegions
import Assembly
from AssemblyOperation import AssemblyOperation
from copy import deepcopy



class BusRide():

    
    #TODO: pass these in from chairenv.py
    def __init__(self, assembly_operations):
        self.assembly_operations = assembly_operations
        self.__bus_rides = []
        self.__node_to_counts = {}
        self.index = 0
        
        #TODO: should this take in the chairenv.py instead?
        
        self.assembly_tree = nx.DiGraph()
        for operation in self.assembly_operations:
            self.__node_to_counts[operation.name] = operation.robot_count
            #First, add the parent, if it's not already there
            if not operation.name in self.assembly_tree:
                self.assembly_tree.add_node(operation.name)
            for item in operation.assembly_op_list:
                #Then, add the child, if it's not already there
                if not item.name in self.assembly_tree:
                    self.assembly_tree.add_node(item.name)
                #Finally, add an edge:
                self.assembly_tree.add_edge(operation.name, item.name)
                
        root = [n for n,d in self.assembly_tree.in_degree().items() if d==0]
        

        #nx.draw(self.assembly_tree, cmap = plt.get_cmap('jet'))
        #plt.show()
        self.root = root[0]
        
        self.__bus_rides = self.__compute_busrides()
        
        #TODO: In the future, make it so that they're computed online somehow
        
        
    def __compute_busrides(self):
        #TODO: may not need to make this a separate function anymore - but if so, should use it.  Depends how we want to handle the root node.
        #Doesn't really matter who's holding the chair
        busrides =  self.__compute_busrides_helper(self.root)
        #print busrides
        #print len(busrides)
        #print type(busrides)
        return busrides
        #for node in self.root.successors():
        #    #DO a subroutine here: the root node is a special case
        #    self.__compute_busrides_helper(node)
        #    #TODO: merge here and set value to __bus_rides
            
    def __compute_busrides_helper(self, node):
        if len(self.assembly_tree.successors(node)) == 0:
            ret_list = []
            #TODO: fix this so there can be multiple graspers for leaves
            if not node in self.__node_to_counts:
                self.__node_to_counts[node] = 1
            ret_list.append( { node :  [ (node, self.__node_to_counts[node] ) ] } )

            return ret_list
            
        #get number of robots required to carry object:
        count = self.__node_to_counts[node]
        
        
        #print current
        children = []
        for child in self.assembly_tree.successors(node):

            #get the children
            ret_val = self.__compute_busrides_helper(child)
            #print ret_val
            if ret_val is None:
                continue #do nothing
            else:
                #cartesian product it with all the other previous children
                children = self.__cart_product(children, ret_val)
                
        #Finally, let's add the current node
        #TODO: terrible name, let's rename this later

        #Children is a list of tuples
        return self.__generate_children(node, self.assembly_tree.successors(node), children, count)        
                
    def __cart_product(self, l1, l2):
    
        if len(l1) == 0:
            return l2
        if len(l2) == 0:
            return l1
            

        if len(l1) == 0:
            return l2
        if len(l2) == 0:
            return l1
            
        dicts = []
        

        for dict1 in l1:
            for dict2 in l2:
                d1 = dict1.copy()
                d2 = dict2.copy()
                d1.update(d2)
                dicts.append(d1)
                
        return dicts
        
            
    def __combinations_helper(self, tuples, count):
        ret_list = []
        comb_list = []

        for tup in tuples:
            for i in range(tup[1]):
                comb_list.append(tup[0])
                
                

        combos_packed = it.combinations(comb_list, count)
        
        combos = []
        
        for combo in combos_packed:
            #print combo
            combos.append(list(combo))
        #raw_input('analyze')
            
        #Now we have a list of combos.  We just have to put it back in the correct form.
        
        for combo in combos:
            combo.sort()
            combo = zip(combo,map(combo.count,combo))
            ret_list.append(combo)
        #print ret_list
        #raw_input('wait')

        return ret_list
    
    def __generate_children(self, node, successors, children, count):
        ret_val = []
        

        
        #We have a list of dictionaries mapping nodes to tuples
        #The key is the step
        #The list of tuples is a list of the parts being grasped and the number of grasps on that part
        
        #We assume we have a list of all possible ways we could have gotten here at this point
        
        #Then, for each possibility, we need to take all combinations of choose "count"
        
        #Then we need to add all of these to that dictionary.
        
        #Thus, we should iterate over all the children - the ways of getting here first
        
        #Then, build up all the combinations somehow smartly.
        
        #And then add those combinations to the right dictionnary (child)
        
        #Finally, append that dictionary to the ret_val
        #print children
        #print successors
        
        for child in children:
            possible_graspers = []
            for key in successors:
                possible_graspers.append( child[key] )
            #flatten it:
            possible_graspers = [item for sublist in possible_graspers for item in sublist]
            #Now we have a list of all the possible graspers
            print possible_graspers
            
            
            combos = self.__combinations_helper(possible_graspers, count)
            
            for combo in combos:
                new_node = {node : combo}
                old_node = child.copy()
                new_node.update(old_node)
                ret_val.append(new_node)

                
                
        print ret_val
        raw_input('pause')
        return ret_val
            
            
        
    def order_busrides(self, function):
        
        self.__bus_rides = sorted(self.__bus_rides, key=function)    
        
        
    def order_busrides_function(self, busride):
        
        #The number of times a part appears is the length
        
        part_to_counts = {} #TODO: is there a better data structure here?
        for value in busride.itervalues():
            for (part, count) in value:
                if part not in part_to_counts:
                    part_to_counts[part] = 0
                part_to_counts[part] += 1
                
        
        max_value = 0
        for count in part_to_counts.itervalues():
            if count > max_value:
                max_value = count
        
        return max_value
        
    def get_next_busride(self):
        while True:
            if self.index > len(self.__bus_rides) - 1:
                return None #We're out of stuff to get
                            #Should we just generate new busrides?
            else:
                ret_val = self.__bus_rides[self.index]
                self.index += 1
                ret_flag = True
                for key in ret_val:
                    #TODO: THIS IS A VERY BAD HACK, FIX THIS PROPERLY
                    if (not key == ret_val[key][0][0]) and ('fastener' in ret_val[key][0][0]):
                        ret_flag = False
                        break
                        
                if ret_flag:        
                    return ret_val
                else:
                    pass
            
        
    def flag_failed(self, busride):
        pass
    
    def __len__(self):
        return len(self.__bus_rides)
    
    #TODO: constraints on parts - can't hold onto certain parts, for instance
    #TODO: instead of True/False, do we want actual robot number?  Probably want the actual robot number
    #THings to do tomorrow: make a tree
    #when children connect and that goes up through a parent, iterate over each child in all combinations.  Do this recursively and merge upward.  This should generate all busrides in a randomized fashion.
    
    #Prune out monotonically larger bus rides for failures.
    
"""
from chairenv import *

br = BusRide(assembly_operations)
br2 = deepcopy(br)


br.order_busrides(br.order_busrides_function)

print len(br)
for i in range(len(br)):
    x = br.get_next_busride()
    print x

    raw_input('wait')
"""

