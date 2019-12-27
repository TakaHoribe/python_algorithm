# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import matplotlib.pyplot as plt

goal_t = 20
goal_s = 40
t_max = 21
weight_v = 10.0 # weight for (vref - v)^2
a_max = 0.5

Ns = 40
sarr = list(range(Ns))
vref = [3] * Ns
vmax = [10] * Ns
vmax[Ns-1] = 0

"""
fig1, ax1 = plt.subplots(1, 1)
fig1_init_s = True
fig1_init_l = True
"""

print(vmax)
    
class Node:
    
    def __init__(self, t, s, v):
        self.var = (t, s, v)
        self.hs  = (s - goal_s)**2
        self.gs  = 0
        self.fs  = 0
        self.parent_node = None

    def isGoal(self):
        # return (self.var[0] >= goal_t) or self.var[1] >= goal_s
        return self.var[1] >= goal_s
    
    
    
class NodeList(list):
    def find(self, t, s, v):
        l = [p for p in self if p.var==(t, s, v)]
        return l[0] if l != [] else None
    def remove(self,node):
        del self[self.index(node)]
        

def getRefV(s):
    n = min(sarr, key=lambda x:abs(x-s))
    return vref[n]

def getVmax(s):
    n = min(sarr, key=lambda x:abs(x-s))
    return vmax[n]

def printNode(n, color):
    t = n.var[0]
    s = n.var[1] 
    plt.figure(1)
    plt.scatter(t, s, color=color)
      
    plt.xlim((0, 23))
    plt.ylim((0, 45))

    m = n.parent_node
    if m:
        mt = m.var[0]
        ms = m.var[1]
        plt.plot([mt, t], [ms, s], color=color)
    
    plt.grid()
    plt.pause(.01)
    plt.grid()



    
def isOccupied(t, s):
    if 4 <= t and t <= 7:
        if 10 <= s and s <= 14:
            return True
    
    return False

if __name__ == '__main__':
    
    open_list     = NodeList()
    close_list    = NodeList()
    start_node    = Node(0, 0, 0) # t, s, v
    start_node.fs = start_node.hs
    open_list.append(start_node)
    
    count = 0
    
    # visualize occpuied area
    for t in range (11):
        for s in range(20):
            if (isOccupied(t, s)):
                plt.scatter(t, s, color="black", marker="s", s=200)
    
    
    try: 
        while (True):
            
            print("count = ", count, ", len(open_list) = ", len(open_list))
            
            # if the open_list is empty, it means no solution exists.
            if open_list == []:
                print("there is no open list")
                break
            
            
            
            # get the optimal (minimum) cost node from open_list
            n = min(open_list, key=lambda x:x.fs)
            open_list.remove(n)
            close_list.append(n)
            
            # print("current node : t = ", n.var[0], ", s = ", n.var[1], ", v = ", n.var[2])
            
            printNode(n, "lightgray")
            
            # if the optimal node is a goal, end.
            if n.isGoal():
                end_node = n
                break
                
            #f*() = g*() + h*() -> g*() = f*() - h*()
            n_gs = n.fs - n.hs
        
    
            # calculate candidates
            vi = n.var[2]
            for v in [vi-2, vi-1, vi, vi+1, vi+2]: # should be calculated by amax
                t = n.var[0] + 1
                s = n.var[1] + vi
                if (v >= 0 and v <= getVmax(s) and t < t_max and (not isOccupied(t, s))):

                    m = open_list.find(t, s, v)
                    m_dgs = weight_v * (getRefV(s) - v)**2
                    
                    if m:
                        if m.fs > (n_gs + m_dgs) + m.hs:
                            # print("a : candidate is already in open_list. Update optimal cost.")
                            m.fs = (n_gs + m_dgs) + m.hs
                            m.parent_node = n
                        # print("b : candidate is already in open_list. But this has higher cost, ignore this node.")
                    else:
                        m = close_list.find(t, s, v)
                        if m:
                            if m.fs > (n_gs + m_dgs) + m.hs:
                                # print("c : candidate is already in close_list. Update optimal cost")
                                m.fs = (n_gs + m_dgs) + m.hs
                                m.parent_node = n
                                open_list.append(m)
                                close_list.remove(m)
                            # else:
                                # print("d : candidate is already in close_list, but this has higher cost, ignore this node.")
                        else:
                            # print("e : candidate is none of both open & close list. store this node to open_list.")
                            m = Node(t, s, v)
                            m.fs = (n_gs + m_dgs) + m.hs
                            m.parent_node = n
                            open_list.append(m)
                # else:
                    # print("f : v < 0")
            count += 1
            
    except KeyboardInterrupt:
        print("end")
    
    # visualize result
    n = end_node
    opt_sarr = []
    opt_varr = []
    while True:
        if n:
            printNode(n, "red")
            opt_sarr.append(n.var[1])
            opt_varr.append(n.var[2])
        else:
            break
        n = n.parent_node
        
    
    plt.grid()
    
    
    fi2 = plt.figure(3)
    ax2 = fi2.add_subplot(111)
    ax2.plot(opt_sarr, opt_varr, label="result")
    ax2.plot(sarr, vref, label="reference")
    ax2.plot(sarr, vmax, label="max")
    ax2.legend()
                
