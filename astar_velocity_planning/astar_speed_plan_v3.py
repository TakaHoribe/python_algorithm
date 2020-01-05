# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import matplotlib.pyplot as plt

tmax = 20.0
dt = 0.1
dt_inv = 1.0 / dt
tdmax = round(tmax / dt)

smax = 10.0
ds = 0.1
ds_inv = 1.0 / ds
sdmax = round(smax / ds)

dv = ds / dt
dv_inv = 1.0 / dv


weight_v = 1.0 # weight for (vref - v)^2

sdarr = list(range(sdmax))
scarr = []
for s in sdarr:
    scarr.append(s * ds)

vcref = [3] * sdmax
vcmax = [10] * sdmax
vcmax[sdmax-1] = 0

aclim = 1.0



"""
fig1, ax1 = plt.subplots(1, 1)
fig1_init_s = True
fig1_init_l = True
"""


def d2cT(td):
    return td * dt

def d2cS(sd):
    return sd * ds

def d2cV(vd):
    return vd * dv

def c2dT(tc):
    return round(tc * dt_inv)

def c2dS(sc):
    return round(sc * ds_inv)

def c2dV(vc):
    return round(vc * dv_inv)

vdref = []
vdmax = []
for v in vcref:
    vdref.append(c2dV(v))

for v in vcmax:
    vdmax.append(c2dV(v))


class Node:
    
    def __init__(self, td, sd, vd):
        self.var = (td, sd, vd)
        self.hs  = d2cS(s - sdmax)**2
        self.gs  = 0
        self.fs  = 0
        self.parent_node = None

    def isGoal(self):
        # return (self.var[0] >= goal_t) or self.var[1] >= goal_s
        # return self.var[1] >= goal_s
        # return (self.getSd() >= sdmax) or self.getTd() >= tdmax
        return (self.getSd() == sdmax) 
    
    def getTd(self):
        return self.var[0]
    
    def getSd(self):
        return self.var[1]
    
    def getVd(self):
        return self.var[2]
    
    
    
class NodeList(list):
    def find(self, t, s, v):
        l = [p for p in self if p.var==(t, s, v)]
        return l[0] if l != [] else None
    def remove(self,node):
        del self[self.index(node)]
        

# def getRefVc(sd):
#     n = min(sdarr, key=lambda x:abs(x-sd))
#     return vcref[n]

def getRefVd(sd):
    n = min(sdarr, key=lambda x:abs(x-sd))
    return vdref[n]

def getVdmax(sd):
    n = min(sdarr, key=lambda x:abs(x-sd))
    return vdmax[n]

def printNode(n, color):
    t = n.getTd()
    s = n.getSd()
    plt.figure(1)
    plt.scatter(t, s, color=color)
      
    plt.xlim((0, tdmax+1))
    plt.ylim((0, sdmax+1))

    m = n.parent_node
    if m:
        mt = m.getTd()
        ms = m.getSd()
        plt.plot([mt, t], [ms, s], color=color)
    
    plt.grid()
    plt.pause(.01)
    plt.grid()

def getRangeVd(vd):
    delta_vc = aclim * dt
    delta_vd = c2dV(delta_vc)
    # print("vd = ", vd, ", delta_vc = ", delta_vc, ", delta_vd = ", delta_vd)
    range_vd = []
    a = range(vd - delta_vd, vd + delta_vd + 1)
    for vdi in a:
        if vdi >= 0:
            range_vd.append(vdi)
    # print("range_vd = ", range_vd)
    # print("a = ", list(a))
    return range_vd

    
def isOccupiedD(td, sd):

    if 2 <= td and td <= 7:
        if 10 <= sd and sd <= 24:
            return True

    
    return False

def generateTSVCandidates(n):
    
    td = n.getTd()
    sd = n.getSd()
    vd = n.getVd()
    vc = d2cV(vd)
    vdmax = c2dV(vc + aclim * dt)
    vdmin = c2dV(max(vc - aclim * dt, 0.0))
    
tsv = []    
    for i in range(N):
        for j in range(M):
            

            tmp = (ti, si, vi)
            tsv.append((tmp)
    
   return tsv
    
    
    

if __name__ == '__main__':
    
    open_list     = NodeList()
    close_list    = NodeList()
    vc0 = 0.0
    start_node    = Node(0, 0, c2dV(vc0)) # t, s, v
    start_node.fs = start_node.hs
    open_list.append(start_node)
    
    count = 0
    
    # visualize occpuied area
    for td in range(tdmax):
        for sd in range(sdmax):
            if (isOccupiedD(td, sd)):
                plt.scatter(td, sd, color="black", marker="s", s=200)
    
    
    try: 
        while (True):
            
            print("count = ", count, ", len(open_list) = ", len(open_list))
            
            # if the open_list is empty, it means no solution exists.
            if open_list == []:
                print("there is no open list")
                end_node    = Node(0, 0, 0) # t, s, v
                break
            
            
            
            # get the optimal (minimum) cost node from open_list
            n = min(open_list, key=lambda x:x.fs)
            open_list.remove(n)
            close_list.append(n)
            
            # print("current node : t = ", n.var[0], ", s = ", n.var[1], ", v = ", n.var[2])
            
            # printNode(n, "lightgray")
            
            # if the optimal node is a goal, end.
            if n.isGoal():
                end_node = n
                break
                
            #f*() = g*() + h*() -> g*() = f*() - h*()
            n_gs = n.fs - n.hs
        
    
            # calculate candidates
            n_vd = n.getVd()
            n_td = n.getTd()
            
            
            tsvlists = generateTSVCandidates(n)
            
            
            for vd in getRangeVd(n_vd): # should be calculated by amax
                td = n.getTd() + 1
                sd = n.getSd() + n_vd
                #print("v = ", vd, ", t = ", td, ", s = ", sd, ", ", vd <= getVdmax(sd), td <= tdmax, (not isOccupiedD(td, sd)))
                if (vd <= getVdmax(sd) and td <= tdmax and sd <= sdmax and (not isOccupiedD(td, sd))):

                    m = open_list.find(td, sd, vd)
                    m_dgs = weight_v * d2cV(getRefVd(sd) - vd)**2
                    
                    if m:
                        if m.fs > (n_gs + m_dgs) + m.hs:
                            # print("a : candidate is already in open_list. Update optimal cost.")
                            m.fs = (n_gs + m_dgs) + m.hs
                            m.parent_node = n
                        # print("b : candidate is already in open_list. But this has higher cost, ignore this node.")
                    else:
                        m = close_list.find(td, sd, vd)
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
                            m = Node(td, sd, vd)
                            m.fs = (n_gs + m_dgs) + m.hs
                            m.parent_node = n
                            open_list.append(m)
                # else:
                    # print("f : v < 0")
            count += 1
            
    except KeyboardInterrupt:
        print("end")

    
    # visualize result
    if end_node:
        n = end_node
        opt_scarr = []
        opt_vcarr = []
        while True:
            if n:
                printNode(n, "red")
                opt_scarr.append(d2cS(n.getSd()))
                opt_vcarr.append(d2cV(n.getVd()))
            else:
                break
            n = n.parent_node
            
        
        plt.grid()
        
        
        fi2 = plt.figure(3)
        ax2 = fi2.add_subplot(111)
        ax2.plot(opt_scarr, opt_vcarr, label="result")
        ax2.plot(scarr, vcref, label="reference")
        ax2.plot(scarr, vcmax, label="max")
        ax2.legend()
                

    plt.show()