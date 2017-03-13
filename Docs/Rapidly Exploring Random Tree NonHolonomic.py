# -*- coding: utf-8 -*-
"""
Created on Mon Apr 18 20:49:53 2016

@author: Sachin Badgujar
Email: sbadguja@uncc.edu
"""

import random
import numpy as np
from scipy.spatial import distance
import math
import pygame
from pygame.locals import *
import sys
import os


S=12                                    #Velocity of car
h=1                                   #Integration factor for runge kutta
L=15                                    #distance between the wheels

class Vertex:
    def __init__(self):
        self.point = ()
        self.adjlist = {}               #list format is {(X,Y):distance, (X2,Y2):distance2}
    def addVertex(self,pt):
        self.point=pt
        return self                     #just for the addNode method
    def addAdjcent(self, vrtx):
        #print vrtx.point
        self.adjlist[vrtx.point] = distance.euclidean(self.point,vrtx.point)
    def showVertex(self):
        return self.point
    def showAdjlist(self):
        return self.adjlist
 
        
    
        
class Graph:
    def __init__(self):
        self.nodes = {}
        self.edges = {}                         #format {(child):parent}
    def addNode(self, pt):
        if not (self.nodes.has_key(pt)):
            self.nodes[pt] = Vertex().addVertex(pt)
        return self
    def addEdge(self, src, dst):
        if src!=None:                                   #for Normal nodes/edges
            self.addNode(dst[-1])
            self.nodes[src].addAdjcent(self.nodes[dst[-1]])
            self.edges[dst[-1]] = dst
        else:                                            #for first edge source=None, dst=qinit
           self.addNode(dst)
           self.edges[dst] = src
    
    
def findNearest(x,g):
    if type(g) is list:
        k = np.array(g)
    else:
        k = np.array(g.nodes.keys())
    #print 'X', x
    #print 'keys',k
    dist = np.sqrt(np.sum((k-x)**2,axis=1))
    #print dist
    return tuple(k[np.argmin(dist),:])
        

# Graph should be a graph structure and initial shoud be a point (x,y)
def dijsktra(graph, initial):
  visited = {initial: 0}
  path = {}

  nodes = set(graph.nodes)

  while nodes: 
    min_node = None
    for node in nodes:
      if node in visited:
        if min_node is None:
          min_node = node
        elif visited[node] < visited[min_node]:
          min_node = node

    if min_node is None:
      break

    nodes.remove(min_node)
    current_weight = visited[min_node]
    
    current_node = graph.nodes[min_node]
    for edge in current_node.adjlist:
      weight = current_weight + current_node.adjlist[edge]
      if edge not in visited or weight < visited[edge]:
        visited[edge] = weight
        path[edge] = min_node

  return visited, path

def findPath(dst,f,src):
    path = []
    if f.edges[dst] == src:
        return
    path = f.edges[dst]    
    if path[0] != None:
        pygame.draw.lines(screen,0x00ff00,False,np.array([x[:2] for x in path]),2)
    else:
        pygame.draw.lines(screen,0x00ff00,False,np.array([x[:2] for x in path[1:]]),2)
    pygame.time.wait(100)    
    pygame.display.update()
    findPath(path[0],f,src)
    
#returns a new point based on runge kutta integration   
def steerNgo(current,random,h):
    nearest_points = []
    left_pts = integrate_rk(current,math.pi/4,h)
    nearest_points.append(findNearest(random,left_pts))
    straight_pts=integrate_rk(current,0,h)
    nearest_points.append(findNearest(random,straight_pts))
    right_pts=integrate_rk(current,-math.pi/4,h)
    nearest_points.append(findNearest(random,right_pts))
    q_near=findNearest(random,nearest_points)
    if nearest_points.index(q_near) == 0:                   #if the distance of new random point is zero 
        if distance.euclidean(random[:2],q_near[:2]) == 0:  # return the point (intermediate point)
            return left_pts[:left_pts.index(q_near)]
        else:                                               #else return all integrated points 
            return left_pts
    if nearest_points.index(q_near) == 1:
        if distance.euclidean(random[:2],q_near[:2]) == 0:
            return straight_pts[:straight_pts.index(q_near)]
        else:
            return straight_pts
    if nearest_points.index(q_near) == 2:
        if distance.euclidean(random[:2],q_near[:2]) == 0:
            return right_pts[:right_pts.index(q_near)]
        else:
            return right_pts


#Runge Kutta integration  for dubins car 
def integrate_rk(pt,phi,h):

    q = pt
    q_list = []
    for i in range(5):
        k1 = (h*S*math.cos(q[2]),h*S*math.sin(q[2]),h*S*math.tan(phi)/float(L))   #(x,y,theta)
        
        k2 = (h*S*math.cos(q[2]+(0.5*h)),h*S*math.sin(q[2]+(0.5*h)),h*S*math.tan(phi+k1[2]*0.5*h)/float(L))
        
        k3 = (h*S*math.cos(q[2]+(0.5*h)),h*S*math.sin(q[2]+(0.5*h)),h*S*math.tan(phi+k2[2]*0.5*h)/float(L))
        
        k4 = (h*S*math.cos(q[2]+h),h*S*math.sin(q[2]+h),h*S*math.tan(phi+k3[2]*h)/float(L)) 
        
        
        x_new = q[0] + (k1[0]+2*k2[0]+2*k3[0]+k4[0])*h*(1/float(6))
        y_new = q[1] + (k1[1]+2*k2[1]+2*k3[1]+k4[1])*h*(1/float(6))
        theta_new = q[2] + (k1[2]+2*k2[2]+2*k3[2]+k4[2])*h*(1/float(6))
        q_list.append(q)
#        pygame.draw.line(screen,white,q[:2],(x_new,y_new))
#        pygame.display.update()
        q=(x_new,y_new,theta_new)
        
        
    return q_list                                           #return whole list.
    
if __name__ == '__main__':
    os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (30,30)      #position pygame window
    win_x = 400                                                 #C space dimension
    win_y = 300                                                 #C space dimension
    pygame.init()
    screen = pygame.display.set_mode([win_x,win_y])
    white = 255, 240, 200
    black = 20, 20, 40
    screen.fill(black)
    f = Graph()                                                 #instantiate graph
    running = True                                              #Unless qinit is selected 
    while running:    
        for e in pygame.event.get():
            if e.type == MOUSEBUTTONDOWN:                       #if user clicks
                qinit = (e.pos+(math.pi/4,))
                pygame.draw.circle(screen,0x7cfc00,e.pos,6,6)   #Draw quinit
                pygame.display.update()
                running = False
        
    f.addEdge(None,qinit)
    for k in range(1000):
        xrand = random.randint(0,win_x),random.randint(0,win_y),0
        xnear = findNearest(xrand,f)
        xnew_list = steerNgo(xnear,xrand,h)
        f.addEdge(tuple(xnear),xnew_list)
        pygame.draw.lines(screen,white,False,np.array([x[:2] for x in xnew_list]),1)
        pygame.display.update()
    
    #visit,path = dijsktra(f, (1,1,0))

    while True:    
        for e in pygame.event.get():
            if (e.type == KEYUP and e.key == K_ESCAPE):
                pygame.quit()
                sys.exit("Exiting")
            if e.type == MOUSEBUTTONDOWN:
                near_goal_pt0 = findNearest(e.pos+(0,),f)
                pygame.draw.circle(screen,0xff0000,e.pos,6,6)
                findPath(near_goal_pt0,f,None)
  

    
    
