# -*- coding: utf-8 -*-
"""
Created on Mon Apr 18 20:49:53 2016

@author: Sachin Badgujar
"""

import random
import numpy as np
from scipy.spatial import distance
import math
import pygame
from pygame.locals import *
import sys

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
    def addNode(self, pt):
        if not (self.nodes.has_key(pt)):
            self.nodes[pt] = Vertex().addVertex(pt)
            #print self.nodes[pt].point
            #print self.nodes[pt].adjlist
        return self
    def addEdge(self, src, dst):
        self.addNode(src)
        self.addNode(dst)
        self.nodes[src].addAdjcent(self.nodes[dst])
        #self.nodes[src].adjlist[dst] = distance(src,dst)
    
    
def findNearest(x,g):
    k = np.array(g.nodes.keys())
    #print 'X', x
    #print 'keys',k
    dist = np.sqrt(np.sum((k-x)**2,axis=1))
    #print dist
    return k[np.argmin(dist),:]
        
def towardsXrand(xsrc,xdst,epsi):
    if distance.euclidean(xsrc,xdst) <= epsi:
        return xdst
    else:
        theta = math.atan2(xdst[1]-xsrc[1],xdst[0]-xsrc[0])
        newpt = xsrc[0] + epsi*math.cos(theta), xsrc[1] + epsi*math.sin(theta) 
        return newpt
        


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

def roadto(dst,path):
    if dst == (1,1):
        return
    parent = path[dst]    
    pygame.draw.line(screen,0x00ff00,dst,parent,3)
    pygame.display.update()
    pygame.time.delay(100)    
    roadto(path[dst],path)
    
    
if __name__ == '__main__':
    pygame.init()
    screen = pygame.display.set_mode([300,300])
    white = 255, 240, 200
    black = 20, 20, 40
    screen.fill(black)
    f = Graph()
    epsi = 25
    qinit = (1,1)
    f.addNode(qinit)
    pygame.display.update()
    pygame.time.delay(3000)
    #f.addNode((2,2))
    for k in range(1000):
        xrand = random.randint(0,300),random.randint(0,300)
        #xrand = 3,3
        xnear = findNearest(xrand,f)
        xnew = towardsXrand(xnear,xrand,epsi)
        f.addEdge(tuple(xnear),xnew)
        pygame.draw.line(screen,white,xnear,xnew)
        pygame.display.update()
    ''' 
    print f.nodes
    for value in f.nodes.values():
        print value.point
        print 'Value', value.adjlist
    '''    
    visit,path = dijsktra(f, (1,1))
    near_pt = findNearest((270,110),f)
    '''
	path = []
    for node in f.nodes:
        print node
        print f.nodes[node].adjlist.keys()
    ''' 
        
    pygame.draw.circle(screen,0xff00ff,(270,110),4,4)
    roadto(tuple(findNearest((270,110),f)), path)
    running = True
    while running:
        for e in pygame.event.get():
            if (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.")
    
    #f.showVertex()
    #print f.showVertex()    
    #x=f.showVertex()
    #print x
    #f.addAdjcent(f)
    #for v in f.adjlist:
    #    print v.point