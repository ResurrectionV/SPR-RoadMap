import sys
import numpy as np
from matplotlib import pyplot as plt
import math
from collections import deque
import heapq


class Node:
    def __init__(self, data):
        self.data = data
        self.next = None
        self.parent = None


class lList:
    def __init__(self):
        self.head = None



#cd Desktop/2022' 'Fall/Robotics/HW/Machine' 'Problem' 'Assignment' '02/cl1268/SPR
def orient(pt1, pt2, pt3):
    #[x,y]
    val = (float(pt2[1] - pt1[1]) * (pt3[0] - pt2[0])) - (float(pt2[0] - pt1[0]) * (pt3[1] - pt2[1]))
    if (val > 0):
        # Clockwise orientation
        return 1
    elif (val < 0): 
        # Counterclockwise orientation
        return 2
    else: 
        # Collinear orientation
        return 0

def dists(pt1, pt2):
    return round(math.sqrt( (pt2[1] - pt1[1])**2 + (pt2[0] - pt1[0])**2 ), 2)

def ccw(pt1, pt2, pt3):
    return (pt3[1]-pt1[1])*(pt2[0]-pt1[0]) > (pt2[1]-pt1[1])*(pt3[0]-pt1[0])

def check_intersect(pt1, pt2, pt3, pt4):
    return ccw(pt1,pt3,pt4) != ccw(pt2,pt3,pt4) and ccw(pt1,pt2,pt3) != ccw(pt1,pt2,pt4)


def visibility(polygons, poit1, poit2):
    for polyg in polygons:
        for i in range(len(polyg)):
            if poit1 != polyg[i-1] and poit1 != polyg[i] and poit2 != polyg[i-1] and poit2 != polyg[i]:
                if check_intersect(poit1,poit2,polyg[i-1],polyg[i]):
                    return 0
            # fix for when i=0
            if i == 0 and poit1 != polyg[-1] and poit2 != polyg[-1]:
                if check_intersect(poit1,poit2,polyg[-1],polyg[0]):
                    return 0
    return 1


def check_bitan(polygons, pot1, pot2):
    check1 = 0
    check2 = 0
    for polyg in polygons:
        for i in range(-1,len(polyg) - 1):
            if polyg[i] == pot1:
                if orient(pot2,pot1,polyg[i-1]) == orient(pot2,pot1,polyg[i+1]):
                    check1 = 1
            elif polyg[i] == pot2:
                if orient(pot1,pot2,polyg[i-1]) == orient(pot1,pot2,polyg[i+1]):
                    check2 = 1
            if check1 and check2:
                return 1
    return 0

def cons_reflex(polygons, pit1, pit2):
    for polyg in polygons:
        for i in range(-1,len(polyg) - 1):
            if polyg[i] == pit1:
                if polyg[i-1] == pit2 or polyg[i+1] == pit2:
                    return 1
    return 0




'''
Report reflexive vertices
'''
def findReflexiveVertices(polygons):
    vertices=[]
    
    for plg in polygons:
        for i in range(len(plg)):
            if(i != len(plg)-1):
                if orient(plg[i-1],plg[i],plg[i+1]) == 1 or orient(plg[i-1],plg[i],plg[i+1]) == 0:
                    vertices.append(plg[i])
            else:
                if orient(plg[i-1],plg[i],plg[0]) == 1 or orient(plg[i-1],plg[i],plg[0]) == 0:
                    vertices.append(plg[i])

    return vertices

'''
Compute the roadmap graph
'''
def computeSPRoadmap(polygons, reflexVertices):
    vertexMap = dict()
    adjacencyListMap = dict()
    
    for i in range(len(reflexVertices)):
        vertexMap[i+1] = reflexVertices[i]
        adjacencyListMap[i+1] = []


    for i in range(len(reflexVertices)-1):
        for j in range(i+1, len(reflexVertices)):
            if visibility(polygons, reflexVertices[i], reflexVertices[j]):
                if cons_reflex(polygons, reflexVertices[i], reflexVertices[j]) or check_bitan(polygons, reflexVertices[i], reflexVertices[j]):
                    adjacencyListMap[i+1].append([j+1,dists(reflexVertices[i],reflexVertices[j])])
                    adjacencyListMap[j+1].append([i+1,dists(reflexVertices[i],reflexVertices[j])])
    
    return vertexMap, adjacencyListMap

'''
Perform uniform cost search 
'''
def uniformCostSearch(adjListMap, start, goal):
    # initialize open list and closed list
    openList = [(0, start, [])]  # (f, node, path)
    closedList = set()
    
    while openList:
        # get the node with the smallest f value
        f, currNode, currPath = heapq.heappop(openList)
        
        if currNode == goal:
            return currPath, f
        
        if currNode in closedList:
            continue
        
        closedList.add(currNode)
        
        # expand the current node
        for neighbor, dist in adjListMap[currNode]:
            if neighbor in closedList:
                continue
            
            g = f + dist
            h = 0
            newF = g + h
            
            newPath = currPath + [neighbor]
            
            # add the neighbor to the open list
            heapq.heappush(openList, (newF, neighbor, newPath))
    
    # if no path is found
    return None, float('inf')
    



    
    return path, pathLength

'''
Agument roadmap to include start and goal
'''
def updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2):
    updatedALMap = dict()
    startLabel = 0
    goalLabel = -1

    vertexMap[startLabel] = [x1,y1]
    vertexMap[goalLabel] = [x2,y2]
    adjListMap[startLabel] = []
    adjListMap[goalLabel] = []


    for i in range(len(vertexMap)-2):
        if visibility(polygons, vertexMap[i+1], vertexMap[-1]):
            adjListMap[-1].append([i+1,dists(vertexMap[i+1],vertexMap[-1])])
        if visibility(polygons, vertexMap[i+1], vertexMap[0]):
            adjListMap[0].append([i+1,dists(vertexMap[i+1],vertexMap[0])])
    updatedALMap = adjListMap
    return startLabel, goalLabel, updatedALMap

if __name__ == "__main__":
    
    #python3 spr.py env_01.txt 1.0 2.0 3.0 4.0
    #python3 visualize.py env_01.txt


    # Retrive file name for input data
    if(len(sys.argv) < 6):
        print("Five arguments required: python spr.py [env-file] [x1] [y1] [x2] [y2]")
        exit()
    
    filename = sys.argv[1]
    x1 = float(sys.argv[2])
    y1 = float(sys.argv[3])
    x2 = float(sys.argv[4])
    y2 = float(sys.argv[5])

    # Read data and parse polygons
    lines = [line.rstrip('\n') for line in open(filename)]
    polygons = []
    for line in range(0, len(lines)):
        xys = lines[line].split(';')
        polygon = []
        for p in range(0, len(xys)):
            polygon.append(list(map(float, xys[p].split(','))))
        polygons.append(polygon)

    # Print out the data
    print ("Pologonal obstacles:")
    for p in range(0, len(polygons)):
        print (str(polygons[p]))
    print ("")

    # Compute reflex vertices
    reflexVertices = findReflexiveVertices(polygons)
    print ("Reflexive vertices:")
    print (str(reflexVertices))
    print ("")

    # Compute the roadmap 
    vertexMap, adjListMap = computeSPRoadmap(polygons, reflexVertices)
    print ("Vertex map:")
    print (str(vertexMap))
    print ("")
    print ("Base roadmap:")
    print (str(adjListMap))
    print ("")

    # Update roadmap
    start, goal, updatedALMap = updateRoadmap(polygons, vertexMap, adjListMap, x1, y1, x2, y2)
    print ("Updated roadmap:")
    print (str(updatedALMap))
    print ("")

    # Search for a solution     
    path, length = uniformCostSearch(updatedALMap, start=3, goal=10)
    print ("Final path:")
    print (str(path))
    print ("Final path length:" + str(length))
    

    # Extra visualization elements goes here
