# File Name     :   PSO_local.py
# Created on    :   25/11/2016
# Author        :   Shawn Goh <shawngoh87@hotmail.com>

"""
This is a simplified version of PSO, emphasizing:
-- Direction of each swarm unit.
-- Locality, therefore updateBest is removed, and costs are calculated based on local inputs.
-- Trail left behind by each swarm unit.
-- A constantly arranged and trimmed pointArray buffer.
-- Scalability, adding more bots doesn't affect the algorithm as it is based on local rules.

Idea:
1.  Add local bias for points made for "trail"
2.  Use grid to replace point array.
    --  Initialize grid with size of searchSpace
    --  Whenever a coordinate is added, the grid[i][j] += 1\
        where i and j are the bounded boxes.
3.  Increase the size of the checking boxes.
4.  Grid checking to return spread consistency, grid size = 1% of search space?

Issues:
1.  The points still have noticeable clustering.
"""
#-----------------------IMPORTS & NAMESPACE-------------------------#
import os, sys
import random
import numpy as np
import matplotlib.pyplot as plt
import time
from math import *

#----------------------------FUNCTIONS------------------------------#

def testGraphs():
    """Stub. Mock functions for contour and humidity."""
    pass
    contour = sin(x) + sin(y)
    humidity = sin(x+2) + sin(y+2)
def add2DVectors(vec1, vec2):
    """Return vec1 + vec2"""
    return [vec1[0] + vec2[0], vec1[1] + vec2[1]]
def checkForward(position, velocity, angle, angleStep, pointArray):
    """Return cost count
    This function creates a triangle in the direction of the velocity,
    and counts the point in the triangle. Then plots three lines and store
    it in the global lineList for removal after the iteration.
    2------1------3
     \           /
      \         /       0 is the current position
       \       /        1 is the current position + current velocity,
        \     /                 which equals to the next position
         \   /          2 is cLeft
          \ /           3 is cRight
           0
    """
    count = 0
    px, py = position
    pointMid = rotate(velocity, angle)
    pointLeft = rotate(pointMid, angleStep / 2)
    pointRight = rotate(pointMid, -angleStep / 2)
    pointLeft = [x*checkScale for x in pointLeft]
    pointRight = [x*checkScale for x in pointRight]
    cLeft = add2DVectors(position, pointLeft)
    cRight = add2DVectors(position, pointRight)
    lineList.append(plt.plot([cLeft[0], cRight[0]], [cLeft[1], cRight[1]], color='k', lw=1),)
    lineList.append(plt.plot([cLeft[0], px], [cLeft[1], py], color='k', lw=1),)
    lineList.append(plt.plot([cRight[0], px], [cRight[1], py], color='k', lw=1),)
    for point in pointArray:
        if inPolygon(point, cLeft, cRight, position):
            count += 1
    return count
def cost(position, velocity, angleLimit, angleStep, costRadius, pointArray):
    """Return next position and velocity
    This function checks the path forward using checkForward().
    Then checks if the next move is out-of-bound, reassign a move if yes.
    """
    costArray = []
    px, py = position
    for angle in range(-angleLimit, angleLimit + angleStep, angleStep):
        count = checkForward(position, velocity, angle, angleStep, pointArray)
        costArray.append([angle, count])
    costArray = sorted(costArray, key = lambda a:a[1])
    medianAngle = roulette(costArray, rouletteDegree)[0]
    vx, vy = rotate(velocity, medianAngle)
    cpoint = [px + vx, py + vy]
    while(outOfBound(cpoint)):
        temp = random.random() * 360 # Reflects when boundary hit
        vx, vy = rotate([vx, vy], temp)
        cpoint = [px + vx, py + vy]
    rand = random.uniform(0.5, 1.0) # Prevent clustering of resonant steps
    return [px + rand*vx, py + rand*vy], [vx, vy]
def dist(pointA, pointB):
    """Return Euclidean distance between pointA and pointB"""
    return np.linalg.norm(np.array(pointA) - np.array(pointB))
def inPolygon(point, *args):
    """Return True if the point is inside a convex polygon.
    Checks whether the target point is inside a polygon specified
    by the vertices.
    Example call: check point [2,3] in a triangle.
    inPolygon([2,3], [0,0], [10,10], [0,10])
    """
    prev = args[-1]
    for vertex in args:
        if onLeftSide(vertex, prev, point) == False:
            return False
        prev = vertex
    return True
def onLeftSide(linePointA, linePointB, point):
    """Return True if point is on left of line A to B
    Maths. Geometry.
    """
    x, y = point
    xA, yA = linePointA
    xB, yB = linePointB
    A = -(yB - yA)
    B = xB - xA
    C = -(A * xA + B * yA)
    D = A * x + B * y + C
    if D >= 0:
        return True
    else: return False
def outOfBound(point):
    """Return True if point is out-of-bound
    Checks global variable searchSpace if the target point
    is out-of-bound
    """
    x, y = point
    xMax, yMax = searchSpace[0], searchSpace[1]
    if (x <= xMax[0] or x >= xMax[1]) or (y <= yMax[0] or y >= yMax[1]):
        return True
    else: return False
def randomVector(range):
    """Return a float between range"""
    return random.uniform(range[0], range[1])
def rotate(vector, degree):
    """Return rotated vector"""
    x, y = vector
    degree = degree/180 *pi
    return [x*cos(degree) - y*sin(degree), x*sin(degree) + y*cos(degree)]
def roulette(array, degree):
    """Return an element of the array by weighted randomization.
    The degree will determine the weight of the elements.
    Example: a 4-element array will have the following weights--
    Degree 1: [4,3,2,1]     normalized to   [0.4, 0.3, 0.2, 0.1]
    Degree 2: [16,9,4,1]    normalized to   [0.53, 0.3, 0.13, 0.03]
    Degree 3: [64,27,8,1]   normalized to   [0.64, 0.27, 0.08, 0.01]
    """
    numel = len(array)
    rand = random.random()*sumAscend(numel, degree)
    for i in range(len(array)):
        rand -= numel**degree
        numel -= 1
        if rand <= 0:
            return array[len(array) - numel - 1]
def sumAscend(length, degree):
    """Return 1+2+3+...+length
    Degree will increase the power of addition
    """
    if length > 1:
        return length ** degree + sumAscend(length - 1, degree)
    else:
        return length

def init(step, maxParticle, searchSpace):
    """Return position and velocity array of size(maxParticle)"""
    particlePosition = []
    particleVelocity = []
    for i in range(maxParticle):
        particlePosition.append([randomVector(searchSpace[0]), randomVector(searchSpace[1])])
        temp = random.random()*360
        particleVelocity.append(rotate([step, step], temp))
    return particlePosition, particleVelocity

#----------------------------CONSTANTS------------------------------# SO MANY GLOBAL VARIABLES OH GOD
global rouletteDegree
global counter
global searchSpace
global step
global ellipseRatio
global lineList
global checkScale
checkScale = 2
lineList = []
ellipseRatio = 5
rouletteDegree = 5.0
counter = [0]*3
stop = 1
gap = 50
step = 20
iteration = 0
maxParticle = 5
maxIteration = 1000
costRadius = 50
angleLimit = 30
angleStep = 30
length = 500.0
searchSpace = [[-(length/2),(length/2)], [-(length/2), (length/2)]] # [[x_range], [y_range]]
plt.figure(1)
plt.xlim(searchSpace[0][0]-gap,searchSpace[0][1]+gap)
plt.ylim(searchSpace[1][0]-gap,searchSpace[1][1]+gap)
pointArray = []

#----------------------------MAIN------------------------------#
t = time.time()
particlePosition, particleVelocity = init(step, maxParticle, searchSpace)
while(iteration < maxIteration and stop):
    for n in range(maxParticle):
        print("Iteration: " + str(iteration) + " Particle: " + str(n))
        particlePosition[n], particleVelocity[n] = cost(particlePosition[n], particleVelocity[n], \
                                                        angleLimit, angleStep, costRadius, pointArray)
        pointArray.append(particlePosition[n])
        plt.plot(particlePosition[n][0], particlePosition[n][1], 'b.')

    plt.pause(0.01)
    iteration += 1
    for i in range(len(lineList)):
        """Removes every line in the figure
        Still not sure why a simple FOR-loop won't work
        Have to manually pop the list
        """
        line, = lineList.pop()
        line.remove()

print("Point Array stored: " + str(len(pointArray)) + " Coordinate pairs")
print("Elapsed time: " + str(round(time.time() - t, 3)) + " seconds")
plt.figure(1)
plt.close()
plt.figure(2)
plt.xlim(searchSpace[0][0]-gap,searchSpace[0][1]+gap)
plt.ylim(searchSpace[1][0]-gap,searchSpace[1][1]+gap)
for i in range(len(pointArray)):
    plt.plot(pointArray[i][0], pointArray[i][1], 'r.')

plt.show()