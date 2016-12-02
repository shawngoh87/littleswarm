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
3.  In directional checking, use box/really-slim-ellipse instead of circle.
    --  Or a series of small circles!

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
def dist(pointA, pointB):
    """Return Euclidean distance between pointA and pointB"""
    return np.linalg.norm(np.array(pointA) - np.array(pointB))
def sumAscend(length, degree):
    """Return 1+2+3+...+length"""
    if length > 1:
        return length ** degree + sumAscend(length - 1, degree)
    else:
        return length
def randomVector(range):
    """Return a float between range"""
    return random.uniform(range[0], range[1])
def rotate(vector, degree):
    """Return rotated vector"""
    x, y = vector
    degree = degree/180 *pi
    return [x*cos(degree) - y*sin(degree), x*sin(degree) + y*cos(degree)]
def roulette(array, degree):
    """Randomly choose an element, decreasing weight across the array"""
    numel = len(array)
    rand = random.random()*sumAscend(numel, degree)
    for i in range(len(array)):
        rand -= numel**degree
        numel -= 1
        if rand <= 0:
            return array[len(array) - numel - 1]
def add2DVectors(vec1, vec2):
    return [vec1[0] + vec2[0], vec1[1] + vec2[1]]
def outOfBound(point):
    """If out-of-bound, return True"""
    x, y = point
    xMax, yMax = searchSpace[0], searchSpace[1]
    if (x <= xMax[0] or x >= xMax[1]) or (y <= yMax[0] or y >= yMax[1]):
        return True
    else: return False
def checkForward(cpoint, point, mode='Ellipse'):
    if mode == 'Ellipse':
        bigRadius = step
        smallRadius = step/ellipseRatio
        h, k = cpoint
        x, y = point

        if (((x-h)**2)/bigRadius + ((y-k)**2)/smallRadius) <= 1:
            return True
        else: return False

    elif mode == 'Circle':
        print("Dev is pissed with circles.")
def onLeftSide(linePointA, linePointB, point):
    """Return True if the point is on the left side of the line"""
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
def inPolygon(point, *args):
    """Return True if the point is inside a convex polygon.
    point -- Target point.
    *args -- Arbitrary number of vertices, in pairs of [x, y]
    """
    prev = args[-1]
    for vertex in args:
        if onLeftSide(vertex, prev, point) == False:
            return False
        prev = vertex
    return True

def cost(position, velocity, angleLimit, angleStep, costRadius, pointArray):
    """Return newPosition and newVelocity"""
    costArray = []
    px, py = position
    for angle in range(-angleLimit, angleLimit + angleStep, angleStep):
        count = 0
        pointMid = rotate(velocity, angle)
        pointLeft = rotate(pointMid, angleStep / 2)
        pointRight = rotate(pointMid, -angleStep / 2)
        cMid = add2DVectors(position, pointMid)
        cLeft = add2DVectors(position, pointLeft)
        cRight = add2DVectors(position, pointRight)
        plt.plot([cLeft[0], cRight[0]], [cLeft[1], cRight[1]], color='k', lw=2)
        plt.plot([cLeft[0], px], [cLeft[1], py], color='k', lw=2)
        plt.plot([cRight[0], px], [cRight[1], py], color='k', lw=2)
        for index, point in enumerate(pointArray):
            if inPolygon(point, pointLeft, pointRight, [px, py]):
                count += 1
        costArray.append([angle, count])
    costArray = sorted(costArray, key = lambda a:a[1])
    print(costArray)
    medianAngle = roulette(costArray, rouletteDegree)[0]
    if medianAngle > 0:
        counter[0] += 1
    elif medianAngle == 0:
        counter[1] += 1
    else: counter[2] += 1
    vx, vy = rotate(velocity, medianAngle)
    cpoint = [px + vx, py + vy]
    while(outOfBound(cpoint)):
        temp = random.random() * 360 # Reflects when boundary hit
        vx, vy = rotate([vx, vy], temp)
        cpoint = [px + vx, py + vy]
    rand = random.uniform(0.5, 1.0) # Prevent clustering of resonant steps
    return [px + rand*vx, py + rand*vy], [vx, vy]

def init(step, maxParticle, searchSpace):
    """Return position and velocity array of size(maxParticle)"""
    particlePosition = []
    particleVelocity = []
    for i in range(maxParticle):
        particlePosition.append([randomVector(searchSpace[0]), randomVector(searchSpace[1])])
        temp = random.random()*360
        particleVelocity.append(rotate([step, step], temp))
    return particlePosition, particleVelocity

#----------------------------CONSTANTS------------------------------#
global rouletteDegree
global counter
global searchSpace
global step
global ellipseRatio
ellipseRatio = 5
rouletteDegree = 5.0
counter = [0]*3
stop = 1
gap = 50
step = 20
iteration = 0
maxParticle = 5
maxIteration = 100
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
particlePosition, particleVelocity = init(step, maxParticle, searchSpace)
while(iteration < maxIteration and stop):
    for n in range(maxParticle):
        print("Iteration: " + str(iteration) + " Particle: " + str(n))
        particlePosition[n], particleVelocity[n] = cost(particlePosition[n], particleVelocity[n], \
                                                        angleLimit, angleStep, costRadius, pointArray)
        pointArray.append(particlePosition[n])
        plt.plot(particlePosition[n][0], particlePosition[n][1], 'b.')
    plt.pause(0.001)
    iteration += 1
    if iteration%5 == 0:
        plt.clf()
        plt.xlim(searchSpace[0][0] - gap, searchSpace[0][1] + gap)
        plt.ylim(searchSpace[1][0] - gap, searchSpace[1][1] + gap)


print("Point Array stored: " + str(len(pointArray)) + " Coordinate pairs")
print(counter)
plt.figure(1)
plt.close()
plt.figure(2)
plt.xlim(searchSpace[0][0]-gap,searchSpace[0][1]+gap)
plt.ylim(searchSpace[1][0]-gap,searchSpace[1][1]+gap)
for i in range(len(pointArray)):
    plt.plot(pointArray[i][0], pointArray[i][1], 'r.')

plt.show()