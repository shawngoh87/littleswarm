# File Name     :   PSO_local.py
# Created on    :   25/11/2016
# Author        :   Shawn Goh <shawngoh87@hotmail.com>

"""
This is a simplified version of PSO, emphasizing:
-- Direction of each swarm unit.
-- Locality, therefore updateBest is removed, and costs are calculated based on local inputs.
-- Trail left behind by each swarm unit.
-- A constantly arranged and trimmed pointArray buffer.
-- Scalability, adding more bots doesn't affect the algorithm as it based on local rules.

Idea:
1.  Add local bias for points made for "trail"
"""
#-----------------------IMPORTS & NAMESPACE-------------------------#
import os, sys
import random
import numpy as np
import matplotlib.pyplot as plt
from math import *



#----------------------------FUNCTIONS------------------------------#
def dist(pointA, pointB):
    """Return Euclidean distance between pointA and pointB"""
    return np.linalg.norm(np.array(pointA) - np.array(pointB))

def randomVector(range):
    """Return a float between range"""
    return random.uniform(range[0], range[1])

def rotate(vector, degree):
    """Return rotated vector"""
    x, y = vector
    degree = degree/180 *pi
    return [x*cos(degree) - y*sin(degree), x*sin(degree) + y*cos(degree)]

def roulette(array):
    """Randomly choose an element, decreasing weight across the array"""
    numel = len(array)


def cost(position, velocity, angleLimit, angleStep, costRadius, pointArray):
    """Return newPosition and newVelocity"""
    count = 0
    costArray = []
    px, py = position
    for angle in range(-angleLimit, angleLimit + angleStep, angleStep):
        vx, vy = rotate(velocity, angle)
        cpoint = px + vx, py + vy
        for point in pointArray:
            if dist(point, cpoint) < costRadius:
                count += 1
        costArray.append([angle, count])
    costArray = sorted(costArray, key = lambda a:a[1])
    # Roulette wheel!!!! For BIASED RANDOMING

    temp = floor(random.random()*(len(costArray)-1))
    medianAngle = costArray[temp][0] # Need tweaks
    vx, vy = rotate(velocity, medianAngle)
    return [px + vx, py + vy], [vx, vy]

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
stop = 1
gap = 50
step = 20
iteration = 0
maxParticle = 5
maxIteration = 100
costRadius = 50
angleLimit = 60
angleStep = 10
length = 1000.0
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
        particlePosition[n], particleVelocity[n] = cost(particlePosition[n], particleVelocity[n], angleLimit, angleStep, costRadius, pointArray)
        pointArray.append(particlePosition[n])
        plt.plot(particlePosition[n][0], particlePosition[n][1], 'b.')
    plt.pause(0.001)
    iteration += 1

plt.figure(1)
plt.close()
plt.figure(2)
plt.xlim(searchSpace[0][0]-gap,searchSpace[0][1]+gap)
plt.ylim(searchSpace[1][0]-gap,searchSpace[1][1]+gap)
for i in range(len(pointArray)):
    plt.plot(pointArray[i][0], pointArray[i][1], 'r.')

plt.show()