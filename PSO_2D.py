# File Name     :   PSO_2D.py
# Created on    :   21/11/2016
# Author        :   Shawn Goh <shawngoh87@hotmail.com>

"""Current issues:
1.  Swarm just won't explore.
    -- Found out that the particleBest Cost/Position is stagnant at full Zeros.
    -- Maybe
2.  Mosquito-Attack Algorithm?
    -- Run around a set of points.
"""

import os
import sys
import numpy as np
import random
import matplotlib.pyplot as plt
from math import *

# Namespace manipulation
rand = random.random()

#----------------------------FUNCTIONS------------------------------#
def randomVector(range):
    """Return a float
    Float is chosen between the two elements of a input list.
    """
    return random.uniform(range[0], range[1])


def dist(pointA, pointB):
    """Return Euclidean distance between A and B"""
    return np.linalg.norm(np.array(pointA) - np.array(pointB))


def rotate(vector, times=1):
    """Return [-b,a] from [a,b]
    Rotates the vector 90 degrees CCW.
    Can be repeated recursively with a second argument.
    EG: rotate([3,4], 2) = [-3,-4] , rotated twice.
    """
    if times > 0:
        return rotate([-vector[1], vector[0]], times-1)
    else: return [vector[0], vector[1]]


def saturationCheck(thresholdCount, currentPosition, currentVelocity, currentCost):
    """Return new position

    """
    # print(currentPosition)
    # print(currentVelocity)
    px, py = currentPosition[0], currentPosition[1]
    front = currentVelocity
    left = rotate(currentVelocity)
    back = rotate(currentVelocity, 2)
    right = rotate(currentVelocity, 3)


    if currentCost > thresholdCount:
        # print(left)
        return [left[0] + px, left[1] + py]
    else:
        # print(front)
        return [front[0] + px, front[1] + py]



def costFunction(costBias, costRadius, currentPosition, pointArray):
    """Return cost as float
    Calculate the cost according to [x, y], where f(x, y) = blah blah.

    Idea: Take previous point counts into account so the robots leaves a "trail of shadow"

    Changelog:
    1. Test: cos(x*y)*((x-0.2)^2-y^2)
    2. Changed cost to the sum of points within a certain radius costRadius.
        Added costBias for the lulz.
    """
    count = 0
    for point in pointArray:
        if dist(currentPosition, point) < costRadius:
            count+=1
        else: pass

    cost = costBias*count
    return cost


def updateVelocity(c1, c2, currentVelocity, currentPosition, particleBestPosition, globalBestPosition, maxVelocity, inertiaFactor):
    """Return [new X velocity, new Y velocity]
    Calculates axial velocity according to PSO velocity formula.

    Idea: Change velocity direction if saturates, compare current with best particle position,
            if within a certain radius then reflect

    Changelog:
    1. Negative velocity are now bound by maxVelocity.
    """
    xVelocity = inertiaFactor*currentVelocity[0] + \
                (c1 * rand * (particleBestPosition[0] - currentPosition[0])) + \
                (c2 * rand * (globalBestPosition[0] - currentPosition[0]))
    yVelocity = inertiaFactor*currentVelocity[1] + \
                (c1 * rand * (particleBestPosition[1] - currentPosition[1])) + \
                (c2 * rand * (globalBestPosition[1] - currentPosition[1]))
    if abs(xVelocity) >= maxVelocity:
        if xVelocity > 0:
            xVelocity = maxVelocity
        else: xVelocity = -maxVelocity
    if abs(yVelocity) >= maxVelocity:
        if yVelocity > 0:
            yVelocity = maxVelocity
        else: yVelocity = -maxVelocity
    newVelocity = [xVelocity, yVelocity]
    return newVelocity


def updatePosition(currentPosition, currentVelocity, searchSpace):
    """Return [new X position, new Y position]
    Calculates the new axial position.

    Idea: THE POINTS ARE NOT BOUND!!!! WENT OUTSIDE THE PERIMETER

    Changelog:
    1. Added searchSpace to the mix, so the points are bound.
    """
    xPosition = currentPosition[0] + currentVelocity[0]
    yPosition = currentPosition[1] + currentVelocity[1]
    if xPosition < min(searchSpace[0]):
        xPosition = min(searchSpace[0])
    elif xPosition > max(searchSpace[0]):
        xPosition = max(searchSpace[0])
    if yPosition < min(searchSpace[1]):
        yPosition = min(searchSpace[1])
    elif yPosition > max(searchSpace[1]):
        yPosition = max(searchSpace[1])
    newPosition = [xPosition, yPosition]
    return newPosition


def updateGlobalBest(mode, particleBestPosition, particleBestCost, globalBestPosition, globalBestCost):
    """Return globalBestPosition and globalBestCost
    Based on mode, swaps the global best with particle best
    min     -- Used for convergences at 0
    max     -- Used for convergences at |infinity|

    Idea: three modes, -ve +ve 0
    """
    if mode == 'min':
        temp = min(particleBestCost)
        if temp < globalBestCost:
            globalBestCost = temp
            globalBestPosition = particleBestPosition[particleBestCost.index(temp)]
        else: pass
    elif mode == 'max':
        temp = max(particleBestPosition)
        if temp > globalBestPosition:
            globalBestCost = temp
            globalBestPosition = particleBestPosition[particleBestCost.index(temp)]
        else: pass
    else:
        print("Mode is neither min nor max")
        pass
    return globalBestPosition, globalBestCost


def updatePersonalBest(particleCurrentPosition, particleCurrentCost, particleBestPosition, particleBestCost):
    """Return particleBestPosition and particleBestCost
    Mode-less swap, defaulting to minimum.

    Idea: add mode, sync with updateGlobalBest
    """
    if particleCurrentCost < particleBestCost:
        particleBestCost = particleCurrentCost
        particleBestPosition = particleCurrentPosition
    else:
        pass
    return particleBestPosition, particleBestCost


def createParticle(c1, c2, maxParticle, searchSpace, globalBestPosition, globalBestCost, maxVelocity, inertiaFactor):
    """Return a bunch of initialized lists/values.
    Creates the following:
    particleCurrentPosition -- Every particle's current position in [x, y]
    particleCurrentCost     -- Every particle's current cost
    particleBestPosition    -- Every particle's past best position, based on the particle's best cost
    particleBestCost        -- Every particle's past best cost
    particleVelocity        -- Every particle's initial velocity
    globalBestPosition      -- Global best position based on global best cost
    globalBestCost          -- Global best cost
    """
    particleCurrentPosition = []
    particleCurrentCost = []
    particleBestPosition = []
    particleBestCost = []
    particleVelocity = []
    for i in range(maxParticle):
        particleCurrentPosition.append([randomVector(searchSpace[0]), randomVector(searchSpace[1])])
        particleCurrentCost.append(costFunction(costBias, costRadius, particleCurrentPosition[i], pointArray))
        particleBestPosition.append(particleCurrentPosition[i])
        particleBestCost.append(particleCurrentCost[i])

    globalBestCost = min(particleBestCost)
    globalBestPosition = particleBestPosition[particleBestCost.index(globalBestCost)]

    for i in range(maxParticle):
        particleVelocity.append(updateVelocity(c1, c2, initialVelocity, particleCurrentPosition[i],
                                               particleBestPosition[i], globalBestPosition, maxVelocity,
                                               inertiaFactor))
    return particleCurrentCost, particleBestCost, particleVelocity, \
           particleCurrentPosition, particleBestPosition, globalBestPosition, globalBestCost

#----------------------------CONSTANTS------------------------------#
# Parameters:
length = 1000.0
searchSpace = [[-(length/2),(length/2)], [-(length/2), (length/2)]] # [[x_range], [y_range]]\
initialVelocity = [0.0,0.0]
globalBestPosition = [0,0]
globalBestCost = 1000000.0
maxIteration = 1000
maxVelocity = 7
maxParticle = 5
iteration = 0
stopCondition = True # Not used yet
mode = 'min' # optimize for max/min
gap = 50

# Tweaks
c1 = 2.0 # personal best bias
c2 = 2.0 # global best bias
inertiaFactor = 1.0 # 0.5 < x < 2.0
costBias = 1
individualRadius = 10.0
costRadius = 50.0 # costRadius >> individualRadius
thresholdCount = ((costRadius/individualRadius)**2)*pi/4

# DEBUG: VARIABLES
debugBestPosition = []
debugBestCost = []
debugAllPosition = []
debugAllCost = []
pointArray = []

#----------------------------MAIN------------------------------#
# Particle creation
particleCurrentCost, particleBestCost, particleVelocity, \
particleCurrentPosition, particleBestPosition, globalBestPosition, globalBestCost \
= createParticle(c1, c2, maxParticle, searchSpace, globalBestPosition, globalBestCost, maxVelocity, inertiaFactor)

# Initialize plot
plt.figure()
plt.xlim(searchSpace[0][0]-gap,searchSpace[0][1]+gap)
plt.ylim(searchSpace[1][0]-gap,searchSpace[1][1]+gap)

# Iterate to maxIteration or stopCondition = False
while(iteration < maxIteration and stopCondition): # TODO: stopCondition as a function of cohesion/area coverage

    # Iterate through all particles
    for n in range(maxParticle):
        # print("iteration " + str(iteration) + " particle " + str(n))
        particleVelocity[n] = updateVelocity(c1, c2, particleVelocity[n], particleCurrentPosition[n],
                                             particleBestPosition[n], globalBestPosition, maxVelocity, inertiaFactor)
        particleCurrentPosition[n] = saturationCheck(thresholdCount, particleCurrentPosition[n], \
                                                     particleVelocity[n], particleCurrentCost[n])
        # particleCurrentPosition[n] = updatePosition(particleCurrentPosition[n], particleVelocity[n], searchSpace)
        particleCurrentCost[n] = costFunction(costBias, costRadius, particleCurrentPosition[n], pointArray)
        particleBestPosition[n], particleBestCost[n] = updatePersonalBest(particleCurrentPosition[n], \
                                                                    particleCurrentCost[n], particleBestPosition[n], particleBestCost[n])

        plt.plot(particleCurrentPosition[n][0], particleCurrentPosition[n][1], 'b.')
        pointArray.append(particleCurrentPosition[n])

    print(particleBestCost)
    print(particleBestPosition)
    plt.pause(0.001)
    globalBestPosition, globalBestCost = updateGlobalBest(mode, particleBestPosition, particleBestCost, globalBestPosition, globalBestCost)
    print(len(pointArray))

    iteration+=1
    # DEBUG FOR MOVEMENT
    if iteration%5 == 0:
        plt.clf()
        plt.xlim(searchSpace[0][0] - gap, searchSpace[0][1] + gap)
        plt.ylim(searchSpace[1][0] - gap, searchSpace[1][1] + gap)
        # if iteration%50==0:
        #     pointArray = []


