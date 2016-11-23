# PSO_2D.py
# Shawn Goh
#
#################################

import os
import sys
import numpy as np
import random
import matplotlib.pyplot as plt
from math import *

# Namespace manipulation
rand = random.random()

def randomVector(range):
    """Return a float
    Float is chosen between the two elements of a input list.
    """
    return random.uniform(range[0], range[1])


def costFunction(currentPosition): # currentPosition = [x, y, z, ..., last-dimension]
    """Return f(x, y) as float
    Calculate the cost according to [x, y], where f(x, y) = blah blah.

    Changelog:
    1. Test: cos(x*y)*((x-0.2)^2-y^2)
    """
    x = currentPosition[0]
    y = currentPosition[1]
    # return (abs(currentPosition[0]**2.0 - currentPosition[1]**2.0))
    return (cos(x*y)*(x**2.0-y**2.0))


def updateVelocity(c1, c2, currentVelocity, currentPosition, particleBestPosition, globalBestPosition, maxVelocity):
    """Return [new X velocity, new Y velocity]
    Calculates axial velocity according to PSO velocity formula.

    Changelog:
    1. Negative velocity are now bound by maxVelocity.
    """
    xVelocity = currentVelocity[0] + \
                (c1*rand*(particleBestPosition[0] - currentPosition[0])) + \
                (c2*rand*(globalBestPosition[0] - currentPosition[0]))
    yVelocity = currentVelocity[1] + \
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


def updatePosition(currentPosition, currentVelocity):
    """Return [new X position, new Y position]
    Calculates the new axial position.
    """
    xPosition = currentPosition[0] + currentVelocity[0]
    yPosition = currentPosition[1] + currentVelocity[1]
    newPosition = [xPosition, yPosition]
    return newPosition


def createParticle(c1, c2, maxParticle, searchSpace, globalBestPosition, globalBestCost, maxVelocity):
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
        particleCurrentCost.append(costFunction(particleCurrentPosition[i]))
        particleBestPosition.append(particleCurrentPosition[i])
        particleBestCost.append(particleCurrentCost[i])

    globalBestCost = min(particleBestCost)
    globalBestPosition = particleBestPosition[particleBestCost.index(globalBestCost)]

    for i in range(maxParticle):
        particleVelocity.append(updateVelocity(c1, c2, initialVelocity, particleCurrentPosition[i],
                                               particleBestPosition[i], globalBestPosition, maxVelocity))
    return particleCurrentCost, particleBestCost, particleVelocity, \
           particleCurrentPosition, particleBestPosition, globalBestPosition, globalBestCost


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


# Parameters:
problemSize = 0 # Not used yet
searchSpace = [[-500.0,500.0], [-500.0, 500.0]] # [[x_range], [y_range]]
initialVelocity = [0.0,0.0]
maxIteration = 100
maxVelocity = 20
maxParticle = 40
populationSize = 50 # Not used yet
iteration = 0
stopCondition = 1 # Not used yet
mode = 'min' # optimize for max/min

c1 = 4
c2 = 4
globalBestPosition = [0,0]
globalBestCost = 1000000.0

# DEBUG: VARIABLES
debugBestPosition = []
debugBestCost = []
debugAllPosition = []
debugAllCost = []

# Main
# Particle creation
particleCurrentCost, particleBestCost, particleVelocity, \
particleCurrentPosition, particleBestPosition, globalBestPosition, globalBestCost \
= createParticle(c1, c2, maxParticle, searchSpace, globalBestPosition, globalBestCost, maxVelocity)

# Initialize plot
plt.figure()
plt.xlim(searchSpace[0][0],searchSpace[0][1])
plt.ylim(searchSpace[1][0],searchSpace[1][1])

# Iterate to maxIteration or stopCondition = False
while(iteration < maxIteration and stopCondition): # TODO: stopCondition as a function of cohesion

    # Iterate through all particles
    for n in range(maxParticle):
        print("iteration " + str(iteration) + " particle " + str(n))
        particleVelocity[n] = updateVelocity(c1, c2, particleVelocity[n], particleCurrentPosition[n], \
                                             particleBestPosition[n], globalBestPosition, maxVelocity)
        particleCurrentPosition[n] = updatePosition(particleCurrentPosition[n], particleVelocity[n])
        particleCurrentCost[n] = costFunction(particleCurrentPosition[n])
        particleBestPosition[n], particleBestCost[n] = updatePersonalBest(particleCurrentPosition[n], \
                                                                    particleCurrentCost[n], particleBestPosition[n], particleBestCost[n])
        debugAllPosition.append([])
        debugAllCost.append([])
        debugAllPosition[iteration].append(particleCurrentPosition[n])
        debugAllCost[iteration].append(particleCurrentCost[n])

    globalBestPosition, globalBestCost = updateGlobalBest(mode, particleBestPosition, particleBestCost, globalBestPosition, globalBestCost)
    debugBestPosition.append(globalBestPosition)
    debugBestCost.append(globalBestCost)

    iteration+=1


# Plot
# iteration = 1
for i in range(iteration):
    for j in range(maxParticle):
        plt.plot(debugAllPosition[i][j][0], debugAllPosition[i][j][1], 'b.')
    plt.plot(debugBestPosition[i][0], debugBestPosition[i][1], 'rp')
    # plt.savefig()

# print(debugBestPosition)
# for i in range(len(debugBestPosition)):
#     plt.plot(debugBestPosition[i][0], debugBestPosition[i][1], '.')

plt.show()

# DEBUG PRINT
# print(particleCurrentPosition[:2])
# print(particleCurrentCost[:2])
# print(particleBestPosition[:2])
# print(particleBestCost[:2])
# print(globalBestPosition)
# print(globalBestCost)
# print(particleVelocity[:2])

