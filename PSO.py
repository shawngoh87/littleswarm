import os
import sys
import numpy as np
import random
import matplotlib.pyplot as plt

# Namespace manipulation
rand = random.random()


def randomVector(range): # 2D-vector only
    """
    Returns a random point in 'range',
    Returned value of point follows the dimension of 'range'.
    """
    return random.uniform(range[0], range[1])


def costFunction(currentPosition): # f(x) = x^2
    """
    Calculate the cost of the current position.
    May need to change into the circular shade formula thingy.
    """
    return currentPosition**2.0


def updateVelocity(c1, c2, currentVelocity, currentPosition, personalBestPosition, globalBestPosition):
    """
    Updates velocity based on positions
    May need to incorporate maximum velocity
    """
    newVelocity = currentVelocity + \
                  (c1*rand*(personalBestPosition - currentPosition)) + \
                  (c2*rand*(globalBestPosition - currentPosition))
    return newVelocity


def updatePosition(currentPosition, currentVelocity):
    newPosition = currentPosition + currentVelocity
    return newPosition


def createParticle(c1, c2, maxParticle, searchSpace, globalBestPosition, globalBestCost):
    particleCurrentCost = []
    particleBestCost = []
    particleVelocity = []
    particleCurrentPosition = []
    particleBestPosition = []
    for i in range(maxParticle):
        particleCurrentPosition.append(randomVector(searchSpace))
        particleCurrentCost.append(costFunction(particleCurrentPosition[i]))
        particleBestPosition.append(particleCurrentPosition[i])
        particleBestCost.append(particleCurrentCost[i])


    globalBestCost = min(particleBestCost)
    globalBestPosition = particleBestPosition[particleBestCost.index(globalBestCost)]


    for i in range(maxParticle):
        particleVelocity.append(updateVelocity(c1, c2, 0, particleCurrentPosition[i], particleBestPosition[i], globalBestPosition))
    return particleCurrentCost, particleBestCost, particleVelocity, \
           particleCurrentPosition, particleBestPosition, globalBestPosition, globalBestCost


def updateGlobalBest(mode, particleBestPosition, particleBestCost, globalBestPosition, globalBestCost):
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
    if particleCurrentCost < particleBestCost:
        particleBestCost = particleCurrentCost
        particleBestPosition = particleCurrentPosition
    else:
        pass
    return particleBestPosition, particleBestCost


# Parameters:
problemSize = 0
searchSpace = [-5.0,5.0]
maxIteration = 100
maxVelocity = 100
maxParticle = 40
populationSize = 50
iteration = 0
mode = 'min' # optimize for max/min

c1 = 2
c2 = 2
globalBestPosition = 1000
globalBestCost = 1000

# DEBUG
debugPlotPoints = []

# Main
particleCurrentCost, particleBestCost, particleVelocity, \
particleCurrentPosition, particleBestPosition, globalBestPosition, globalBestCost \
= createParticle(c1, c2, maxParticle, searchSpace, globalBestPosition, globalBestCost)
while(iteration < maxIteration):
    for n in range(maxParticle):
        print("iteration " + str(iteration) + " particle " + str(n))
        particleVelocity[n] = updateVelocity(c1, c2, particleVelocity[n], particleCurrentPosition[n], \
                                             particleBestPosition[n], globalBestPosition)
        particleCurrentPosition[n] = updatePosition(particleCurrentPosition[n], particleVelocity[n])
        particleCurrentCost[n] = costFunction(particleCurrentPosition[n])
        particleBestPosition[n], particleBestCost[n] = updatePersonalBest(particleCurrentPosition[n], \
                                                                    particleCurrentCost[n], particleBestPosition[n], particleBestCost[n])


    globalBestPosition, globalBestCost = updateGlobalBest(mode, particleBestPosition, particleBestCost, globalBestPosition, globalBestCost)
    debugPlotPoints.append(globalBestPosition)
    # print(particleCurrentPosition[:5])
    # print(globalBestPosition)
    # print(globalBestCost)
    iteration+=1

# DEBUG
plt.figure(1)
for i in range(len(debugPlotPoints)):
    plt.plot(i, abs(debugPlotPoints[i]), '.')

plt.show()

# DEBUG PRINT
'''print(particleCurrentPosition[:5])
print(particleCurrentCost[:5])
print(particleBestPosition[:5])
print(particleBestCost[:5])
print(globalBestPosition)
print(globalBestCost)
print(particleVelocity[:5])'''

