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


def costFunction(currentPosition): # currentPosition = [x, y, z, ..., last-dimension]
    return (currentPosition[0]**2.0 + currentPosition[1]**2.0)


def updateVelocity(c1, c2, currentVelocity, currentPosition, particleBestPosition, globalBestPosition, maxVelocity):
    """
    Updates velocity based on positions
    May need to incorporate maximum velocity
    """
    xVelocity = currentVelocity[0] + \
                (c1*rand*(particleBestPosition[0] - currentPosition[0])) + \
                (c2*rand*(globalBestPosition[0] - currentPosition[0]))
    yVelocity = currentVelocity[1] + \
                (c1 * rand * (particleBestPosition[1] - currentPosition[1])) + \
                (c2 * rand * (globalBestPosition[1] - currentPosition[1]))
    if xVelocity > maxVelocity:
        xVelocity = maxVelocity
    else: pass
    if yVelocity > maxVelocity:
        yVelocity = maxVelocity
    else: pass
    newVelocity = [xVelocity, yVelocity]
    #newVelocity = currentVelocity + \
                  #(c1*rand*(particleBestPosition - currentPosition)) + \
                  #(c2*rand*(globalBestPosition - currentPosition))
    return newVelocity


def updatePosition(currentPosition, currentVelocity):
    xPosition = currentPosition[0] + currentVelocity[0]
    yPosition = currentPosition[1] + currentVelocity[1]
    newPosition = [xPosition, yPosition]
    return newPosition


def createParticle(c1, c2, maxParticle, searchSpace, globalBestPosition, globalBestCost, maxVelocity):
    particleCurrentCost = []
    particleBestCost = []
    particleVelocity = []
    particleCurrentPosition = []
    particleBestPosition = []
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
searchSpace = [[-500.0,500.0], [-500.0, 500.0]] # [[x_range], [y_range]]
initialVelocity = [0,0]
maxIteration = 100
maxVelocity = 20
maxParticle = 40
populationSize = 50
iteration = 0
mode = 'min' # optimize for max/min

c1 = 2
c2 = 2
globalBestPosition = [1000.0,1000.0]
globalBestCost = 1000.0

# DEBUG
debugPlotPoints = []

# Main
particleCurrentCost, particleBestCost, particleVelocity, \
particleCurrentPosition, particleBestPosition, globalBestPosition, globalBestCost \
= createParticle(c1, c2, maxParticle, searchSpace, globalBestPosition, globalBestCost, maxVelocity)
while(iteration < maxIteration):
    for n in range(maxParticle):
        print("iteration " + str(iteration) + " particle " + str(n))
        particleVelocity[n] = updateVelocity(c1, c2, particleVelocity[n], particleCurrentPosition[n], \
                                             particleBestPosition[n], globalBestPosition, maxVelocity)
        particleCurrentPosition[n] = updatePosition(particleCurrentPosition[n], particleVelocity[n])
        particleCurrentCost[n] = costFunction(particleCurrentPosition[n])
        particleBestPosition[n], particleBestCost[n] = updatePersonalBest(particleCurrentPosition[n], \
                                                                    particleCurrentCost[n], particleBestPosition[n], particleBestCost[n])

    globalBestPosition, globalBestCost = updateGlobalBest(mode, particleBestPosition, particleBestCost, globalBestPosition, globalBestCost)
    print(str(globalBestPosition) + str(globalBestCost))
    debugPlotPoints.append(globalBestPosition)
    # print(particleCurrentPosition[:5])
    # print(globalBestPosition)
    # print(globalBestCost)
    iteration+=1


# DEBUG
plt.figure(1)
# plt.xlim(-50,50)
# plt.ylim(-50,50)
print(debugPlotPoints)
for i in range(len(debugPlotPoints)):
    plt.plot(abs(debugPlotPoints[i][0]), abs(debugPlotPoints[i][1]), '.')

plt.show()

# DEBUG PRINT
# print(particleCurrentPosition[:2])
# print(particleCurrentCost[:2])
# print(particleBestPosition[:2])
# print(particleBestCost[:2])
# print(globalBestPosition)
# print(globalBestCost)
# print(particleVelocity[:2])

