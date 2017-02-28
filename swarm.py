# File Name     :   swarm.py
# Created on    :   26/1/2017
# Author        :   Shawn Goh <shawngoh87@hotmail.com>

import matplotlib.pyplot as plt
import os, sys, numpy
import urllib.request
import random
from matplotlib.patches import Rectangle

"""
botState:
- X,Y coordinates
- Bearing (Theta)
- Percent coverage
- Role

groundState
- X,Y coordinates
- Temperature
- Altitude
- Trees?
"""

def getState():
    """Return all bots' state
    server = botStateDict
    """
    pass
def pushState(p, r, c, bot, botStateDict):
    """Push self state to server
    server = botStateDict
    """
    botStateDict["position"][bot] = p
    botStateDict["role"][bot] = r
    botStateDict["coverage"][bot] = c
    return botStateDict
def roleArbiter(bot):
    """Return role
    Things have not taken into account:
    Is this bot nearest to workable area?
    Does this bot has better exploration prospect than others?
    """
    if envStateDict["workable"] == []:
        return "explorer"
    else:
        workerPercent = botStateDict["role"].count("worker")/maxBotCount
        treePercent = len(envStateDict["workable"])/len(envStateDict["explored"])
        if workerPercent*copingConstant > treePercent: # Can improve on this
            return "explorer" # Can cope
        else: return "worker" # Cannot cope

def isOutOfBound(position):
    x, y = position
    xmin, xmax, ymin, ymax = boundary # Global variable
    return x < xmin or x > xmax or y < ymin or y > ymax
def shade(position, color):
    x, y = position
    currentAxis = plt.gca()
    currentAxis.add_patch(Rectangle((x-0.5, y-0.5), 1, 1, facecolor=color))

def checkCoverage(position):
    """Return coverage percentage in float
    Environmental states taken from server:envStateDict
    """
    x, y = position
    count = 0
    totalCount = (coverageLevel*2+1)**2 - 1
    for xx in range(x-coverageLevel,x+coverageLevel+1):
        for yy in range(y-coverageLevel,y+coverageLevel+1):
            if [xx,yy] != [x,y]: # Exclude origin
                if [xx,yy] in envStateDict["explored"]:
                    count+=1

    return count/totalCount

def getNextPosition(position, dir):
    """Return next position
    Use this function to replace global dirDict
    """
    x, y = position
    dirDict = {
        0:[x+1  ,y  ],
        1:[x+1  ,y+1],
        2:[x    ,y+1],
        3:[x-1  ,y+1],
        4:[x-1  ,y  ],
        5:[x-1  ,y-1],
        6:[x    ,y-1],
        7:[x+1  ,y-1]
    }
    return dirDict[dir]

def checkExplorePath(position):
    """Returns available directions
    Bearing: 0 - 7 starting from 3 o'clock anti-clockwise
    """
    emptyPath = []
    x, y = position
    dirDict ={
        (x + 1, y): 0,
        (x + 1, y + 1): 1,
        (x, y + 1): 2,
        (x - 1, y + 1): 3,
        (x - 1, y): 4,
        (x - 1, y - 1): 5,
        (x, y - 1): 6,
        (x + 1, y - 1): 7
    }
    for xx in range(x-1, x+2):
        for yy in range(y-1, y+2):
            if [xx,yy] != [x,y]:
                if [xx,yy] not in envStateDict["explored"]:
                    emptyPath.append(dirDict[(xx,yy)])
    return emptyPath

def checkWorkPath(position):
    pass

def chooseExplorePath(position, pathList):
    """Returns the next direction
    from a direction list 0-7
    -- what if empty list?
    -- what if ??
    """
    x, y = position
    c = {}
    for dir in pathList:
        c[dir] = checkCoverage(getNextPosition(position, dir))
    c = sorted(c.items(), key = lambda k:k[1])
    if len(c) == 0:
        return None
    else: return c[-1][0] # Return the one with the most coverage

def calculate(bot, botStateDict, envStateDict):
    p, r, c = [botStateDict["position"][bot], botStateDict["role"][bot], botStateDict["coverage"][bot]]
    # Set new role
    newRole = roleArbiter(bot)
    if newRole == "explorer":
        # Set new explorer position
        pathList = checkExplorePath(p)
        dir = chooseExplorePath(p, pathList) # Improve this to deterministic, what if nowhere to go?
        if dir is None:
            print("i am in none")
            dir = random.randint(0,7)
            newPosition = getNextPosition(p, dir)
            while isOutOfBound(newPosition):
                dir = random.randint(0,7)
                newPosition = getNextPosition(p, dir)
        else:
            newPosition = getNextPosition(p, dir)
            while isOutOfBound(newPosition):
                pathList.remove(dir)
                dir = chooseExplorePath(p, pathList)
                newPosition = getNextPosition(p, dir)
            newCoverage = checkCoverage(newPosition)

    elif newRole == "worker":
        # Set new worker position
        path = checkWorkPath(p)
    else: print("Invalid role.")

    return newPosition, newRole, newCoverage

def pushData(p, envStateDict):
    """Update envStateDict"""
    if p not in envStateDict["explored"]:
        envStateDict["explored"].append(p)
    return envStateDict

def init():
    botStateDict = { "position" :   [[5,5],[5,6]],#[1,0],[1,1],[2,1]],
                     "role"     :   ["explorer"]*maxBotCount,
                     "coverage" :   [0]*maxBotCount}
    envStateDict = { "explored" : botStateDict["position"][:], # Fucking list passed by reference dammit, use [:] #NEVERFORGET
                     "workable" : [],
                     "complete" : []}
    return botStateDict, envStateDict

#------------------------------------MAIN--------------------------------------#
global coverageLevel
global roleSwapThreshold
global maxBotCount
global copingConstant # Ability of a bot to cope with workable areas
global boundary


artist = []
copingConstant = 1
coverageLevel = 1 # 1 = 3x3, 2 = 5x5 ... etc
gridBoxSize = 1
gridCount = 10
pauseInterval = 0.5
boundary = [0, gridCount-1, 0, gridCount-1]
stopFlag = False
maxBotCount = 2
roleSwapThreshold = 0.5
botColor = {0:"red",1:"yellow",2:"blue",3:"green",4:"purple"}

fig = plt.figure(1)
[plt.plot([x/10,x/10], [-0.5,9.5], lw=1, color='grey') for x in range(-5, gridCount*10+5, 10)]
[plt.plot([-0.5,9.5], [y/10,y/10], lw=1, color='grey') for y in range(-5, gridCount*10+5, 10)]
plt.xlim(-0.5, gridCount-0.5)
plt.ylim(-0.5, gridCount-0.5)

#--------------------------------INIT----------------------------------------#
botStateDict, envStateDict = init()
for i in range(maxBotCount):
    botStateDict["coverage"][i] = checkCoverage(botStateDict["position"][i])

#-------------------------------LOOP------------------------------------------#
while(stopFlag == False):
    for bot in range(maxBotCount):
        print("Bot number: " + str(bot))
        # print(envStateDict)
        p, r, c = calculate(bot, botStateDict, envStateDict)

        botStateDict = pushState(p, r, c, bot, botStateDict)
        envStateDict = pushData(p, envStateDict)

        for each in envStateDict["explored"]:
            shade(each, "#e0e0eb")
        for index, each in enumerate(botStateDict["position"]):
            artist.append(plt.plot(each[0], each[1], ".", color=botColor[index]),)
        plt.pause(pauseInterval)
        for n in range(len(artist)):
            thingy, = artist.pop()
            thingy.remove()
        for each in envStateDict["explored"]:
            shade(each, "#ffffff")

