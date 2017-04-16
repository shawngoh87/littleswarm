# File Name     :   swarm.py
# Created on    :   26/1/2017
# Author        :   Shawn Goh <shawngoh87@hotmail.com>

import matplotlib.pyplot as plt
import os, sys, numpy
import urllib.request
import random
import math

from matplotlib.patches import Rectangle
from pylab import figure, axes, pie, title, show
from time import gmtime, strftime
import pylab

"""
botStateDict:
- X,Y coordinates
- Bearing
- Percent coverage
- Role
envStateDict:
- Explored coordinates
- Tree coordinates
- Completed coordinates

Urgent:
-- Implement flocking with Simple Flocking in Fav bar
-- Bot collision check by Kwang Liang (if bot is surrounded by other bots)

Optimization:
Data structure for coordinate indexing
-- Sorting in server and Binary Search in bot (Sort the Dict after every push, then binary search)
"""
def dist(a,b):
    return math.hypot(a[0]-b[0], a[1]-b[1])
def getState():
    """Return all bots' state
    server = botStateDict
    """
    pass
def rotate(l, n):
    return l[n:] + l[:n]
def pushState(p, r, c, b, bot, botStateDict):
    """Push self state to server
    server = botStateDict
    """
    botStateDict["position"][bot] = p
    botStateDict["role"][bot] = r
    botStateDict["coverage"][bot] = c
    botStateDict["bearing"][bot] = b
    return botStateDict
def pushData(p, envStateDict):
    """Update envStateDict"""
    if p not in envStateDict["explored"]:
        envStateDict["explored"].append(p)
    return envStateDict
def checkWorkPath(position):
    pass
def snapDirection(dir):
    """Snap direction to multiples of 45"""
    dir = round(dir)
    if dir > 7:
        return 0
    else: return dir
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
def roleArbiter(bot):
    """Return role
    Things have not taken into account:
    Is this bot nearest to workable area?
    Does this bot has better exploration prospect than others?
    """
    if envStateDict["workable"] == []:
        return "explorer"
    if botStateDict["role"][bot] == "worker":
        if isCollide(botStateDict["position"][bot], botStateDict, bot):
            return "explorer"
        print("BOT: ", bot, " ROLE: ", botStateDict["role"][bot], " REMAIN")
        return "worker"
    else:
        maxWorker = math.floor(len(envStateDict["explored"])*maxBotCount/(gridCount**2))
        if botStateDict["role"].count("worker") < maxWorker:
            plantX = sum([x[0] for x in envStateDict["workable"]])
            plantY = sum([x[1] for x in envStateDict["workable"]])
            plantX = plantX/len(envStateDict["workable"])
            plantY = plantY/len(envStateDict["workable"])
            print("MEAN PLANT LOCATION ", plantX, plantY)
            if (1-(math.hypot(plantX-botStateDict["position"][bot][0], plantY-botStateDict["position"][bot][1])/math.hypot(gridCount*gridBoxSize, gridCount*gridBoxSize))**2) > random.random():
                return "worker"
            else:
                return "explorer"
        else: return "explorer"
        # workerPercent = botStateDict["role"].count("worker")/maxBotCount
        # treePercent = len(envStateDict["workable"])/len(envStateDict["explored"])
        # if workerPercent*copingConstant > treePercent: # Can improve on this
        #     return "explorer" # Can cope
        # else: return "worker" # Cannot cope
def isOutOfBound(position):
    x, y = position
    xmin, xmax, ymin, ymax = boundary # Global variable
    return x < xmin or x > xmax or y < ymin or y > ymax
def isCollide(position, botStateDict, bot):
    notMe = [botStateDict["position"][x] for x in range(maxBotCount) if x != bot]
    if position in notMe:
        return True
    else: return False
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
        7:[x+1  ,y-1],
        "stay":[x,y]
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
def chooseExplorePath(position, pathList):
    """Returns the next direction
    from a direction list 0-7
    """
    x, y = position
    c = {}
    for dir in pathList:
        nextPos = getNextPosition(position, dir)
        if isOutOfBound(nextPos):
            continue
        else: c[dir] = checkCoverage(nextPos)
    c = sorted(c.items(), key = lambda k:k[1])
    return c
def flock(position, botStateDict):
    """Return the flock angles
    Cohesion -- Long range attraction
    Alignment -- Average heading of neighbours
    Separation -- Short range repulsion (Not implemented, use bot collision instead)
    """
    x, y = position
    flockPos = []
    flockDir = []
    for index, [xx, yy] in enumerate(botStateDict["position"]):
        if abs(xx-x) <= flockRadius and abs(yy-y) <= flockRadius and (abs(xx-x) or abs(yy-y)): # within flockRadius but not origin
            flockPos.append([xx,yy])
            flockDir.append(botStateDict["bearing"][index])

    if flockPos == []: # No flockmate :(
        print("No flockmates :(")
        arr = [0,1,2,3,4,5,6,7,"stay"]
        random.shuffle(arr)
        return arr
    flockDir = [x for x in flockDir if isinstance(x, int)]

    # Cohesion
    meanX, meanY = 0, 0
    for each in flockPos:
        meanX += each[0]
        meanY += each[1]
    meanX = meanX / len(flockPos)
    meanY = meanY / len(flockPos)
    cohX = meanX - x
    cohY = meanY - y
    cohTheta = math.atan2(cohY, cohX)

    # Alignment
    lengthX, lengthY = 0, 0
    for each in flockDir:
        lengthX += math.cos(each*0.25*math.pi)
        lengthY += math.sin(each*0.25*math.pi)
    alignTheta = math.atan2(lengthY, lengthX)

    # Separation (Not implemented, use bot collision)

    # Total
    """dir = A*exploreDir + B*cohesionDir + C*separationDir + D*alignmentDir, A+B+C+D = 1"""
    lengthX = COH*math.cos(cohTheta*0.25*math.pi) + ALG*math.cos(alignTheta*0.25*math.pi)
    lengthY = COH*math.sin(cohTheta*0.25*math.pi) + ALG*math.sin(alignTheta*0.25*math.pi)
    finalTheta = math.atan2(lengthY, lengthX)
    if finalTheta < 0:
        finalTheta += 360

    # Original order
    allDir = [0, 1, 2, 3, 4, 5, 6, 7]

    # Rotated order
    closestDir = round(finalTheta/45)
    if closestDir == 8:
        closestDir = 0
    allDir = rotate(allDir, closestDir)

    # Execution Order
    dir = []
    for i in range(8):
        if i % 2 == 0:
            dir.append(allDir.pop(0))
        else:
            dir.append(allDir.pop())

    # Added "Stay" option as last resort
    dir.insert(0, "stay")
    return dir

def decide(p, explorePath, botStateDict, bot):
    # print("BOT: ", bot, " E: ", explorePath)
    if len(explorePath):
        dir = explorePath.pop()
        if isinstance(dir, tuple):# fix chooseExplorePath to return a ranked list instead of tuple
            dir = dir[0]
        newPosition = getNextPosition(p, dir)
        if not isOutOfBound(newPosition):
            if not isCollide(newPosition, botStateDict, bot):
                return dir, newPosition
            else: return decide(p, explorePath, botStateDict, bot)
        else: return decide(p, explorePath, botStateDict, bot)
    else: return decide(p, flock(p, botStateDict), botStateDict, bot) # Flock path is unranked

def calculate(bot, botStateDict, envStateDict):
    p, r, c, b = [botStateDict["position"][bot], botStateDict["role"][bot], botStateDict["coverage"][bot], botStateDict["bearing"][bot]]
    # Set new role
    newRole = roleArbiter(bot)
    print(newRole)
    if newRole == "explorer":
        # Set new explorer position
        pathList = checkExplorePath(p)
        explorePath = chooseExplorePath(p, pathList)
        # exploreTheta = math.atan2(math.sin(exploreDir*0.25*math.pi), math.cos(exploreDir*0.25*math.pi))
        # cohTheta, alignTheta = flock(p, botStateDict)
        # """dir = A*exploreDir + B*cohesionDir + C*separationDir + D*alignmentDir, A+B+C+D = 1"""
        # if cohTheta!=8 and alignTheta!= 8:
        #     lengthX = EXP*math.cos(exploreTheta*0.25*math.pi) + COH*math.cos(cohTheta*0.25*math.pi) + ALG*math.cos(alignTheta*0.25*math.pi)
        #     lengthY = EXP*math.sin(exploreTheta*0.25*math.pi) + COH*math.sin(cohTheta*0.25*math.pi) + ALG*math.sin(alignTheta*0.25*math.pi)
        #     finalTheta = math.atan2(lengthY, lengthX)
        #     if finalTheta < 0:
        #         finalTheta += 360
        #     # NEED TO CONVERT DIRECTION INTO SCALABLE VARIABLES (direction (0-7) cannot be weighted)
        #     # Maybe use unit vector of magnitude one, times with weight
        #     # Should expect vector instead? more accurate
        #     dir = snapDirection(finalTheta/45)
        # else: dir = exploreDir
        dir, newPosition = decide(p, explorePath, botStateDict, bot)
        # if dir != 8:
        #     newPosition = getNextPosition(p, dir)
        #     if not isOutOfBound(newPosition):
        #         if not isCollide(newPosition, botStateDict):
        #             pass
        #         else: newPosition = p
        #     else: newPosition = p
        # else: newPosition = p

        # if dir > 7: # No direction found
        #     dir = random.randint(0,7)
        #     newPosition = getNextPosition(p, dir)
        #     # if nextPosition is OOB, random again
        #     while isOutOfBound(newPosition) or isCollide(newPosition, botStateDict):
        #         dir = random.randint(0,7)
        #         if dir == 8:
        #             dir = "stay"
        #         newPosition = getNextPosition(p, dir)
        # else: # Valid direction found
        #     newPosition = getNextPosition(p, dir)
        #     # if nextPosition is OOB, remove from list and loop again
        #     while isOutOfBound(newPosition) or isCollide(newPosition, botStateDict):
        #         pathList.remove(dir)
        #         dir = chooseExplorePath(p, pathList)
        #         if dir == 8:
        #             dir = "stay"
        #         newPosition = getNextPosition(p, dir)
        newBearing = dir
        newCoverage = checkCoverage(newPosition)
        if newPosition in sample:
            if newPosition not in envStateDict["workable"] and newPosition not in envStateDict["completed"] :
                envStateDict["workable"].append(newPosition)
        return newPosition, newRole, newCoverage, newBearing

    elif newRole == "worker":
        # Set new worker position
        min = dist([gridCount*gridBoxSize, gridCount*gridBoxSize], [0,0])
        # print(envStateDict["workable"])
        for each in envStateDict["workable"]:
            if dist(each, p) <= min:
                min = dist(each,p)
                choose = each
        # print("CHOOSE: ", choose,"STATEDICT:", envStateDict["workable"])
        newCoverage = 0
        newBearing = 0
        if p[0]>choose[0]:
            if p[1]>choose[1]:
                newPosition = [p[0]-1,p[1]-1]
            elif p[1]==choose[1]:
                newPosition = [p[0]-1,p[1]]
            else: newPosition = [p[0]-1,p[1]+1]
        elif p[0]==choose[0]:
            if p[1]>choose[1]:
                newPosition = [p[0],p[1]-1]
            elif p[1]==choose[1]:
                newPosition = [p[0], p[1]]
                if isCollide(newPosition, botStateDict, bot):
                    newRole = "explorer"
                else:
                    newRole="explorer"
                    envStateDict["completed"].append(newPosition)
                    envStateDict["workable"].remove(newPosition)
            else: newPosition = [p[0],p[1]+1]
        else:
            if p[1]>choose[1]:
                newPosition = [p[0]+1,p[1]-1]
            elif p[1]==choose[1]:
                newPosition = [p[0]+1,p[1]]
            else: newPosition = [p[0]+1,p[1]+1]

        return newPosition, newRole, newCoverage, newBearing
    else: print("Invalid role.")




def init():
    botStateDict = { "position" :   [[0,0],[0,0],[0,0]],
                     "role"     :   ["explorer"]*maxBotCount,
                     "bearing"  :   [random.randint(0,7) for n in range(maxBotCount)], # Actually the last direction it travelled
                     "coverage" :   [0]*maxBotCount}
    envStateDict = { "explored" : botStateDict["position"][:], # list passed by reference dammit, use [:] #NEVERFORGET
                     "workable" : [],
                     "completed" : []}
    return botStateDict, envStateDict

#------------------------------------MAIN--------------------------------------#
global coverageLevel
global roleSwapThreshold
global maxBotCount
global copingConstant # Ability of a bot to cope with workable areas
global boundary
global flockRadius
global EXP, COH, ALG
global sample
sample = [[random.randint(0,4),random.randint(0,4)] for x in range(10)]
if [0,0]in sample:
    sample.remove([0,0])
EXP, COH, ALG = 1, 0.5, 0.5 # Must add up to 1 (Now only COH and ALG)

artist = []
flockRadius = 4
copingConstant = 1
coverageLevel = 2 # 1 = 3x3, 2 = 5x5 ... etc
gridBoxSize = 1
gridCount = 5
pauseInterval = 0.01
iteration = 0

boundary = [0, gridCount-1, 0, gridCount-1]
stopFlag = False
maxBotCount = 3
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

path = 'img/' + strftime("%H%M%S", gmtime())
if not os.path.exists(path):
    os.makedirs(path)
#-------------------------------LOOP------------------------------------------#
while(stopFlag == False):
    # try:
    iteration+=1
    for bot in range(maxBotCount):
        p, r, c, b = calculate(bot, botStateDict, envStateDict)
        # print("Bot ", bot, " P: ", p, " R: ", r, " C: ", c, " B: ", b)
        botStateDict = pushState(p, r, c, b, bot, botStateDict)
        envStateDict = pushData(p, envStateDict)
        print(envStateDict["completed"])

        # Simulation
        for each in envStateDict["explored"]:
            shade(each, "#e0e0eb")
        for each in envStateDict["workable"]:

            shade(each, "#8cf442")
        for each in envStateDict["completed"]:
            shade(each, "#f4be41")
        for each in sample:
            artist.append(plt.plot(each[0], each[1], "+", color="black"), )
        for index, each in enumerate(botStateDict["position"]):
            if botStateDict["role"][index] == "explorer":
                artist.append(plt.plot(each[0], each[1], ".", color=botColor[index]),)
            else: artist.append(plt.plot(each[0], each[1], "x", color=botColor[index]),)
        plt.pause(pauseInterval)

        pylab.savefig(path +'/'+ str(iteration)+'.png', bbox_inches='tight')
        for n in range(len(artist)):
            thingy, = artist.pop()
            thingy.remove()
        for each in envStateDict["explored"]:
            shade(each, "#ffffff")
        # Simulation
    # except: plt.show()