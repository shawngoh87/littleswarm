# File Name     :   swarm.py
# Created on    :   26/1/2017
# Author        :   Shawn Goh <shawngoh87@hotmail.com>

import matplotlib.pyplot as plt
import os, sys, numpy
import urllib.request

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
    return

def pushState():
    """Push self state to server
    server = botStateDict
    """

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



def calculate(bot):
    p, r, c = [botStateDict["position"][bot], botStateDict["role"][bot], botStateDict["coverage"][bot]]
    if c < roleSwapThreshold:
        pass
    return newPosition, newRole, newCoverage

def pushData():
    temperature = getTemp()
    altitude = getAltitude()
    x, y = getPos()
    state = [x, y, temperature, altitude]
    # TODO: POST request to server
    # return 0 if okay, 1 if not.
    pass
# def getPos():
#
#     return x, y
#     pass
# def getTemp(): # Get self current temperature
#     pass
# def getAltitude(): # Get self current altitude
#     pass

def init(): # Initialization
    botStateDict = { "position" :   [[0,0],[0,1],[1,0],[1,1],[2,1]],
                     "role"     :   ["explorer","explorer","explorer","explorer","explorer"],
                     "coverage" :   [0,0,0,0,0]}
    envStateDict = { "explored" : botStateDict["position"],
                     "workable" : [],
                     "complete" : []}

    return botStateDict, envStateDict

#------------------------------------MAIN--------------------------------------#
global botStateDict
global envStateDict
global coverageLevel
global roleSwapThreshold
global maxBotCount

coverageLevel = 1 # 1 = 3x3, 2 = 5x5 ... etc
gridBoxSize = 1
gridCount = 10
stopFlag = False
maxBotCount = 5
roleSwapThreshold = 0.5

fig = plt.figure(1)
[plt.plot([x/10,x/10], [-0.5,9.5], lw=1, color='grey') for x in range(-5, gridCount*10+5, 10)]
[plt.plot([-0.5,9.5], [y/10,y/10], lw=1, color='grey') for y in range(-5, gridCount*10+5, 10)]
plt.xlim(-0.5, gridCount-0.5)
plt.ylim(-0.5, gridCount-0.5)
# plt.show()

#--------------------------------INIT----------------------------------------#
botStateDict, envStateDict = init()
for i in range(maxBotCount):
    botStateDict["coverage"][i] = checkCoverage(botStateDict["position"][i])

#-------------------------------LOOP------------------------------------------#
while(stopFlag == False):
    for bot in range(maxBotCount):
        # getState() ...replaced with botStateDict for the moment
        # botState = [botStateDict["position"][n], botStateDict["role"][n], botStateDict["coverage"][n]]
        # position, role, coverage = calculate(botState)
        calculate(bot)
        pushState(bot, )
        do()
        pushData()

    pass