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
    Now store in same pc.
    """
    return

def pushState():
    """Push self state to server
    server = botStateDict"""

def checkCoverage(position):
    x, y = position
    leftColumn = [coveredGround[]]

def calculate(bot):
    p, r, c = [botStateDict["position"][bot], botStateDict["role"][bot], botStateDict["coverage"][bot]]
    if c < roleSwapThreshold:

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
    coveredGround = botStateDict["position"]
    return


#------------------------------------MAIN--------------------------------------#
global botStateDict
global coveredGround
global roleSwapThreshold


stopFlag = False
maxBotCount = 5
roleSwapThreshold = 0.5

init()
while(stopFlag == False):
    for bot in range(maxBotCount):
        #getState() ...replaced with botStateDict for the moment
        # botState = [botStateDict["position"][n], botStateDict["role"][n], botStateDict["coverage"][n]]
        # position, role, coverage = calculate(botState)
        calculate(bot)
        pushState(bot, )
        do()
        pushData()
    pass