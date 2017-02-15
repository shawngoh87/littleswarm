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

def nextMove():
    pass
    botState = getState()
    # posList = [[x[0], x[1]] for x in botState]
    # bearList = [x[2] for x in botState]
    # covList = [x[3] for x in botState]
    # roleList = [x[4] for x in botState]


def getState(): # Get all bots' current state from server
    pass

def pushState(): # Push self state to server
    pass

def pushData():
    temperature = getTemp()
    altitude = getAltitude()
    x, y = getPos()
    state = [x, y, temperature, altitude]
    # TODO: POST request to server
    # return 0 if okay, 1 if not.
    pass

def getPos():

    return x, y
    pass

def getTemp(): # Get self current temperature
    pass

def getAltitude(): # Get self current altitude
    pass

def init(): # Initialization

    pass



#------------------------------------MAIN--------------------------------------#
stopFlag == False
init()
while(stopCondition == False):
    for n in range
    pushState()
    pushData()
    getState()
    calculate()
    move()
    pass