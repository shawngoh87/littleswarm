# File Name     :   PSO_local.py
# Created on    :   25/11/2016
# Author        :   Shawn Goh <shawngoh87@hotmail.com>

"""
This is a simplified version of PSO, emphasizing:
-- Direction of each swarm unit.
-- Locality, therefore updateBest is removed, and costs are calculated based on local inputs.
-- Trail left behind by each swarm unit.
-- A constantly arranged and trimmed pointArray buffer.
-- Scalability, adding more bots doesn't affect the algorithm as it is based on local rules.

Idea:
1.  Add local bias for points made for "trail"
2.  Use grid to replace point array.
    --  Initialize grid with size of searchSpace
    --  Whenever a coordinate is added, the grid[i][j] += 1\
        where i and j are the bounded boxes.
3.  Increase the size of the checking boxes.
4.  Grid checking to return spread consistency, grid size = 1% of search space?

Issues:
1.  The points still have noticeable clustering.
"""
#-----------------------IMPORTS & NAMESPACE-------------------------#
import os, sys
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gspec
from mpl_toolkits.mplot3d import Axes3D
import time
from math import *

#----------------------------FUNCTIONS------------------------------#

def testGraphs(pointArray):
    """Return 3D pointArray of contour and humidity"""
    contour = [[x,y,-(x**2)-y**2] for x,y in pointArray]
    humidity = [[x,y,x**2 - y**2] for x,y in pointArray]
    return contour, humidity
def add2DVectors(vec1, vec2):
    """Return vec1 + vec2"""
    return [vec1[0] + vec2[0], vec1[1] + vec2[1]]
def checkForward(position, velocity, angle, angleStep, pointArray):
    """Return cost count
    This function creates a triangle in the direction of the velocity,
    and counts the point in the triangle. Then plots three lines and store
    it in the global lineList for removal after the iteration.
    2------1------3
     \           /
      \         /       0 is the current position
       \       /        1 is the current position + current velocity,
        \     /                 which equals to the next position
         \   /          2 is cLeft
          \ /           3 is cRight
           0
    """
    count = 0
    px, py = position
    pointMid = rotate(velocity, angle)
    pointLeft = rotate(pointMid, angleStep / 2)
    pointRight = rotate(pointMid, -angleStep / 2)
    pointLeft = [x*checkScale for x in pointLeft]
    pointRight = [x*checkScale for x in pointRight]
    cLeft = add2DVectors(position, pointLeft)
    cRight = add2DVectors(position, pointRight)
    lineList.append(plt.plot([cLeft[0], cRight[0]], [cLeft[1], cRight[1]], color='k', lw=1),)
    lineList.append(plt.plot([cLeft[0], px], [cLeft[1], py], color='k', lw=1),)
    lineList.append(plt.plot([cRight[0], px], [cRight[1], py], color='k', lw=1),)
    for point in pointArray:
        if inPolygon(point, cLeft, cRight, position):
            count += 1
    return count
def cost(position, velocity, angleLimit, angleStep, pointArray):
    """Return next position and velocity
    This function checks the path forward using checkForward().
    Then checks if the next move is out-of-bound, reassign a move if yes.
    """
    costArray = []
    px, py = position
    for angle in range(-angleLimit, angleLimit + angleStep, angleStep):
        count = checkForward(position, velocity, angle, angleStep, pointArray)
        costArray.append([angle, count])
    costArray = sorted(costArray, key = lambda a:a[1])
    medianAngle = roulette(costArray, rouletteDegree)[0]
    vx, vy = rotate(velocity, medianAngle)
    cpoint = [px + vx, py + vy]
    while(outOfBound(cpoint)):
        temp = random.random() * 360 # Reflects when boundary hit
        vx, vy = rotate([vx, vy], temp)
        cpoint = [px + vx, py + vy]
    rand = random.uniform(0.5, 1.0) # Prevent clustering of resonant steps
    return [px + rand*vx, py + rand*vy], [vx, vy]
def dist(pointA, pointB):
    """Return Euclidean distance between pointA and pointB"""
    return np.linalg.norm(np.array(pointA) - np.array(pointB))
def inPolygon(point, *args):
    """Return True if the point is inside a convex polygon.
    Checks whether the target point is inside a polygon specified
    by the vertices.
    Example call: check point [2,3] in a triangle.
    inPolygon([2,3], [0,0], [10,10], [0,10])
    """
    prev = args[-1]
    for vertex in args:
        if onLeftSide(vertex, prev, point) == False:
            return False
        prev = vertex
    return True
def onLeftSide(linePointA, linePointB, point):
    """Return True if point is on left of line A to B
    Maths. Geometry.
    """
    x, y = point
    xA, yA = linePointA
    xB, yB = linePointB
    A = -(yB - yA)
    B = xB - xA
    C = -(A * xA + B * yA)
    D = A * x + B * y + C
    if D >= 0:
        return True
    else: return False
def outOfBound(point):
    """Return True if point is out-of-bound
    Checks global variable searchSpace if the target point
    is out-of-bound
    """
    x, y = point
    xMax, yMax = searchSpace[0], searchSpace[1]
    if (x <= xMax[0] or x >= xMax[1]) or (y <= yMax[0] or y >= yMax[1]):
        return True
    else: return False
def randomVector(range):
    """Return a float between range"""
    return random.uniform(range[0], range[1])
def rotate(vector, degree):
    """Return rotated vector"""
    x, y = vector
    degree = degree/180 *pi
    return [x*cos(degree) - y*sin(degree), x*sin(degree) + y*cos(degree)]
def roulette(array, degree):
    """Return an element of the array by weighted randomization.
    The degree will determine the weight of the elements.
    Example: a 4-element array will have the following weights--
    Degree 1: [4,3,2,1]     normalized to   [0.4, 0.3, 0.2, 0.1]
    Degree 2: [16,9,4,1]    normalized to   [0.53, 0.3, 0.13, 0.03]
    Degree 3: [64,27,8,1]   normalized to   [0.64, 0.27, 0.08, 0.01]
    """
    numel = len(array)
    rand = random.random()*sumAscend(numel, degree)
    for i in range(len(array)):
        rand -= numel**degree
        numel -= 1
        if rand <= 0:
            return array[len(array) - numel - 1]
def sumAscend(length, degree):
    """Return 1+2+3+...+length
    Degree will increase the power of addition
    """
    if length > 1:
        return length ** degree + sumAscend(length - 1, degree)
    else:
        return length
def checkGrid(pointArray):
    percent = 0.05
    total = 0
    totalSquared = []
    gridLength = percent*length
    gridSideCount = 1/percent
    pointDict = {}
    for i in range(int(searchSpace[1][0]),int(searchSpace[1][1]+gridLength),int(gridLength)):
        for j in range(int(searchSpace[0][0]),int(searchSpace[0][1]+gridLength),int(gridLength)):
            leftBound = j
            rightBound = j + gridLength
            upperBound = i + gridLength
            lowerBound = i
            withinBound = [[x[0],x[1]] for x in pointArray if leftBound<=x[0]<rightBound and lowerBound<=x[1]<upperBound]
            count = len(withinBound)
            total += count
            totalSquared.append(count)
            # if (count>2):
            #     print(j,i)
            pointDict[(j,i)] = [count, withinBound]
    avg = total/(gridSideCount**2)
    totalSquared = [(x-avg)**2 for x in totalSquared]
    variance = (sum(totalSquared))/(gridSideCount**2)
    sd = sqrt(variance)
    print("Average: " + str(avg))
    print("Variance: " + str(variance))
    print("Standard Deviation: " + str(sd))
    return pointDict

def init(step, maxParticle, searchSpace):
    """Return position and velocity array of size(maxParticle)"""
    particlePosition = []
    particleVelocity = []
    for i in range(maxParticle):
        particlePosition.append([randomVector(searchSpace[0]), randomVector(searchSpace[1])])
        temp = random.random()*360
        particleVelocity.append(rotate([step, step], temp))
    return particlePosition, particleVelocity

#----------------------------CONSTANTS------------------------------# SO MANY GLOBAL VARIABLES OH GOD
global rouletteDegree
global counter
global searchSpace
global step
global ellipseRatio
global lineList
global checkScale
global length
lineList = []
pointArray = []
stop = 1 # Not implemented yet
iteration = 0
counter = [0]*3
gap = 50 # Plot padding

length = 1000.0 # How big is the side length of the square field?
rouletteDegree = 4.0 # How attracted are the bots, to unexplored areas?
maxParticle = 5 # How many bots?
maxIteration = 100 # How many iteration?
checkAngle = 60 # How wide in front you want to check?
checkDirection = 3 # How many directions available to choose?

angleLimit = int(checkAngle/2) # Experimental relationship
angleStep = int((2*angleLimit)/(checkDirection-1)) # Experimental relationship
step = length/sqrt(maxIteration*maxParticle) # Experimental relationship
checkScale = 0.1*length/step # Experimental relationship
searchSpace = [[-(length/2),(length/2)], [-(length/2), (length/2)]] # [xrange,yrange] of the square field

#----------------------------MAIN------------------------------#
fig = plt.figure(1, facecolor='white')
fig.canvas.set_window_title("Real-time movement monitoring")
plt.xlim(searchSpace[0][0]-gap,searchSpace[0][1]+gap)
plt.ylim(searchSpace[1][0]-gap,searchSpace[1][1]+gap)
t = time.time()
particlePosition, particleVelocity = init(step, maxParticle, searchSpace)
while(iteration < maxIteration and stop):
    for n in range(maxParticle):
        print("Iteration: " + str(iteration) + " Particle: " + str(n))
        particlePosition[n], particleVelocity[n] = cost(particlePosition[n], particleVelocity[n], \
                                                        angleLimit, angleStep, pointArray)
        pointArray.append(particlePosition[n])
        plt.plot(particlePosition[n][0], particlePosition[n][1], 'b.')

    plt.pause(0.01)
    iteration += 1
    # Remove boxes
    for i in range(len(lineList)):
        """Removes every line in the figure
        Still not sure why a simple FOR-loop won't work
        Have to manually pop the list
        """
        line, = lineList.pop()
        line.remove()


pointDict = checkGrid(pointArray)
xy = [[x[0],x[1]] for x in pointDict if x[0] > 8]
# print(len(xy))

# Conclusion plot
print("Point Array stored: " + str(len(pointArray)) + " Coordinate pairs")
delta_time = round(time.time() - t,3)
print("Elapsed time: " + str(delta_time) + " seconds")
plt.figure(1)
plt.close()
fig = plt.figure(2, figsize=(10,7), facecolor='white')
fig.canvas.set_window_title("Summary")
gs = gspec.GridSpec(8,10) # 6 cells
ax1 = plt.figure(2).add_subplot(gs[0:5,0:5],title='Heatmap',
                                xlim=(searchSpace[0][0]-gap,searchSpace[0][1]+gap),
                                ylim=(searchSpace[1][0]-gap,searchSpace[1][1]+gap))
ax2 = plt.figure(2).add_subplot(gs[0:4,6:10],projection='3d',title='Contour')
ax3 = plt.figure(2).add_subplot(gs[4:8,6:10],projection='3d',title='Humidity')
ax4 = plt.figure(2).add_subplot(gs[6:8,0:5],frame_on=False,title='Search summary:')
ax4.get_xaxis().set_visible(False)
ax4.get_yaxis().set_visible(False)
ax4.text(0, 0.75, "Elapsed time:  ".expandtabs() + str(delta_time) + " seconds")
ax4.text(0, 0.5,  "Iterations:   \t\t".expandtabs(tabsize=4) + str(iteration)) # Stuffs
ax4.text(0, 0.25, "Bots:\t\t\t".expandtabs() + str(maxParticle)) # More Stuffs

contour, humidity = testGraphs(pointArray)

ax1.plot([i[0] for i in pointArray], [j[1] for j in pointArray], 'r.')

for x in range(-500,501,50):
    ax1.plot([x,x], [-500,500], lw=1, color='grey')

for y in range(-500,501,50):
    ax1.plot([-500,500], [y,y], lw=1, color='k')

ax2.plot_trisurf([i[0] for i in contour], [j[1] for j in contour], [k[2] for k in contour])
ax3.plot_trisurf([i[0] for i in humidity], [j[1] for j in humidity], [k[2] for k in humidity])

plt.show()
