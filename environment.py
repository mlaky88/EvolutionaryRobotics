import pygame as pg
import random as rnd
import math
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import numpy as np

class Environment():
    def __init__(self,robots,obstacles,screenSize):
        self.robots = robots
        self.obstacles = obstacles
        self.screenSize = screenSize

    def updateWorldObstacles(self):
            for _, obst in enumerate(self.obstacles):
                if obst.isDynamic:
                    if obst.dx == 1 and obst.pyObst.x < obst.rightBarier: #moving right
                        obst.pyObst.x += obst.velocity
                    if obst.dx == -1 and obst.pyObst.x > obst.leftBarier: #moving left
                        obst.pyObst.x -= obst.velocity

                    if obst.pyObst.x + obst.width >= obst.rightBarier or obst.pyObst.x <= obst.leftBarier: #reverse movement at x barriers
                        obst.dx*=-1

                    if obst.dy == 1 and obst.pyObst.y < obst.lowBarier: #moving down
                        obst.pyObst.y += obst.velocity
                    if obst.dy == -1 and obst.pyObst.y > obst.topBarier:
                        obst.pyObst.x -= obst.velocity

                    if obst.pyObst.y + obst.height >= obst.lowBarier or obst.pyObst.y <= obst.topBarier: #reverse movement at x barriers
                        obst.dy*=-1

    def init(self):
        for r in self.robots:
            r.calculateGoalPotential()
        
class Vec2D():
    def __init__(self,x,y):
        self.x = x
        self.y = y

    def __add__(self,other):
        return Vec2D(self.x+other.x,self.y+other.y)

    def __sub__(self,other):
        return Vec2D(self.x-other.x,self.y-other.y)  

    def mul(self,value):
        return Vec2D(self.x * value,self.y*value)

    def div(self,value):
        return Vec2D(self.x / value,self.y/value)

    def __eq__(self,other):
        if int(self.x) == int(other.x) and int(self.y) == int(other.y):
            return True
        return False

    def toTuple(self):
        return tuple((int(self.x),int(self.y)))

    def distance(self,other):
        return math.sqrt(math.pow(self.x-other.x,2)+math.pow(self.y-other.y,2))


class Robot():
    def __init__(self,loc,goalLoc, radius, velocity, color,sensorRange,screenSize):
        self.loc = loc
        self.goalLoc = goalLoc
        self.nextLoc = goalLoc
        self.radius = radius
        self.color = color
        self.velocity = velocity
        self.sensorRange = sensorRange
        self.isMoving = False
        self.screenSize = screenSize
        self.isFinished = False
        self.robotRange = pg.Rect(self.loc.x-self.sensorRange, self.loc.y-self.sensorRange, self.sensorRange*2, self.sensorRange*2)
        self.history = []

    def addHistory(self,loc):
        #Always maintain history of 5 locations
        self.history.append(loc)
        
        if (len(self.history) > 5):
            del self.history[0]

    def calcSTDHistory(self):
        std_x = np.std(np.array([l.x for l in self.history]))
        std_y = np.std(np.array([l.y for l in self.history]))
        return (std_x+std_y)/2

    def checkCollisionWithObstacle(self,obstacles):
        point = Point(self.loc.x, self.loc.y)    
        robotCircle = point.buffer(self.radius)
        
        for obst in obstacles:
            obstPolygon = Polygon([(obst.left,obst.top+obst.height), (obst.left+obst.width,obst.top+obst.height), (obst.width+obst.left,obst.top),(obst.left,obst.top)])
            if obstPolygon.intersects(robotCircle):
                return True
        return False
        

    def calculateGoalPotential(self):
        ka = 10
        self.apf = np.zeros((self.screenSize[0],self.screenSize[1]))     
        for x in range(self.screenSize[0]):
            for y in range(self.screenSize[1]):    
                self.apf[x,y] = 0.5 * ka *  np.hypot(x - self.goalLoc.x, y - self.goalLoc.y)
        '''import matplotlib.pyplot as plt
        data = np.array(self.apf).T
        plt.pcolor(data, vmax=self.apf.max(), cmap=plt.cm.Blues)
        plt.gca().invert_yaxis()
        plt.show()'''



    def updateRobotPosition(self):
        motion = [[1, 0],[0, 1],[-1, 0],[0, -1],[-1, -1],[-1, 1],[1, -1],[1, 1]]
        move = motion[rnd.randint(0,2)]
        location = (self.loc.x+move[0],self.loc.y+move[1])
        self.loc = location
        self.robotRange = pg.Rect(self.loc.x-self.sensorRange, self.loc.y-self.sensorRange, self.sensorRange*2, self.sensorRange*2)
        

class Obstacle():
    def __init__(self,left,top,width,height,leftBarier,rightBarier,topBarier, lowBarier, isDynamic,dx,dy,velocity,color):
        self.isDynamic = isDynamic
        self.left = left
        self.top = top
        self.width = width
        self.height = height
        self.leftBarier = leftBarier
        self.rightBarier = rightBarier
        self.topBarier = topBarier
        self.lowBarier = lowBarier
        self.x = left
        self.dx = dx
        self.dy = dy
        self.velocity = velocity
        self.color = color
        self.pyObst = pg.Rect(self.left, self.top, self.width, self.height)
        #check for possible params conflicts: left and right bariers with width and length of obstacle
