import pygame as pg

class Environment():
    def __init__(self,size,robots):
        self.obs = None
        self.size = size
        self.robots = robots
        self.goal = goal
        

class Robot():
    def __init__(self,loc,goalLoc, radius, velocity, color,sensorRange):
        self.loc = loc
        self.goalLoc = goalLoc
        self.radius = radius
        self.color = color
        self.velocity = velocity
        self.sensorRange = sensorRange
        


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
