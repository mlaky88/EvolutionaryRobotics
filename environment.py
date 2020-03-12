import pygame as pg
import random as rnd

class Environment():
    def __init__(self,robots,obstacles):
        self.robots = robots
        self.obstacles = obstacles

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
        

class Robot():
    def __init__(self,loc,goalLoc, radius, velocity, color,sensorRange):
        self.loc = loc
        self.goalLoc = goalLoc
        self.radius = radius
        self.color = color
        self.velocity = velocity
        self.sensorRange = sensorRange
        self.robotRange = pg.Rect(self.loc[0]-self.sensorRange, self.loc[1]-self.sensorRange, self.sensorRange*2, self.sensorRange*2)

    def updateRobotPosition(self):

        motion = [[1, 0],[0, 1],[-1, 0],[0, -1],[-1, -1],[-1, 1],[1, -1],[1, 1]]
        move = motion[rnd.randint(0,2)]
        location = (self.loc[0]+move[0],self.loc[1]+move[1])
        self.loc = location
        #pg.draw.circle(screen, r.color, location, r.radius)
        self.robotRange = pg.Rect(self.loc[0]-self.sensorRange, self.loc[1]-self.sensorRange, self.sensorRange*2, self.sensorRange*2)
        #pg.draw.rect(screen,(255,0,0),robotRange,1)
        


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
