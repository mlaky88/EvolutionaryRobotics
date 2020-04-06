import sys
import pygame as pg
from pygame.locals import *
import numpy as np
import random as rnd
import threading as thr
import time
import math

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
import matplotlib.pyplot as plt
rnd.seed(42)

from environment import Robot, Obstacle, Environment, Vec2D
from differentialevolution import DifferentialEvolution as DE
from problem import Problem


env = None
global finished #

class robotNavProblem(Problem):
    def __init__(self):
        super().__init__()

    def setParams(self,bounds, goal,radius,currLoc,robots,obstacles,apf):
        self.bounds = bounds
        self.dim = len(bounds)
        self.CollideObstaclePenalty = 5000
        self.CollideRobotPenalty = 4000
        self.LocalMinimaPenalty = 1
        self.isDistance = 1
        self.goal = goal
        self.currLoc = currLoc
        self.robots = robots
        self.obstacles = obstacles
        self.radius = radius
        self.apf = apf        
    
    def funcEval(self,sol):
        x = int(sol[0])
        y = int(sol[1])
        isObstCollison = 0
        isRobotCollison = 0
        isLocalMinima = 0
        #print(x)
        distanceToTarget = math.sqrt(math.pow(self.goal.x-x,2)+math.pow(self.goal.y-y,2))
        
        point = Point(x, y)    
        robotCircle = point.buffer(self.radius)
        #checkCollisonWithObstacles
        for obst in self.obstacles:
            obstPolygon = Polygon([(obst.left,obst.top+obst.height), (obst.left+obst.width,obst.top+obst.height), (obst.width+obst.left,obst.top),(obst.left,obst.top)])
            if obstPolygon.intersects(robotCircle):
                isObstCollison = 1
                break
        
        
        #checkCollisionWithRobots
        p = Vec2D(x,y)
        for robot in self.robots:
            if robot.goalLoc == self.goal:
                continue
            if robot.loc.distance(p) < robot.radius:
                isRobotCollison = 1
                break
        #localMinima??
        #CalcAveragePopultion, if centered around one point, create avoid area, soultions must go away from this area or penalize
        


        #if abs(x[0]-self.goal.x) < 1 or abs(x[1]-self.goal.y) < 1:
            #isLocalMinima = 1
            #print("Minima")
            
            #self.isDistance = 0
        #polygon = Polygon([(0, 0), (0, 1), (1, 1), (1, 0)])
        #print(polygon.contains(point))     
        
        print(self.apf[x,y],self.apf[x+1,y])
        #print(self.apf[180,200])
        #print(self.apf.min())
        #x_i,y_i = np.unravel_index(np.argmin(self.apf),self.apf.shape)
        #print(x_i,y_i)
        #check point for obstacle collision
        #check point for robot colision
        #calculate local apf and let it be a guide for point calculation

        #print(distanceToTarget)
        
        return isLocalMinima * self.LocalMinimaPenalty + isObstCollison * self.CollideObstaclePenalty + isRobotCollison * self.CollideRobotPenalty + self.isDistance*distanceToTarget

        

def drawObstacles(screen,obstacles):
    for _, obst in enumerate(obstacles): 
        pg.draw.rect(screen, obst.color, obst.pyObst)


def done():
    
    quit = False 
    global finished
    for event in pg.event.get():
        if event.type == pg.KEYDOWN:
            if event.key == K_ESCAPE:
                quit = True
                finished = True
        if event.type == pg.QUIT:
            quit = True
            finished = True
    if finished == True:
        quit = True
    return quit

def updateAllRobotsPositions():
    global finished
    prob = robotNavProblem()
    while not finished:
        time.sleep(0.005*20)
        for r in env.robots:
            if r.isFinished == True:
                continue

            if r.isMoving == False:
                
                #run DE and calculate point to move to
                #check collisions and everything
                #clip out of bounds!!! on screen                
                problemBounds = [(r.loc.x-r.sensorRange,r.loc.x+r.sensorRange),(r.loc.y-r.sensorRange,r.loc.y+r.sensorRange)]
            
                prob.setParams(problemBounds,r.goalLoc,r.radius,r.loc,env.robots,env.obstacles,r.apf)
                opt = DE(prob,popSize=15,maxFunEvals=3000).run()

                r.nextLoc = Vec2D(int(opt[0]),int(opt[1]))
                r.distance = r.nextLoc.distance(r.loc)
                r.addHistory(r.nextLoc)

                r.direction = (r.nextLoc - r.loc).div(r.distance)                
                r.isMoving = True
            else:
                #print(r.loc.x,r.loc.y)
                #just keep moving
                speed = 3
                r.loc += r.direction.mul(speed)
                if r.checkCollisionWithObstacle(env.obstacles):# and r.checkCollisionWithRobot == True:
                    r.isMoving = False
                    r.loc -= r.direction.mul(speed)

                #check for collision ; if present set moving to false
                r.robotRange = pg.Rect(r.loc.x-r.sensorRange, r.loc.y-r.sensorRange, r.sensorRange*2, r.sensorRange*2)
                #check if at nextLoc is reached then is moving set to False
                #print(r.loc.distance(r.nextLoc))
                if r.loc.distance(r.nextLoc) <= 3: #and also if collision set moving to false TODO
                    r.isMoving = False
                
                if r.loc.distance(r.goalLoc) <= 1:
                    r.isFinished = True
        if sum([1 if r.isFinished else 0 for r in env.robots]) == len(env.robots):
            finished = True

        
def drawRobots(screen,robots):
    for r in robots:        
        pg.draw.circle(screen, r.color, r.loc.toTuple(), r.radius)
        pg.draw.circle(screen, (0,0,255), r.nextLoc.toTuple(), r.radius)
        pg.draw.rect(screen,(255,0,0),r.robotRange,1)
        pg.draw.circle(screen, (255,128,0), r.goalLoc.toTuple(), r.radius)
 


def draw():
    global screenSize
    bgColor = (255,255,255)
    
    screen = pg.display.set_mode(screenSize)

    pg.init()
    clock = pg.time.Clock()
    
    frame_per_second = 30
    time.sleep(1)
    while not done():
        #time.sleep(1)              
        screen.fill(bgColor) #set background
        
        env.updateWorldObstacles()        

        drawObstacles(screen,env.obstacles)         
        drawRobots(screen,env.robots)
        
        pg.display.update()
        clock.tick(frame_per_second)

if __name__ == '__main__':   
    global finished
    global screenSize

    finished = False
    screenSize = (500,500)

    r1 = Robot(Vec2D(50,50),Vec2D(180,200),5,3,(255,0,0),40,screenSize)
    r2 = Robot(Vec2D(400,80),Vec2D(370,400),5,3,(255,0,0),40,screenSize)

    o1 = Obstacle(40,100,200,50,20,300,-1,-1,False,1,0,1,(150,150,150))
    o2 = Obstacle(70,180,100,30,10,200,-1,-1,False,1,0,1,(150,150,150))
    o3 = Obstacle(300,130,30,130,-1,-1,-1,-1,False,0,0,0,(0,0,0))

    
    #obstPolygon = Polygon([(o1.left,o1.top+o1.height), (o1.left+o1.width,o1.top+o1.height), (o1.width+o1.left,o1.top),(o1.left,o1.top)])
    '''p1 = Polygon([(10,20),(10,40),(30,40),(30,20)])
    p2 = Polygon([(15,25),(15,35),(20,35),(20,25)])
    p3 = Polygon([(8,25),(15,35),(20,35),(20,25)])
    plt.plot(*p1.exterior.xy)
    plt.plot(*p2.exterior.xy)
    plt.plot(*p3.exterior.xy)
    print(p1.contains(p2))
    print(p1.contains(p3))
    #print(*obstPolygon.exterior.xy)
    plt.show()
    '''
    obst = [o1,o2,o3]
    robots = [r1]
    env = Environment(robots,obst,screenSize)
    env.init()


    drawThread = thr.Thread(target=draw,daemon=True,args=())
    robotThread = thr.Thread(target=updateAllRobotsPositions,args=())
    drawThread.start()
    robotThread.start()
    robotThread.join()
    drawThread.join()
    pg.quit()
    sys.exit()