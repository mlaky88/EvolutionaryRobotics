import sys
import pygame as pg
from pygame.locals import *
import numpy as np
import random as rnd
import threading as thr
import time
rnd.seed(42)

from environment import Robot, Obstacle, Environment, Vec2D
from differentialevolution import DifferentialEvolution as DE
from problem import Problem


env = None
global finished

class rProblem(Problem):
    def __init__(self,bounds,dim):
        super().__init__()
        self.bounds = bounds
        self.dim = dim
        
    
    def funcEval(self,x):
        pass

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
    return quit

def updateAllRobotsPositions():
    global finished
    while not finished:
        time.sleep(0.01)
        for r in env.robots:
            if r.isMoving == False:
                pass
                #run DE and calculate point to move to
                #check collisions and everything
                #bounds = [(r.loc[0]-r.sensorRange,r.loc[0]+r.sensorRange),(r.loc[1]-r.sensorRange,r.loc[1]+r.sensorRange)]
                #DE()
                #r.updateRobotPosition()
            else:
                #just keep moving
                speed = 2
                r.loc += r.direction.mul(speed)
                #check if at nextLoc is reached then is moving set to False
                if r.loc == r.nextLoc:
                    r.isMoving = False


        
def drawRobots(screen,robots):
    for r in robots:        
        pg.draw.circle(screen, r.color, r.loc.toTuple(), r.radius)
        pg.draw.rect(screen,(255,0,0),r.robotRange,1)
        pg.draw.circle(screen, (255,128,0), r.goalLoc.toTuple(), r.radius)

def calcLocalAPF(robot,bounds, obstacles):
    ka = 5
    kr = 100
    pmap = np.zeros((robot.sensorRange*2,robot.sensorRange*2))
    for y in range(bounds[0][0],bounds[0][1]):
        for x in range(bounds[1][0],bounds[1][1]):
            pmap[x,y] = 0.5 * ka * np.hypot(x - robot.goalLoc[0], y - robot.goalLoc[1]) ** 2 #att
            d = [np.hypot(x - obst[0], y - o[1]) for obst in obstacles]
            #pmap[x,y] += 0.5 * ka * np.hypot(x - robot.goalLoc[0], y - robot.goalLoc[1]) ** 2  #rep      

def findPath(robots,obstacles):
    #apfMap = calcLocalAPF(robots)
    for r in robots:
        pass
        #for each robot calculate bounds
        
        #print(bounds, len(bounds))
        #localApf = calcLocalAPF(r)
        #prob = rProblem(bounds,len(bounds))
        #de = DE(prob)
        #best = de.run()

def draw():
    bgColor = (255,255,255)
    screenSize = (500,500)
    screen = pg.display.set_mode(screenSize)

    pg.init()
    clock = pg.time.Clock()
    
    frame_per_second = 30

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

    finished = False

    r1 = Robot(Vec2D(50,50),Vec2D(200,200),5,2,(255,0,0),30)

    o1 = Obstacle(40,100,200,50,20,300,-1,-1,True,1,0,2,(150,150,150))
    o2 = Obstacle(70,180,100,30,10,200,-1,-1,True,1,0,4,(150,150,150))
    o3 = Obstacle(300,130,30,130,-1,-1,-1,-1,False,0,0,0,(0,0,0))

    obst = [o1,o2,o3]
    robots = [r1]
    env = Environment(robots,obst)
    
    drawThread = thr.Thread(target=draw,daemon=True,args=())
    robotThread = thr.Thread(target=updateAllRobotsPositions,args=())
    drawThread.start()
    robotThread.start()
    robotThread.join()
    drawThread.join()
    pg.quit()
    sys.exit()