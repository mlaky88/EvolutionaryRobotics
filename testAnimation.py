import sys
import pygame as pg
import random as rnd
rnd.seed(42)

from environment import *
from differentialevolution import DifferentialEvolution as DE
from problem import Problem



def moveRobots(screen,robots):
    motion = [[1, 0],[0, 1],[-1, 0],[0, -1],[-1, -1],[-1, 1],[1, -1],[1, 1]]
    for r in robots:
        move = motion[rnd.randint(0,2)]
        location = (r.loc[0]+move[0],r.loc[1]+move[1])
        r.loc = location
        pg.draw.circle(screen, r.color, location, r.radius)
        robotRange = pg.Rect(r.loc[0]-r.range, r.loc[1]-r.range, r.range*2, r.range*2)
        pg.draw.rect(screen,(255,0,0),robotRange,1)



def moveDynamicObstacles(obstacles):

    for _, obst in enumerate(obstacles):
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
            
    return obstacles

def drawObstacles(screen,obstacles):
    for _, obst in enumerate(obstacles): 
        pg.draw.rect(screen, obst.color, obst.pyObst)


def calcLocalAPF():
    for r in robots:
        

def findPath(robots):
    apfMap = calcLocalAPF(robots)
    prob = Problem()
    #de = DE()

def main():


    
    bgColor = (40,40,40)
    obstColor = (150,200,20)
    screenSize = (500,500)

    screen = pg.display.set_mode(screenSize)

    r1 = Robot((50,50),(200,200),5,2,(255,0,0),30)

    o1 = Obstacle(40,100,200,50,20,300,-1,-1,True,1,0,2,(150,200,20))
    o2 = Obstacle(70,180,100,30,10,200,-1,-1,True,1,0,4,(150,200,20))
    o3 = Obstacle(300,130,30,130,-1,-1,-1,-1,False,0,0,0,(250,200,20))


    O = [o1,o2,o3]
    robots = [r1]
    
    clock = pg.time.Clock()

    frame_per_second = 30
    running = True

    while running:
        for event in pg.event.get():
            print(event,event.type)
            if event.type == pg.QUIT:
                running = False
                break
        
        findPath()

        screen.fill(bgColor) #set background
        O = moveDynamicObstacles(O)       

        drawObstacles(screen,O) 

        moveRobots(screen,robots)

        #checkCollision()
        pg.display.flip()
        clock.tick(frame_per_second)

if __name__ == '__main__':
    pg.init()
    main()
    pg.quit()
    sys.exit()