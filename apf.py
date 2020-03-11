import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance

class ArtificialPotentialField():
    def __init__(self,q0,qf,O,ka,kr,n,r,N,width,height):
        self.tao = 0 #number of configurations
        self.C = 0 #path length
        self.safe = True
        self.N = N #number of max allowed configurations
        self.r = r #radius of amrq
        self.ka = ka
        self.kr = kr
        self.n = n
        self.q0 = q0
        self.qf = qf
        self.width = width
        self.height = height
        self.O = O
        

    def calculate(self):
        G = np.hypot(self.qf,self.q0)
        pMap = self.calculateAPF()
        while G > 0 and self.tao < self.N and self.safe == True:
            

    def calcAttPot(self,x, y):        
        return 0.5 * self.ka * distance.euclidean((x,y),self.qf)# np.hypot(x - self.qf[0], y - self.qf[1])


    def rep(self,x,y):
        rep_p = 0
        for obs in self.O:
            d = np.hypot(x - obs[0], y - obs[1])
            if d == 0:
                #rep_p += 1000
                continue
            if d <= self.r:
                rep_p += 0.5 * self.kr * (1.0 / d - 1.0 / self.r) ** 2
        return rep_p
        
    def calcRepPot(self,x,y):

        d = [np.hypot(x - o[0], y - o[1]) for o in self.O]
        print(d)
        minid = d.index(min(d))

        dq = np.hypot(x - self.O[minid][0], y - self.O[minid][1])
        
        if dq <= self.r:
            if dq <= 0.1:
                dq = 0.1
            return 0.5 * self.kr * (1.0 / dq - 1.0 / self.r) ** 2
            
        else:
            return 0.0
        
    
    def calculateAPF(self):
        pmap = np.zeros((self.height,self.width))
        for x in range(self.height):
            for y in range(self.width):
                pmap[x,y] = self.calcAttPot(x,y)+self.rep(x,y)

        data = np.array(pmap).T

        print(pmap[0:10,0:10])
        print()
        plt.pcolor(data, vmax=pmap.max(), cmap=plt.cm.Blues)
        plt.plot(self.q0[0],self.q0[1], "*k")
        plt.plot(self.qf[0],self.qf[1], "*m")
        plt.show()


sx = (5,5)
sg = (40,40)
obs = [(15,20),(25,20),(30,32),(40,31)]
ka = 5;
kr = 100
n=100
rr = 5
N=1000
width=50
height=50
apf = ArtificialPotentialField(sx,sg,obs,ka,kr,n,rr,N,width,height)
apf.calculateAPF()

exit(1)
        





"""

Potential Field based path planner

author: Atsushi Sakai (@Atsushi_twi)

Ref:
https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf

"""

import numpy as np
import matplotlib.pyplot as plt

# Parameters
KP = 5.0  # attractive potential gain
ETA = 100.0  # repulsive potential gain
AREA_WIDTH = 50.0  # potential area width [m]

show_animation = True


def calc_potential_field(gx, gy, ox, oy, reso, rr):
    print("ox",ox)
    print("oy",oy)
    minx = min(ox) - AREA_WIDTH / 2.0
    miny = min(oy) - AREA_WIDTH / 2.0
    maxx = max(ox) + AREA_WIDTH / 2.0
    maxy = max(oy) + AREA_WIDTH / 2.0
    print(minx,miny,maxx,maxy)
    
    xw = int(round((maxx - minx) / reso))
    
    yw = int(round((maxy - miny) / reso))
    print(xw,yw)
    

    # calc each potential
    pmap = [[0.0 for i in range(yw)] for i in range(xw)]

    for ix in range(xw):
        x = ix * reso + minx
        print(x)
        

        for iy in range(yw):
            y = iy * reso + miny
            
            ug = calc_attractive_potential(x, y, gx, gy)
            uo = calc_repulsive_potential(x, y, ox, oy, rr)
            uf = ug + uo
            pmap[ix][iy] = uf
            #print(x,y,uf)
    print(len(pmap),len(pmap[0]))
    return pmap, minx, miny


def calc_attractive_potential(x, y, gx, gy):
    return 0.5 * KP * np.hypot(x - gx, y - gy)


def calc_repulsive_potential(x, y, ox, oy, rr):
    # search nearest obstacle
    minid = -1
    dmin = float("inf")
    for i, _ in enumerate(ox):
        d = np.hypot(x - ox[i], y - oy[i])
        if dmin >= d:
            dmin = d
            minid = i

    # calc repulsive potential
    dq = np.hypot(x - ox[minid], y - oy[minid])

    if dq <= rr:
        if dq <= 0.1:
            dq = 0.1

        return 0.5 * ETA * (1.0 / dq - 1.0 / rr) ** 2
    else:
        return 0.0


def get_motion_model():
    # dx, dy
    motion = [[1, 0],
              [0, 1],
              [-1, 0],
              [0, -1],
              [-1, -1],
              [-1, 1],
              [1, -1],
              [1, 1]]

    return motion


def potential_field_planning(sx, sy, gx, gy, ox, oy, reso, rr):

    # calc potential field
    pmap, minx, miny = calc_potential_field(gx, gy, ox, oy, reso, rr)

    # search path
    d = np.hypot(sx - gx, sy - gy) #dolzina do cilja
    ix = round((sx - minx) / reso)
    iy = round((sy - miny) / reso)
    gix = round((gx - minx) / reso)
    giy = round((gy - miny) / reso)

    if show_animation:
        draw_heatmap(pmap)
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(ix, iy, "*k")
        plt.plot(gix, giy, "*m")

    rx, ry = [sx], [sy]
    motion = get_motion_model()
    while d >= reso:
        minp = float("inf")
        minix, miniy = -1, -1
        for i, _ in enumerate(motion):
            inx = int(ix + motion[i][0])
            iny = int(iy + motion[i][1])
            if inx >= len(pmap) or iny >= len(pmap[0]):
                p = float("inf")  # outside area
            else:
                p = pmap[inx][iny]
            if minp > p:
                minp = p
                minix = inx
                miniy = iny
        ix = minix
        iy = miniy
        xp = ix * reso + minx
        yp = iy * reso + miny
        d = np.hypot(gx - xp, gy - yp)
        rx.append(xp)
        ry.append(yp)

        if show_animation:
            plt.plot(ix, iy, ".r")
            plt.pause(0.01)

    print("Goal!!")

    return rx, ry


def draw_heatmap(data):
    data = np.array(data).T
    plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)


def main():
    print("potential_field_planning start")

    sx = 00.0  # start x position [m]
    sy = 10.0  # start y positon [m]
    gx = 30.0  # goal x position [m]
    gy = 30.0  # goal y position [m]
    grid_size = 0.5  # potential grid size [m]
    robot_radius = 5.0  # robot radius [m]

    ox = [15.0, 5.0, 20.0, 25.0]  # obstacle x position list [m]
    oy = [25.0, 15.0, 26.0, 25.0]  # obstacle y position list [m]

    if show_animation:
        plt.grid(True)
        plt.axis("equal")

    # path generation
    _, _ = potential_field_planning(
        sx, sy, gx, gy, ox, oy, grid_size, robot_radius)

    if show_animation:
        plt.show()


if __name__ == '__main__':
    print(__file__ + " start!!")
    main()
    print(__file__ + " Done!!")
    