
import numpy as np
import copy as cp
import random as rnd

class DeIndividual():
    def __init__(self,dim, bounds):
        self.F = 0.5
        self.Cr = 0.9
        self.tao = 0.10

        self.dim = dim
        self.bounds = bounds #array of tuple -> [(0,1),(0,1),(0,1)] if dim = 3
        assert len(self.bounds) != self.dim
        self.x = [0 for _ in range(self.dim)]
        self.eval = float('inf')   
        

    def evaluate(self):
        pass

    def updateCtrlParams(self,parent):
        self.F = parent.F
        self.Cr = parent.Cr
        
        if rnd.random() < self.tao:
            self.F = rnd.random()            

        if rnd.random() < self.tao:
            self.Cr = rnd.random()

    def __gt__(self, other):
        return self.eval < other.eval

    def repair(self):
        self.x = [self.bounds[i][0] + rnd.random()*(self.bounds[i][1]-self.bounds[i][0]) for i, x in enumerate(self.x) if x < self.bounds[i][0] or x > self.bounds[i][1]]


class DifferentialEvolution():
    def __init__(self,popSize=20, maxFunEvals=10e4,dim=None,bounds=None):
        self.popSize = popSize
        self.maxFunEvals = maxFunEvals
        self.pop = []
        self.dim = dim
        self.bounds = bounds

        self.best = DeIndividual(self.dim, self.bounds)

    def generatePop(self):
        pass

    def searchBest(self):
        cB = max(self.pop)
        if self.best > cB:
            self.best = cp.deepcopy(cB)

    def evolveGeneration(self):
        pass
        self.best = cp.deepcopy(max(self.pop))
        for i, current in enumerate(self.pop):
            trial = DeIndividual(self.dim,self.bounds)
            trial.updateCtrlParams(parent)
            idxs = [idx for idx in range(popsize) if idx != i]
            r1, r2, r3 = [np.random.choice(idxs, 3, replace = False)]
            
            jrnd = int(rnd.random()*self.popSize)
            trial.x = parent.x
            for j in range(current.dim):
                if rnd.random() < trial.Cr or j == jrnd:
                    trial.x[j] = self.pop[r1][j] + trial.F * (self.pop[r2][j]-self.pop[r3][j])

            trial.repair()
            trial.eval()
            self.currFunEvals += 1
            #if trial 

            

    def run(self):
        self.currFunEvals = 0
        genCounter = 1
        while currFunEvals < self.maxFunEvals:
            print("Generation {}".format(genCounter))
            self.evolveGeneration()

            genCounter += 1

x1 = DeIndividual(1,[0,0])
x2 = DeIndividual(1,[0,0])

x1.eval = 2
x2.eval = 4

x = [x1,x2]
best = max(x)

print(x1<x2)
print(x1>x2)
print(best.eval)
