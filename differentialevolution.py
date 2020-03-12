
import numpy as np
import copy as cp
import random as rnd
import itertools

from deindividual import DeIndividual



class DifferentialEvolution():
    def __init__(self,problem,popSize=20, maxFunEvals=10e4):
        self.popSize = popSize
        self.maxFunEvals = maxFunEvals
        self.pop = []
        self.problem = problem

        self.best = DeIndividual(self.problem)

    def generatePop(self):
        for i in range(self.popSize):
            self.pop.append(DeIndividual(self.problem))
            self.pop[i].randomize(self.problem)
            self.pop[i].eval = self.problem.funcEval(self.pop[i].x)
            self.pop[i].toString()
        self.best = cp.deepcopy(max(self.pop))        

    def searchBest(self):
        cB = max(self.pop)
        if self.best < cB:
            self.best = cp.deepcopy(cB)

    def evolveGeneration(self):        
        for i, current in enumerate(self.pop):
            #print("Current x: ", i, current.x)
            trial = cp.deepcopy(current)    
            trial.updateCtrlParams()
            idxs = [idx for idx in range(self.popSize) if idx != i]            
            r1, r2, r3 = list(itertools.chain(*[np.random.choice(idxs, 3, replace=False)]))

            jrnd = int(rnd.random()*self.popSize)
            for j in range(self.problem.dim):
                if rnd.random() < trial.Cr or j == jrnd:                    
                    trial.x[j] = self.pop[r1].x[j] + trial.F * (self.pop[r2].x[j]-self.pop[r3].x[j])

            trial.repair(self.problem)
            trial.eval = self.problem.funcEval(trial.x)

            self.currFunEvals += 1
            if trial > current:
                self.pop[i] = cp.deepcopy(trial)

                  

    def run(self):
        self.currFunEvals = 0
        genCounter = 1
        self.generatePop()
        while self.currFunEvals < self.maxFunEvals:
            print("Generation {}".format(genCounter))
            self.evolveGeneration()
            self.searchBest() 
            genCounter += 1
            print("Best",end='')
            self.best.toString()
        return self.best.x

