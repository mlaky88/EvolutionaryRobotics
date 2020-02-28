

from differentialevolution import DifferentialEvolution
from deindividual import DeIndividual
from problem import Problem

import random as rnd
import numpy as np

class myProblem(Problem):
    def __init__(self,dim,bounds):
        super().__init__()
        self.dim = dim
        self.bounds = bounds
        print(len(self.bounds),self.dim)
        assert len(self.bounds) == self.dim  

    def funcEval(self,x):    
        total=0
        for i in range(len(x)):
            total+=x[i]**2
        return total /len(x)   

dimension = 10
prob = myProblem(dimension, [(-5.12,5.12) for _ in range(dimension)])

x1 = DeIndividual(prob)
x2 = DeIndividual(prob)

x2.randomize(prob)

x1.x = x2.x
x1.x[0] = 1337
x1.toString()
x2.toString()
#exit(1)
rnd.seed(42)
np.random.seed(42)

de = DifferentialEvolution(prob,popSize=20,maxFunEvals=10000*dimension)
de.run()