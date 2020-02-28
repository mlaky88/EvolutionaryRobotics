import random as rnd

class DeIndividual():
    def __init__(self,problem):
        self.F = 0.5
        self.Cr = 0.9
        self.tao = 0.10

        self.x = [0 for _ in range(problem.dim)]
        self.eval = float('inf')   
        

    def randomize(self,problem):
        self.x = [problem.bounds[i][0] + rnd.random()*(problem.bounds[i][1]-problem.bounds[i][0]) for i, x in enumerate(self.x)]

    def updateCtrlParams(self):        
        if rnd.random() < self.tao:
            self.F = rnd.random()            

        if rnd.random() < self.tao:
            self.Cr = rnd.random()
    
    def toString(self):        
        print(self.x,self.eval)

    def __gt__(self, other):
        return self.eval < other.eval

    def repair(self,problem):
        self.x = [problem.bounds[i][0] + rnd.random()*(problem.bounds[i][1]-problem.bounds[i][0]) if x < problem.bounds[i][0] or x > problem.bounds[i][1] else x for i, x in enumerate(self.x)]
