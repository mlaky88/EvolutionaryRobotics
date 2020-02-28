from abc import ABC, abstractmethod

class Problem():
    def __init__(self):
        self.bounds = list()
        self.dim = 0

    @abstractmethod
    def funcEval(self,*args):
        pass