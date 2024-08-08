import math
import numpy as np

class TentaclePlanner:
    def __init__(self, dt=0.1, steps=5, alpha=1, beta=0.1):

        self.dt = dt
        self.beta = beta
        self.alpha = alpha
        self.steps = steps
        
        self.tentacles = [(0.0,0.5),
                          (0.0,-0.5),
                          (0.1,1.0),
                          (0.1,-1.0),
                          (0.1,0.5),
                          (0.1,-0.5),
                          (0.1,0.0),
                          (0.0,0.0)]
    
    def roll_out(self, v, w, goal_x, goal_y, goal_th, x, y, th):
        
        for _ in range(self.steps):
        
            x = x + self.dt*v*math.cos(th)
            y = y + self.dt*v*math.sin(th)
            th = (th + w*self.dt)
        
        e_th = goal_th-th
        e_th = math.atan2(math.sin(e_th), math.cos(e_th)) ** 2
        e_xy = (goal_x-x)**2 + (goal_y-y)**2
        
        return self.alpha*e_xy + self.beta*e_th
    
    def plan(self, goal_x, goal_y, goal_th, x, y, th):
        
        costs =[]
        for v, w in self.tentacles:
            dist = self.roll_out(v, w, goal_x, goal_y, goal_th, x, y, th)
            costs.append(dist)
        
        best_idx = np.argmin(costs)
        
        return self.tentacles[best_idx]