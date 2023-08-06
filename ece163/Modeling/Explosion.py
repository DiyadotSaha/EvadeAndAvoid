import random
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule
import ece163.Constants.VehiclePhysicalConstants as VPC
class Explode:
    def __init__(self,x=0, y=0, z=0,  dT=VPC.dT):
        #"125.0 meters is the maximum radius of the explosion"
        self.max_radius = 40.0
        self.max_distance = 200.0
        self.min_distance = 80.0
        self.bn =x
        self.be = y
        self.bd = z
        self.radius = 0.1
        self.dT = dT
        self.climbing = False 
        self.done = False
        self.speed = 53.3
        self.maxZ = z + self.max_radius
        self.minZ = z - self.max_radius
        
        self.maxX = x + self.max_radius
        self.minX = x - self.max_radius

        self.maxY = y+ self.max_radius
        self.minY = y - self.max_radius
        #self.isDone = False
        return

    def startExplosion(self):
        self.climbing = True

    def __str__(self):
        desc = f"Bomb at {self.bn},{self.be}, {self.bd} with radius {self.radius:0.2f}"
        return desc

    def increase(self):
        self.radius += self.speed * self.dT
        self.radius = self.radius
        if self.radius > self.max_radius:
            self.radius = self.max_radius
            self.climbing = False
        return
    def decrease(self):
        self.radius -= self.speed * self.dT
        self.radius = self.radius
        if self.radius < 1:
            self.radius = 1
            self.done = True
        return
    def update(self):
        if self.done != True:
            if self.climbing:
                self.increase()
            else:
                self.decrease()
           

        