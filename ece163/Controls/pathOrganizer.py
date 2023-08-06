

import math 
from ..Utilities import Rotations

class pathOrganizer:



    def __init__(self, underlyingModel, explosions = []):

        self.underlying = underlyingModel

        self.VAM = underlyingModel.getVehicleAerodynamicsModel()

        self.explosions = explosions

        self.currentPath = [[1000.0, 0.0, -100.0], [1200.0, -60.0, -100.0], [1000.0, -160.0, -150.0], [3000.0, 2000.0, -400.0],
             [4500.0, 0.0, -1000.0], [7000.0, 0.0, -1500.0]] #init empty list to store points in our path
        
        self.commandedz = -100
        self.commandedy = 0
        self.step = 100

        """
        0 = fly straight state
        1 = left turn state
        2 = right turn state
        3 = turn around state
        4 = climb state
        5 = dive state

        """
        self.state = 0

        self.isInExplosion = False
        self.shouldMoveInX = False
        self.shouldMoveInY = False 
        self.shouldMoveInZ = False

    def setExplosions(self, explosions):
        self.explosions = explosions

    def getPath(self):

        return self.currentPath
    
    def updatePath(self):
        #will set this variable to use the closest escape to directions 
        self.fastestXDir = 0
        self.fastestYDir = 0 
        self.fastestZDir = 0

        fastestXDirAmt= 0
        fastestYDirAmt = 0 
        fastestZDirAmt = 0

        closestBomb = 0
        lastClosestD = 0
        
        state = self.VAM.getVehicleState()
        enuPoints = Rotations.ned2enu([[state.pn, state.pe, state.pd]])

        x = enuPoints[0][0]
        y = enuPoints[0][1]
        z = enuPoints[0][2]
        for explosion in self.explosions:
            d = math.hypot((x - explosion.bn), (y - explosion.be), (z - explosion.bd))
            if( d > lastClosestD):
                lastClosestD = d
                closestBomb = explosion


            if( abs(x - explosion.maxX) > abs(x - explosion.minX)):
                fastestXDir = 1
            else:
                fastestXDir = -1

            if( abs(y - explosion.maxY) > abs(y - explosion.minY)):
                fastestYDir = 1
            else:
                fastestYDir = -1

            if( abs(z - explosion.maxZ) > abs(z - explosion.minZ)):
                fastestZDir = 1
            else:
                fastestZDir = -1

        #print("Distance to closest Bomb: ", lastClosestD)
        #print("Distance to closest Bomb: ", lastClosestD)

        #x = enuPoints[0][0]
        #y = enuPoints[0][1]
        #z = enuPoints[0][2]

        x = state.pn
        y = state.pe
        z = -state.pd
        if len(self.explosions) > 0:
            for explosion in self.explosions:
                d = math.hypot((x - explosion.bn), (y - explosion.be), (z - explosion.bd))
                if( d > lastClosestD):
                    lastClosestD = d
                    closestBomb = explosion

                xpos = abs(x - explosion.maxY)
                xneg = abs(x - explosion.minY) 
                if( xpos < xneg):
                    self.fastestXDir = 1
                    fastestXDirAmt = xpos
                else:
                    self.fastestXDir = -1
                    fastestXDirAmt = xneg
                if explosion.minX < y < explosion.maxX:
                    self.shouldMoveInY = True
                else: self.shouldMoveInY = False
                    
                ypos = abs(y - explosion.maxX)
                yneg = abs(y - explosion.minX)
                if( ypos < yneg):
                    self.fastestYDir = 1
                    fastestYDirAmt = ypos
                else:
                    self.fastestYDir = -1
                    fastestYDirAmt = yneg
                if explosion.minY < x < explosion.maxY:
                    self.shouldMoveInX = True
                else: self.shouldMoveInX = False

                zpos = abs(z - explosion.maxZ)
                zneg = abs(z - explosion.minZ)
                if( zpos < zneg):
                    self.fastestZDir = 1
                    fastestZDirAmt = zpos
                else:
                    self.fastestZDir = -1
                    fastestZDirAmt = zneg

                if explosion.minX < x < explosion.maxX:
                    self.shouldMoveInZ = True
                else: self.shouldMoveInZ = False

                if self.shouldMoveInZ  or self.shouldMoveInY or self.shouldMoveInX:
                    self.isInExplosion = True
                else: self.isInExplosion = False

            print(x,y,z) 
            print("is in explosion:" , self.isInExplosion)
            print("movex:" , self.shouldMoveInX)
            print("movey:" , self.shouldMoveInY)
            print("movez:" , self.shouldMoveInZ)
            #print("Distance to closest Bomb Origin: ", lastClosestD)
            
            print("Distance to cloest escape in x: ", fastestXDirAmt)
            print("bombs xmax = ", closestBomb.maxY)
            print("bombs xmin = ", closestBomb.minY)
            print("difference btwn x and bomb max x is: ", xpos)
            print("difference btwn x and bomb min x is: ", xneg)

            print("Distance to cloest escape in y: ", fastestYDirAmt)
            print("bombs ymax = ", closestBomb.maxX)
            print("bombs ymin = ", closestBomb.minX)
            print("difference btwn y and bomb max y is: ", ypos)
            print("difference btwn y and bomb min yis: ", yneg)

            print("Distance to cloest escape in z: ", fastestZDirAmt)
            print("bombs zmax = ", closestBomb.maxZ)
            print("bombs zmin = ", closestBomb.minZ)
            print("difference btwn z and bomb max z is: ", zpos)
            print("difference btwn z and bomb min z is: ", zneg)
            
        #after the code above we should now have references to the three closest bomb objects with respect to each axis.
        #This could be all the same bomb or all different ones.

        #using the bomb references we can decide which direction to manuever in would be the most optimal

        #take current explosions and see if the path falls in the range of the max radius of the bomb
        #if so update path starting from closest point to the max radius of the bomb

        #for closestXExplosion

        if self.state == 0:

            if self.isInExplosion:
                
                x = x * self.fastestXDir
                y = y * self.fastestYDir
                z = z * self.fastestZDir
                
                if(self.shouldMoveInX and not self.shouldMoveInY and not self.shouldMoveInZ):
                   
                    self.currentPath = [[x + self.step, y, z], [x + self.step * 2, y, z], [x + self.step * 3, y, z], [x + self.step * 4, y, z], [x + self.step * 5, y, z]]

                elif(self.shouldMoveInY and not self.shouldMoveInX and not self.shouldMoveInZ):
                  
                    self.currentPath = [[x , y + self.step, z], [x, y + self.step * 2, z], [x , y+ self.step * 3, z], [x, y+ self.step * 4, z], [x, y+self.step * 5, z]]
                
                elif(self.shouldMoveInZ and not self.shouldMoveInY and not self.shouldMoveInX):
                  
                    self.currentPath = [[x , y , z + self.step], [x, y , z + self.step * 2], [x , y, z+ self.step * 3], [x, y, z+ self.step * 4], [x, y, z+ self.step * 5]]



                elif(self.shouldMoveInX and self.shouldMoveInY and not self.shouldMoveInZ):
              
                    self.currentPath = [[x + self.step, y + self.step, z], [x + self.step * 2, y + self.step * 2, z], [x + self.step * 3, y + self.step * 3, z], [x + self.step * 4, y + self.step * 4, z], [x + self.step * 5, y + self.step * 5, z]]

                elif(self.shouldMoveInX and self.shouldMoveInZ and not self.shouldMoveInY):

                    self.currentPath = [[x + self.step, y, z + self.step], [x + self.step * 2, y, z+self.step * 2], [x + self.step * 3, y, z+self.step * 3], [x + self.step * 4, y, z+ self.step * 4], [x + self.step * 5, y, z+ self.step * 5]]
               
                elif(self.shouldMoveInY and self.shouldMoveInZ and not self.shouldMoveInX):

                    self.currentPath = [[x , y + self.step, z+ self.step], [x , y+ self.step * 2, z+ self.step * 2], [x, y + self.step * 3, z + self.step * 3], [x , y+ self.step * 4, z+ self.step * 4], [x , y+ self.step * 5, z+ self.step * 5]]

                else:
                    self.currentPath = [[x + self.step, y+ self.step, z+ self.step], [x + self.step * 2, y+ self.step * 2, z+ self.step * 2], [x + self.step * 3, y+ self.step * 3, z+ self.step * 3], [x + self.step * 4, y+ self.step * 4, z+ self.step * 4], [x + self.step * 5, y+ self.step * 5, z+ self.step * 5]]  


            else:

                #get back on normal path 
                self.currentPath = [[x + self.step, self.commandedy, self.commandedz], [x + self.step * 2, self.commandedy, self.commandedz], [x + self.step * 3, self.commandedy, self.commandedz], [x + self.step * 4, self.commandedy, self.commandedz], [x + self.step * 5, self.commandedy, self.commandedz]]
    

        elif self.state == 1:
            pass

        elif self.state == 2:

            pass
        elif self.state == 3:
            pass

        elif self.state == 4:
            pass

        elif self.state == 5:
            pass

        else: 
            print("shouldnt really happen")

        self.underlying.setWaypoints(self.currentPath)
        return self.currentPath
    