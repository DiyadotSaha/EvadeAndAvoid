"""This file is a test harness for the module Explosion .

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testEvadeandAvoid.py (from the root directory) -or-
python testEvadeandAvoid.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehicleClosedLoopControl module"""
import sys
sys.path.append("..")
import math
import ece163.Modeling.VehicleAerodynamicsModel as VAM
import ece163.Modeling.Explosion as Bomb

testVAM = VAM.VehicleAerodynamicsModel()
testBomb1 = Bomb.Explode()
testBomb2 = Bomb.Explode()
testBomb1.startExplosion()
for i in range(310):
    testBomb1.update()
    testBomb2.update()
    if i == 150:
        testBomb2.startExplosion()
    print("Testing Bomb1: ", testBomb1)
    print("Testing Bomb2: ", testBomb2,"\n")



