"""This file is a test harness for the module Explosion .

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testEvadeandAvoid.py (from the root directory) -or-
python testEvadeandAvoid.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehicleClosedLoopControl module"""
import math

import sys

sys.path.append("..")  # python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Controls.VehiclePerturbationModels as VPM
import ece163.Modeling.WindModel as WM
import ece163.Controls.VehicleTrim as VehicleTrim
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Controls.VehicleClosedLoopControl as VCLC
import ece163.Containers.Controls as Controls

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda a, b: math.isclose(a, b, abs_tol=1e-6)


def compareVectors(a, b):
    """A quick tool to compare two vectors"""
    el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
    return all(el_close)


# of course, you should test your testing tools too:
assert (compareVectors([[0], [0], [-1]], [[1e-13], [0], [-1 + 1e-9]]))
assert (not compareVectors([[0], [0], [-1]], [[1e-5], [0], [-1]]))
assert (not compareVectors([[1e8], [0], [-1]], [[1e8 + 1], [0], [-1]]))

failed = []
passed = []


def evaluateTest(test_name, boolean):
    """evaluateTest prints the output of a test and adds it to one of two
	global lists, passed and failed, which can be printed later"""
    if boolean:
        print(f"   passed {test_name}")
        passed.append(test_name)
    else:
        print(f"   failed {test_name}")
        failed.append(test_name)
    return boolean


VCL = VCLC.VehicleClosedLoopControl()
controls = Controls.referenceCommands()
VCL.Update(controls)
