"""This file is a test harness for the module VehicleDynamicsModel. 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter3.py (from the root directory) -or-
python testChapter3.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehicleDynamicsModel module"""

# %% Initialization of test harness and helpers:

import math

import sys

sys.path.append("..")  # python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Modeling.VehicleAerodynamicsModel as VAM

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda a, b: math.isclose(a, b, abs_tol=1e-6)


def compareVectors(a, b):
    """A quick tool to compare two vectors"""
    el_close = [isclose(a[i][0], b[i][0]) for i in range(len(a))]
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


# %% Derivative():
print("Beginning testing of VAM!\n")
proptest = "Propellor test"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testFm = Inputs.forcesMoments()
test_throttle = Inputs.controlInputs()
testprop = testVAM.CalculatePropForces(testState.Va, test_throttle.Throttle)

resultprop = [[testprop[0]], [testprop[1]]]
expectedprop = [[21.817680754436033], [-0.6194943564776727]]

if not evaluateTest(proptest, compareVectors(resultprop, expectedprop)):
    print(f"{resultprop} != {expectedprop}")

testgrav = "Gravity test"
testgravity = testVAM.gravityForces(testState)
resultgrav = [[testgravity.Fx], [testgravity.Fy], [testgravity.Fz]]
expectedgrav = [[0], [0], [107.91]]
if not evaluateTest(testgrav, compareVectors(resultgrav, expectedgrav)):
    print(f"{resultgrav} != {expectedgrav}")

controlftest = "ControlForces Force Test"
controlmtest = "ControlForces Moments Test"
controlf = testVAM.controlForces(testState, test_throttle)
resultf = [[controlf.Fx], [controlf.Fy], [controlf.Fz]]
resultm = [[controlf.Mx], [controlf.My], [controlf.Mz]]
expectf = [[21.8176807], [0], [0]]
expectm = [[-0.6194943], [0], [0]]
if not evaluateTest(controlftest, compareVectors(resultf, expectf)):
    print(f"{resultf} != {expectf}")

if not evaluateTest(controlmtest, compareVectors(resultm, expectm)):
    print(f"{resultm} != {expectm}")

aeroftest = "aeroForces Forces test"
aeromtest = "aeroForces Moments test"
testState.Va = 3.5
aero = testVAM.aeroForces(testState)
aerof = [[aero.Fx], [aero.Fy], [aero.Fz]]
aerom = [[aero.Mx], [aero.My], [aero.Mz]]
aerofexpect = [[-0.2615782], [0], [-0.982617]]
aeromexpect = [[0], [0.0109548], [0]]
if not evaluateTest(aeroftest, compareVectors(aerof, aerofexpect)):
    print(f"{aerof} != {aerofexpect}")

if not evaluateTest(aeromtest, compareVectors(aerom, aeromexpect)):
    print(f"{aerom} != {aeromexpect}")

alpha = 3.5
coefftest = "Coeff Test"
coeff = testVAM.CalculateCoeff_alpha(alpha)
coeffres = [[coeff[0]], [coeff[1]], [coeff[2]]]
coeffexp = [[0.656986], [0.246097], [-9.5765]]
if not evaluateTest(coefftest, compareVectors(coeffres, coeffexp)):
    print(f"{coeffres} != {coeffexp}")

updateftest = "Update Forces f Test"
updatemtest = "Update Forces m Test"
update = testVAM.updateForces(testState, test_throttle)
updatefres = [[update.Fx], [update.Fy], [update.Fz]]
updatemres = [[update.Mx], [update.My], [update.Mz]]
updatefexp = [[21.817680], [0], [107.91]]
updatemexp = [[-0.619494], [0], [0]]
if not evaluateTest(updateftest, compareVectors(updatefres, updatefexp)):
    print(f"{updatefres} != {updatefexp}")

if not evaluateTest(updatemtest, compareVectors(updatemres, updatemexp)):
    print(f"{updatemres} != {updatemexp}")

# %%

"""
Students, add more tests here.  
You aren't required to use the testing framework we've started here, 
but it will work just fine.
"""

# %% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
    print(f"Failed {len(failed)}/{total} tests:")
    [print("   " + test) for test in failed]
