"""This file is a test harness for the module ece163.Utilities.Rotations,
and for the method ece163.Modeling.VehicleGeometry.getNewPoints(). 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter2.py (from the root directory) -or-
python testChapter2.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the Rotations module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleGeometry as VG
import ece163.Utilities.HW1Q4 as attitude

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-6)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-5], [0], [-1]]))
assert(not compareVectors([[1e8], [0], [-1]],[[1e8+1], [0], [-1]]))



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

#Att = attitude.fEulerint(0, 0, 0, 0.1, 1, 1, 1)
#print(Att)

#Rvb = attitude.fEulerRvb(0, 0, 0, 0.1, 1, 1, 1)
#print(Rvb)

#Rvb2 = attitude.fexpo(0, 0, 0, 0.1, 1, 1, 1)
#print(Rvb2)

bp = attitude.bestpossible(0, 0, 0, 0.1, 1, 1, 1)
#%% Euler2dcm():
print("Beginning testing of Rotations.Euler2dcm()")

cur_test = "Euler2dcm yaw test 1"
#we know that rotating [1,0,0] by 90 degrees about Z should produce [0,-1,0], so
R = Rotations.euler2DCM(90*math.pi/180, 0, 0)
orig_vec = [[1], [0], [0]]
expected_vec = [[0], [-1], [0]]
actual_vec = mm.multiply(R, orig_vec)
if not evaluateTest(cur_test, compareVectors(expected_vec, actual_vec) ):
	print(f"{expected_vec} != {actual_vec}")

eul_test = "Euler2dcm yaw test 2"
Ro = Rotations.euler2DCM(-18.43*math.pi/180, 0, 0)
mul_vec = [[0], [1], [0]]
right_vec = [[-0.316145], [0.9487106], [0]]
ac_vec = mm.multiply(Ro, mul_vec)
if not evaluateTest(eul_test, compareVectors(right_vec, ac_vec)):
	print(f"{right_vec} != {ac_vec}")

#%%  dcm2euler():
print("Beginning testing 1 of Rotations.dcm2euler()")
dmc_test = "dcm2euler test 1"
ypr1 = Rotations.dcm2Euler([[0.94, -0.31, 0], [0.31, 0.94, 0], [0, 0, 1]])
expected_euler = [[-0.318555], [0], [0]]
Rot = [[ypr1[0]], [ypr1[1]], [ypr1[2]]]
if not evaluateTest(dmc_test, compareVectors(expected_euler, Rot)):
	print(f"{expected_euler} != {Rot}")

print("Beginning testing 2 of Rotations.dcm2euler()")

"""
Students, add more tests here.  
You aren't required to use the testing framework we've started here, 
but it will work just fine.
"""

#%%  ned2enu():
print("Beginning testing of Rotations.ned2enu()")
ned_test1 = "ned2enu test 1"
Ned1 = Rotations.ned2enu([[0, 1, 0], [1, 0, 0], [0, 0, 1]])
expected_enu = [[1], [0], [1]]
test_m = [[1], [0], [-1]]
ac_mat = mm.multiply(Ned1, test_m)
if not evaluateTest(ned_test1, compareVectors(expected_enu, ac_mat)):
	print(f"{expected_enu} != {ac_mat}")

print("Beginning testing 2 of Rotations.ned2enu()")
ned_test2 = "ned2enu test 2"
Ned2 = Rotations.ned2enu([[1, 0, -1], [1, 0, 1], [-1, 0, -1]])
expected_enu2 = [[1], [-1], [1]]
test_m2 = [[0], [0], [1]]
ac_mat2 = mm.multiply(Ned2, test_m2)
if not evaluateTest(ned_test2, compareVectors(ac_mat2, expected_enu2)):
	print(f"{expected_enu2} != {expected_enu2}")

#%% Print results:

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]