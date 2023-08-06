import ece163.Utilities.MatrixMath as MatrixMath
import ece163.Constants.VehiclePhysicalConstants as VPC
from matplotlib import pyplot as plt
import math
import numpy as np
import random

# 2A
T = 400
sigma = 0.0013
dT = .001
p = 16
step = int(p / dT)
zero = np.zeros(step)

for i in range(1, step):
    gauss = random.gauss(0, sigma)
    temp = zero[i - 1] + gauss
    zero[i] = np.exp(-dT / T) * temp

# tstep = np.linspace(0, p, step)
# plt.plot(tstep, zero)
# plt.xlabel('Time')
# plt.ylabel('Gauss-Markov Process')
# plt.show()

# 2B
cndeltar = -0.069
cnbeta = 0.073
A = [[-0.79, -4.2], [1, 0]]
B = [[1], [0]]
C = [[0, 4.2]]
td = []
for i in range(step):
    td.append(dT * i)
yaw = np.zeros(step)
yawe = np.zeros(step)
wd = []
for i in td:
    if i < 1:
        wd += [0]
    else:
        wd += [10 * math.pi / 180]
mat = [[0], [0]]
for i in range(step):
    u = (cndeltar / cnbeta) + wd[i]
    Amat = MatrixMath.multiply(A, mat)
    scub = MatrixMath.scalarMultiply(u, B)
    xdot = MatrixMath.add(Amat, scub)
    xdt = MatrixMath.scalarMultiply(dT, xdot)
    mat = MatrixMath.add(mat, xdt)
    t = MatrixMath.multiply(C, mat)
    yaw[i] = t[0][0]
    yawe[i] = yaw[i] + zero[i]

fig, ax = plt.subplots()
ax.plot(td, yaw, label='Yaw Response')
ax.plot(td, yawe, label='Yaw Estimate')
ax.set_xlabel("Time")
ax.set_ylabel("Angle")
plt.show()

# 4B
d2r = math.pi / 180.0
RE = 6378137.0
ref = [[36.94937 * d2r], [122.06538 * d2r], [2.3 * d2r]]
P = [[36.95823 * d2r], [122.0176 * d2r], [-42.0 * d2r]]
sphi = math.sin(ref[0][0])
slam = math.sin(ref[1][0])
cphi = math.cos(ref[0][0])
clam = math.cos(ref[1][0])
TEN = [[-sphi * clam, -sphi * slam, cphi], [-slam, clam, 0], [-cphi * clam, -cphi * slam, -sphi]]
ECEF = [[(RE + P[2][0]) * cphi * clam], [(RE + P[2][0]) * cphi * slam], [(RE + P[2][0]) * sphi]]
PTned1 = MatrixMath.multiply(TEN, ECEF)
PTned2 = MatrixMath.scalarMultiply(RE + ref[2][0], [[0], [0], [1]])
PTned = MatrixMath.add(PTned1, PTned2)

