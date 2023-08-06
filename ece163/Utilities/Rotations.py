import copy
import math
from . import MatrixMath

def dcm2Euler(dcm):
    yaw = 0
    pitch = 0
    roll = 0
    pitch_in = 0
    yaw = math.atan2(dcm[0][1], dcm[0][0])
    if dcm[0][2] > 1:
        pitch_in = 1
    elif dcm[0][2] < -1:
        pitch_in = -1
    else:
        pitch_in = dcm[0][2]
    pitch = -1 * math.asin(pitch_in)
    roll = math.atan2(dcm[1][2], dcm[2][2])

    return (yaw, pitch, roll)

def euler2DCM(yaw, pitch, roll):
    cya = math.cos(yaw)
    cpi = math.cos(pitch)
    cro = math.cos(roll)
    sya = math.sin(yaw)
    spi = math.sin(pitch)
    sro = math.sin(roll)
    dcm1 = cpi * cya
    dcm2 = cpi * sya
    dcm3 = -spi
    dcm4 = (sro * spi * cya) - (cro * sya)
    dcm5 = (sro * spi * sya) + (cro * cya)
    dcm6 = sro * cpi
    dcm7 = (cro * spi * cya) + (sro * sya)
    dcm8 = (cro * spi * sya) - (sro * cya)
    dcm9 = cro * cpi
    dcm = [[dcm1, dcm2, dcm3], [dcm4, dcm5, dcm6],
           [dcm7, dcm8, dcm9]]
    return dcm

def ned2enu(points):
    new_n = 0
    for i in points:
        new_n = i[0]
        i[0] = i[1]
        i[1] = new_n
        i[2] *= -1
    #print(points)
    return points

