import math
import sys
import ece163.Containers.Inputs as Inputs
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import random
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule
import ece163.Utilities.MatrixMath as MM
import numpy as np

class PDControl:
    def __init__(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return

    def Update(self, command=0.0, current=0.0, derivative=0.0):
        error = command - current
        inp = (self.kp * error) - (self.kd * derivative) + self.trim
        if inp > self.highLimit:
            out = self.highLimit
        elif inp < self.lowLimit:
            out = self.lowLimit
        else:
            out = inp
        return out

    def setPDGains(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return


class PIControl:
    def __init__(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0.0
        self.err = 0.0
        return

    def Update(self, command=0.0, current=0.0):
        error = command - current
        self.accumulator += (self.dT / 2.0) * (error + self.err)
        inp = (self.kp * error) + (self.ki * self.accumulator) + self.trim
        if inp > self.highLimit:
            out = self.highLimit
            self.accumulator -= (self.dT / 2.0) * (error + self.err)
        elif inp < self.lowLimit:
            out = self.lowLimit
            self.accumulator -= (self.dT / 2.0) * (error + self.err)
        else:
            out = inp
        self.err = error
        return out

    def resetIntegrator(self):
        self.accumulator = 0.0
        self.err = 0.0
        return

    def setPIGains(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return


class PIDControl:
    def __init__(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0.0
        self.err = 0.0
        return

    def Update(self, command=0.0, current=0.0, derivative=0.0):
        error = command - current
        self.accumulator += (self.dT / 2.0) * (error + self.err)
        inp = (self.kp * error) + (self.ki * self.accumulator) - (self.kd * derivative) + self.trim
        if inp > self.highLimit:
            out = self.highLimit
            self.accumulator -= (self.dT / 2.0) * (error + self.err)
        elif inp < self.lowLimit:
            out = self.lowLimit
            self.accumulator -= (self.dT / 2.0) * (error + self.err)
        else:
            out = inp
        self.err = error
        return out

    def resetIntegrator(self):
        self.accumulator = 0.0
        self.err = 0.0
        return

    def setPIDGains(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return


def PathFollow(r, q, p, X, state, Xinf=math.pi / 2.0, kpath=0.001):
    epi = MM.subtract(p, r)
    ki = [[0], [0], [1]]
    topn = MM.crossProduct(q, ki)
    magq = math.hypot(q[0][0], q[1][0], q[2][0])
    magk = math.hypot(ki[0][0], ki[1][0], ki[2][0])
    dot = (q[0][0] * ki[0][0]) + (q[1][0] * ki[1][0]) + (q[2][0] * ki[2][0])
    theta = math.acos(dot / (magk * magq))
    botn = magq * magk * abs(math.sin(theta))
    n = MM.scalarDivide(botn, topn)
    epin = MM.dotProduct(epi, n)
    s2 = MM.scalarMultiply(epin[0][0], n)
    si = MM.subtract(epi, s2)
    hd = -r[2][0] - (math.hypot(si[0][0], si[1][0]) * (q[2][0] / math.hypot(q[0][0], q[1][0])))
    Xq = math.atan2(q[1][0], q[0][0])
    #print("Xq before windup windown:",Xq)
    if Xq - X < -math.pi:
        Xq += 2 * math.pi
    if Xq - X > math.pi:
        Xq -= 2 * math.pi
    #print("Xq after windup windown:",Xq)
    epy = -math.sin(Xq) * (p[0][0] - r[0][0]) + math.cos(Xq) * (p[1][0] - r[1][0])
    # epy = state.Va * math.sin(X-Xq)
    Xc = Xq - (Xinf * (2.0 / math.pi) * math.atan(kpath * epy))
    #print("Xc :",Xc)
    #print("hd: ", hd)
    return hd, Xc


def Waypoint(p, W):
    if not hasattr(Waypoint, 'i'):
        Waypoint.i = 1
    if Waypoint.i >= len(W):
        return None, None
    r = W[Waypoint.i - 1]  # Setting the previous point
    qionet = MM.subtract([W[Waypoint.i]], [W[Waypoint.i - 1]])
    magw = np.linalg.norm(qionet)
    qione = MM.scalarDivide(magw, qionet)
    qit = MM.subtract([W[Waypoint.i + 1]], [W[Waypoint.i]])
    magw = np.linalg.norm(qit)
    qi = MM.scalarDivide(magw, qit)
    nit = MM.add(qione, qi)
    magq = np.linalg.norm(nit)
    ni = MM.scalarDivide(magq, nit)
    H = 60.0
    pW = MM.subtract([p], [W[Waypoint.i]])
    pWn = MM.subtract([p], MM.add([W[Waypoint.i]], ni))
    if np.linalg.norm(pW) < H and np.linalg.norm(pWn) < H:
        Waypoint.i += 1
    return r, qione


class VehicleClosedLoopControl:
    def __init__(self, dT=VPC.dT, rudderControlSource='SIDESLIP'):
        self.amodel = VehicleAerodynamicsModule.VehicleAerodynamicsModel()
        self.dT = dT
        self.mode = Controls.AltitudeStates.HOLDING
        self.cgains = Controls.controlGains()
        self.triminput = Inputs.controlInputs()
        self.aeroinput = Inputs.controlInputs()
        self.rudderControlSource = rudderControlSource
        self.rollFromCourse = PIControl()
        self.rudderFromSideslip = PIControl()
        self.throttleFromAirspeed = PIControl()
        self.pitchFromAltitude = PIControl()
        self.pitchFromAirspeed = PIControl()
        self.elevatorFromPitch = PDControl()
        self.aileronFromRoll = PIDControl()
        self.W = [[1000.0, 0.0, -100.0], [1200.0, -60.0, -100.0], [1000.0, -160.0, -150.0], [3000.0, 2000.0, -400.0],
             [4500.0, 0.0, -1000.0], [7000.0, 0.0, -1500.0]]
        return
    
    def setWaypoints(self, points):
        self.W = points
        return

    def Update(self, referenceCommands=Controls.referenceCommands):
        state = self.amodel.getVehicleState()
        newcommands = Controls.referenceCommands()
        p = [self.amodel.model.state.pn, self.amodel.model.state.pe, self.amodel.model.state.pd]
        pt = [[self.amodel.model.state.pn], [self.amodel.model.state.pe], [-self.amodel.model.state.pd]]
       # W = [[1000.0, 0.0, -100.0], [1000.0, .0, -100.0], [2200.0, 1050.0, -100.0], [3000.0, 2000.0, -400.0],
        #     [4500.0, 0.0, -1000.0], [7000.0, 0.0, -1500.0]]
        RQ = Waypoint(p, self.W)
        qT = RQ[1]
        q = [[qT[0][0]], [qT[0][1]], [qT[0][2]]]
        X = self.amodel.model.state.chi
        r = [[RQ[0][0]], [RQ[0][1]], [RQ[0][2]]]
        hX = PathFollow(r, q, pt, X, state, math.pi / 2.0, 0.01)
        newcommands.commandedAltitude = hX[0]
        newcommands.commandedPitch = referenceCommands.commandedPitch
        newcommands.commandedAirspeed = referenceCommands.commandedAirspeed
        newcommands.commandedRoll = referenceCommands.commandedRoll
        newcommands.commandedCourse = hX[1]
        controls = self.UpdateControlCommands(newcommands, state)

        self.amodel.Update(controls)
        return

    def UpdateControlCommands(self, referenceCommands, state):
        controlinput = Inputs.controlInputs()
        altitude = -state.pd
        if (referenceCommands.commandedCourse - state.chi) >= math.pi:
            state.chi += (2 * math.pi)
        elif (referenceCommands.commandedCourse - state.chi) <= -math.pi:
            state.chi -= (2 * math.pi)
        upper_thresh = referenceCommands.commandedAltitude + VPC.altitudeHoldZone
        lower_thresh = referenceCommands.commandedAltitude - VPC.altitudeHoldZone
        if self.mode == Controls.AltitudeStates.HOLDING:
            controlinput.Throttle = self.throttleFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
            referenceCommands.commandedPitch = \
                self.pitchFromAltitude.Update(referenceCommands.commandedAltitude, altitude)
            if altitude > upper_thresh:
                self.pitchFromAirspeed.resetIntegrator()
                self.mode = Controls.AltitudeStates.DESCENDING
                referenceCommands.commandedPitch = \
                    self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
                controlinput.Throttle = VPC.minControls.Throttle
            if altitude < lower_thresh:
                self.pitchFromAirspeed.resetIntegrator()
                self.mode = Controls.AltitudeStates.CLIMBING
                controlinput.Throttle = VPC.maxControls.Throttle
                referenceCommands.commandedPitch = \
                    self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
        elif self.mode == Controls.AltitudeStates.DESCENDING:
            referenceCommands.commandedPitch = \
                self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
            controlinput.Throttle = VPC.minControls.Throttle
            if lower_thresh < altitude < upper_thresh:
                self.pitchFromAltitude.resetIntegrator()
                self.mode = Controls.AltitudeStates.HOLDING
                controlinput.Throttle = self.throttleFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
                referenceCommands.commandedPitch = \
                    self.pitchFromAltitude.Update(referenceCommands.commandedAltitude, altitude)
        elif self.mode == Controls.AltitudeStates.CLIMBING:
            controlinput.Throttle = VPC.maxControls.Throttle
            referenceCommands.commandedPitch = \
                self.pitchFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
            if lower_thresh < altitude < upper_thresh:
                self.pitchFromAltitude.resetIntegrator()
                self.mode = Controls.AltitudeStates.HOLDING
                controlinput.Throttle = self.throttleFromAirspeed.Update(referenceCommands.commandedAirspeed, state.Va)
                referenceCommands.commandedPitch = \
                    self.pitchFromAltitude.Update(referenceCommands.commandedAltitude, altitude)
        # print([referenceCommands.commandedCourse, state.chi])
        referenceCommands.commandedRoll = self.rollFromCourse.Update(referenceCommands.commandedCourse, state.chi)
        controlinput.Aileron = self.aileronFromRoll.Update(referenceCommands.commandedRoll, state.roll, state.p)
        controlinput.Rudder = self.rudderFromSideslip.Update(0.0, state.beta)
        controlinput.Elevator = self.elevatorFromPitch.Update(referenceCommands.commandedPitch, state.pitch, state.q)
        return controlinput

    def getControlGains(self):
        return self.cgains

    def getTrimInputs(self):
        return self.triminput

    def getVehicleAerodynamicsModel(self):
        return self.amodel

    def getVehicleControlSurfaces(self):
        return self.aeroinput

    def getVehicleState(self):
        return self.amodel.model.state

    def reset(self):
        self.amodel.reset()
        self.rollFromCourse.resetIntegrator()
        self.rudderFromSideslip.resetIntegrator()
        self.throttleFromAirspeed.resetIntegrator()
        self.pitchFromAltitude.resetIntegrator()
        self.pitchFromAirspeed.resetIntegrator()
        self.aileronFromRoll.resetIntegrator()
        return

    def setControlGains(self, controlGains=Controls.controlGains()):
        self.cgains = controlGains
        minc = VPC.minControls
        maxc = VPC.maxControls
        self.rollFromCourse.setPIGains(self.dT, self.cgains.kp_course,
                                       self.cgains.ki_course, 0.0,
                                       -math.radians(VPC.bankAngleLimit), math.radians(VPC.bankAngleLimit))
        self.rudderFromSideslip.setPIGains(self.dT, self.cgains.kp_sideslip, self.cgains.ki_sideslip,
                                           self.triminput.Rudder, minc.Rudder, maxc.Rudder)
        self.throttleFromAirspeed.setPIGains(self.dT, self.cgains.kp_SpeedfromThrottle,
                                             self.cgains.ki_SpeedfromThrottle, self.triminput.Throttle,
                                             minc.Throttle, maxc.Throttle)
        self.pitchFromAltitude.setPIGains(self.dT, self.cgains.kp_altitude, self.cgains.ki_altitude,
                                          0.0, -math.radians(VPC.pitchAngleLimit), math.radians(VPC.pitchAngleLimit))
        self.pitchFromAirspeed.setPIGains(self.dT, self.cgains.kp_SpeedfromElevator, self.cgains.ki_SpeedfromElevator,
                                          0.0, -math.radians(VPC.pitchAngleLimit), math.radians(VPC.pitchAngleLimit))
        self.elevatorFromPitch.setPDGains(self.cgains.kp_pitch, self.cgains.kd_pitch,
                                          self.triminput.Elevator, minc.Elevator, maxc.Elevator)
        self.aileronFromRoll.setPIDGains(self.dT, self.cgains.kp_roll, self.cgains.kd_roll, 0.001,
                                         self.triminput.Aileron, minc.Aileron, maxc.Aileron)
        return

    def setTrimInputs(self, trimInputs=Inputs.controlInputs(Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0)):
        self.triminput = trimInputs
        return

    def setVehicleState(self, state):
        self.amodel.model.state = state
        return
