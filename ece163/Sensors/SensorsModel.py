import math
import random
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Utilities import MatrixMath
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleAerodynamicsModel


class GaussMarkov():
    def __init__(self, dT=VPC.dT, tau=1000000, eta=0.0):
        self.dT = dT
        self.tau = tau
        self.eta = eta
        self.v = 0.0
        return

    def reset(self):
        self.v = 0.0
        return

    def update(self, vnoise=None):
        if vnoise is None:
            vnoise = random.gauss(0, self.eta)
        step = ((math.exp(-self.dT / self.tau)) * self.v) + vnoise
        self.v = step
        return self.v


class GaussMarkovXYZ():
    def __init__(self, dT=VPC.dT, tauX=1000000.0, etaX=0.0, tauY=None, etaY=None, tauZ=None, etaZ=None):
        if tauY is None:
            if etaY is None:
                tauY = tauX
                etaY = etaX
                if tauZ is None:
                    if etaZ is None:
                        tauZ = tauX
                        etaZ = etaX
        else:
            if tauZ is None:
                if etaZ is None:
                    tauZ = tauY
                    etaZ = etaY
        self.GaussX = GaussMarkov(dT, tauX, etaX)
        self.GaussY = GaussMarkov(dT, tauY, etaY)
        self.GaussZ = GaussMarkov(dT, tauZ, etaZ)
        return

    def reset(self):
        self.GaussX.reset()
        self.GaussY.reset()
        self.GaussZ.reset()
        return

    def update(self, vXnoise=None, vYnoise=None, vZnoise=None):
        updateX = self.GaussX.update(vXnoise)
        updateY = self.GaussY.update(vYnoise)
        updateZ = self.GaussZ.update(vZnoise)
        return updateX, updateY, updateZ


class SensorsModel():
    def __init__(self, taugyro=400.0, etagyro=0.0012740903539558606, tauGPS=1100.0,
                 etaGPSHorizontal=0.21, etaGPSVertical=0.4, gpsUpdateHz=1.0):
        self.aero = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
        self.model = self.aero.getVehicleDynamicsModel()
        self.dT = self.model.dT
        self.strue = Sensors.vehicleSensors()
        self.snoisy = Sensors.vehicleSensors()
        self.sbias = self.initializeBiases()
        self.ssigmas = self.initializeSigmas()
        self.gpsUpdateHz = gpsUpdateHz
        self.gps = GaussMarkovXYZ(dT=1.0 / gpsUpdateHz, tauX=tauGPS,
                                  etaX=etaGPSHorizontal, tauZ=tauGPS, etaZ=etaGPSVertical)
        self.gyro = GaussMarkovXYZ(self.dT, taugyro, etagyro)
        self.ticks = 0.0
        return

    def getSensorsNoisy(self):
        return self.snoisy

    def getSensorsTrue(self):
        return self.strue

    def initializeBiases(self, gyroBias=0.08726646259971647, accelBias=0.9810000000000001,
                         magBias=500.0, baroBias=100.0, pitotBias=20.0):
        sbias = Sensors.vehicleSensors()
        sbias.gyro_x = gyroBias * random.uniform(-1, 1)
        sbias.gyro_y = gyroBias * random.uniform(-1, 1)
        sbias.gyro_z = gyroBias * random.uniform(-1, 1)
        sbias.accel_x = accelBias * random.uniform(-1, 1)
        sbias.accel_y = accelBias * random.uniform(-1, 1)
        sbias.accel_z = accelBias * random.uniform(-1, 1)
        sbias.mag_x = magBias * random.uniform(-1, 1)
        sbias.mag_y = magBias * random.uniform(-1, 1)
        sbias.mag_z = magBias * random.uniform(-1, 1)
        sbias.baro = baroBias * random.uniform(-1, 1)
        sbias.pitot = pitotBias * random.uniform(-1, 1)
        return sbias

    def initializeSigmas(self, gyroSigma=0.002617993877991494, accelSigma=0.24525000000000002,
                         magSigma=25.0, baroSigma=10.0, pitotSigma=2.0, gpsSigmaHorizontal=0.4,
                         gpsSigmaVertical=0.7, gpsSigmaSOG=0.05, gpsSigmaCOG=0.002):
        ssigmas = Sensors.vehicleSensors()
        ssigmas.gyro_x = gyroSigma
        ssigmas.gyro_y = gyroSigma
        ssigmas.gyro_z = gyroSigma
        ssigmas.accel_x = accelSigma
        ssigmas.accel_y = accelSigma
        ssigmas.accel_z = accelSigma
        ssigmas.mag_x = magSigma
        ssigmas.mag_y = magSigma
        ssigmas.mag_z = magSigma
        ssigmas.baro = baroSigma
        ssigmas.pitot = pitotSigma
        ssigmas.gps_n = gpsSigmaHorizontal
        ssigmas.gps_e = gpsSigmaHorizontal
        ssigmas.gps_alt = gpsSigmaVertical
        ssigmas.gps_sog = gpsSigmaSOG
        ssigmas.gps_cog = gpsSigmaCOG
        return ssigmas

    def reset(self):
        self.strue = Sensors.vehicleSensors()
        self.snoisy = Sensors.vehicleSensors()
        self.sbias = self.initializeBiases()
        self.ssigmas = self.initializeSigmas()
        self.gps.reset()
        self.gyro.reset()
        self.ticks = 0.0
        return

    def update(self):
        self.ticks += 1.0
        prev = self.getSensorsTrue()
        strue = self.updateSensorsTrue(prev, self.model.state, self.model.dot)
        self.strue = strue
        getsnoisy = self.getSensorsNoisy()
        snoisy = self.updateSensorsNoisy(strue, getsnoisy, self.sbias, self.ssigmas)
        self.snoisy = snoisy
        return

    def updateAccelsTrue(self, state, dot):
        accel = [0, 0, 0]
        skew = MatrixMath.skew(state.p, state.q, state.r)
        scuvw = MatrixMath.multiply(skew, [[state.u], [state.v], [state.w]])
        duvw = MatrixMath.add([[dot.u], [dot.v], [dot.w]], scuvw)
        Rg = MatrixMath.multiply(state.R, [[0], [0], [VPC.g0]])
        atrue = MatrixMath.subtract(duvw, Rg)
        accel[0] = atrue[0][0]
        accel[1] = atrue[1][0]
        accel[2] = atrue[2][0]
        return accel

    def updateGPSTrue(self, state, dot):
        sog = math.hypot(dot.pn, dot.pe)
        cog = math.atan2(dot.pe, dot.pn)
        return state.pn, state.pe, -state.pd, sog, cog

    def updateGyrosTrue(self, state):
        return state.p, state.q, state.r

    def updateMagsTrue(self, state):
        mag = MatrixMath.multiply(state.R, VSC.magfield)
        return mag[0][0], mag[1][0], mag[2][0]

    def updatePressureSensorsTrue(self, state):
        baro = VSC.Pground + (VPC.rho * VPC.g0 * state.pd)
        pitot = VPC.rho * ((state.Va ** 2) / 2.0)
        return baro, pitot

    def updateSensorsNoisy(self, trueSensors=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,
                                                                    accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0,
                                                                    mag_z=0.0, baro=0.0, pitot=0.0,
                                                                    gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0,
                                                                    gps_cog=0.0),
                           noisySensors=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,
                                                               accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0,
                                                               mag_z=0.0,
                                                               baro=0.0, pitot=0.0,
                                                               gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0,
                                                               gps_cog=0.0),
                           sensorBiases=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,
                                                               accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0,
                                                               mag_z=0.0,
                                                               baro=0.0, pitot=0.0,
                                                               gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0,
                                                               gps_cog=0.0),
                           sensorSigmas=Sensors.vehicleSensors(gyro_x=0.0, gyro_y=0.0, gyro_z=0.0, accel_x=0.0,
                                                               accel_y=0.0, accel_z=0.0, mag_x=0.0, mag_y=0.0,
                                                               mag_z=0.0,
                                                               baro=0.0, pitot=0.0,
                                                               gps_n=0.0, gps_e=0.0, gps_alt=0.0, gps_sog=0.0,
                                                               gps_cog=0.0)):
        snoisy = Sensors.vehicleSensors()

        return snoisy

    def updateSensorsTrue(self, prevTrueSensors, state, dot):
        self.ticks += 1.0
        strue = Sensors.vehicleSensors()
        accel = self.updateAccelsTrue(state, dot)
        strue.accel_x = accel[0]
        strue.accel_y = accel[1]
        strue.accel_z = accel[2]
        pressure = self.updatePressureSensorsTrue(state)
        strue.baro = pressure[0]
        strue.pitot = pressure[1]
        mag = self.updateMagsTrue(state)
        strue.mag_x = mag[0]
        strue.mag_y = mag[1]
        strue.mag_z = mag[2]
        gyro = self.updateGyrosTrue(state)
        strue.gyro_x = gyro[0]
        strue.gyro_y = gyro[1]
        strue.gyro_z = gyro[2]
        if self.ticks % self.gpsUpdateHz:
            strue.gps_n = prevTrueSensors.gps_n
            strue.gps_e = prevTrueSensors.gps_e
            strue.gps_alt = prevTrueSensors.gps_alt
            strue.gps_sog = prevTrueSensors.gps_sog
            strue.gps_cog = prevTrueSensors.gps_cog
        else:
            gps = self.updateGPSTrue(state, dot)
            strue.gps_n = gps[0]
            strue.gps_e = gps[1]
            strue.gps_alt = gps[2]
            strue.gps_sog = gps[3]
            strue.gps_cog = gps[4]
        return strue
