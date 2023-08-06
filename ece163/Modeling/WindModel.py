import math
import random
from ..Containers import States
from ..Containers.Inputs import drydenParameters
from ..Utilities import MatrixMath
from ..Constants import VehiclePhysicalConstants as VPC


class WindModel():
    def __init__(self, dT=VPC.dT, Va=VPC.InitialSpeed, drydenParamters=VPC.DrydenNoWind):
        self.pu = [[0.0]]
        self.gu = [[0.0]]
        self.hu = [[0.0]]
        self.pv = [[0.0, 0.0], [0.0, 0.0]]
        self.gv = [[0.0], [0.0]]
        self.hv = [[0.0, 0.0]]
        self.pw = [[0.0, 0.0], [0.0, 0.0]]
        self.gw = [[0.0], [0.0]]
        self.hw = [[0.0, 0.0]]
        self.windState = States.windState()
        self.Va = Va
        self.dT = dT
        self.drydenParamters = drydenParamters
        self.CreateDrydenTransferFns(self.dT, self.Va, self.drydenParamters)
        self.xu = [[0.0]]
        self.xv = [[0.0], [0.0]]
        self.xw = [[0.0], [0.0]]
        return

    def reset(self):
        self.xu = [[0.0]]
        self.xv = [[0.0], [0.0]]
        self.xw = [[0.0], [0.0]]
        self.windState = States.windState()
        # print("Wind Model Reset")
        return

    def getWind(self):
        # print("Getting Wind")
        return self.windState

    def setWind(self, windState):
        self.windState = windState
        return

    def setWindModelParameters(self, Wn=0.0, We=0.0, Wd=0.0, drydenParamters=VPC.DrydenNoWind):
        self.CreateDrydenTransferFns(self.dT, self.Va, drydenParamters)
        self.windState.Wn = Wn
        self.windState.We = We
        self.windState.Wd = Wd
        return

    def CreateDrydenTransferFns(self, dT, Va, drydenParamters):
        if Va <= 0.0:
            raise ArithmeticError
        Lu = drydenParamters.Lu
        Lv = drydenParamters.Lv
        Lw = drydenParamters.Lw
        Su = drydenParamters.sigmau
        Sv = drydenParamters.sigmav
        Sw = drydenParamters.sigmaw
        if drydenParamters == VPC.DrydenNoWind:
            Pu = 1.0
            Gu = 0.0
            Hu = 1.0
            Pv = [[1.0, 0.0], [0.0, 1.0]]
            Gv = [[0.0], [0.0]]
            Hv = [[1.0, 1.0]]
            Pw = [[1.0, 0.0], [0.0, 1.0]]
            Gw = [[0.0], [0.0]]
            Hw = [[1.0, 1.0]]
        else:
            Pu = math.exp((-Va / Lu) * dT)
            Gu = ((Lu / Va) * (1.0 - math.exp((-Va / Lu) * dT)))
            Hu = (Su * math.sqrt((2.0 * Va) / (math.pi * Lu)))
            Pvmat = [[(1.0 - ((Va / Lv) * dT)), (-((Va / Lv) ** 2.0) * dT)],
                     [dT, (1.0 + ((Va / Lv) * dT))]]
            expv = math.exp((-Va / Lv) * dT)
            Pv = MatrixMath.scalarMultiply(expv, Pvmat)
            Gvmat = [[dT], [(((Lv / Va) ** 2.0) * ((math.exp((Va / Lv) * dT)) - 1.0)) - ((Lv / Va) * dT)]]
            Gv = MatrixMath.scalarMultiply(expv, Gvmat)
            Hvsc = Sv * math.sqrt((3.0 * Va) / (math.pi * Lv))
            Hv = MatrixMath.scalarMultiply(Hvsc, [[1.0, (Va / (math.sqrt(3.0) * Lv))]])
            Pwmat = [[(1.0 - ((Va / Lw) * dT)), (-((Va / Lw) ** 2.0) * dT)],
                     [dT, (1.0 + ((Va / Lw) * dT))]]
            expw = math.exp((-Va / Lw) * dT)
            Pw = MatrixMath.scalarMultiply(expw, Pwmat)
            Gwmat = [[dT], [(((Lw / Va) ** 2.0) * ((math.exp((Va / Lw) * dT)) - 1.0)) - ((Lw / Va) * dT)]]
            Gw = MatrixMath.scalarMultiply(expw, Gwmat)
            Hwsc = Sw * math.sqrt((3.0 * Va) / (math.pi * Lw))
            Hw = MatrixMath.scalarMultiply(Hwsc, [[1.0, (Va / (math.sqrt(3.0) * Lw))]])

        self.pu = [[Pu]]
        self.gu = [[Gu]]
        self.hu = [[Hu]]
        self.pv = Pv
        self.gv = Gv
        self.hv = Hv
        self.pw = Pw
        self.gw = Gw
        self.hw = Hw
        return

    def getDrydenTransferFns(self):
        return self.pu, self.gu, self.hu, self.pv, self.gv, self.hv, self.pw, self.gw, self.hw

    def Update(self, uu=None, uv=None, uw=None):
        if uu is None:
            uu = random.gauss(0, 1)
        if uv is None:
            uv = random.gauss(0, 1)
        if uw is None:
            uw = random.gauss(0, 1)
        pumul = MatrixMath.multiply(self.pu, self.xu)
        gumul = MatrixMath.scalarMultiply(uu, self.gu)
        xuplus = MatrixMath.add(pumul, gumul)
        self.windState.Wu = MatrixMath.multiply(self.hu, xuplus)[0][0]
        self.xu = xuplus
        pvmul = MatrixMath.multiply(self.pv, self.xv)
        gvmul = MatrixMath.scalarMultiply(uv, self.gv)
        xvplus = MatrixMath.add(pvmul, gvmul)
        self.windState.Wv = MatrixMath.multiply(self.hv, xvplus)[0][0]
        self.xv = xvplus
        pwmul = MatrixMath.multiply(self.pw, self.xw)
        gwmul = MatrixMath.scalarMultiply(uw, self.gw)
        xwplus = MatrixMath.add(pwmul, gwmul)
        self.windState.Ww = MatrixMath.multiply(self.hw, xwplus)[0][0]
        self.xw = xwplus
        return
