import math
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath


def CreateTransferFunction(trimState, trimInputs):
    # print("Create TF")
    trimuvw = math.hypot(trimState.u, trimState.v, trimState.w)
    if math.isclose(trimuvw, 0.0):
        beta = 0.0
    else:
        beta = trimState.beta
    gtrim = trimState.pitch - trimState.alpha
    ptrim = trimState.roll
    deltaE = trimInputs.Elevator
    Throttle = trimInputs.Throttle
    aphil1 = (-1.0 / 2.0) * (VPC.rho * (trimuvw ** 2.0) * VPC.S *
                             VPC.b * VPC.Cpp * (VPC.b / (2.0 * trimuvw)))
    aphil2 = (1.0 / 2.0) * (VPC.rho * (trimuvw ** 2.0) * VPC.S *
                            VPC.b * VPC.CpdeltaA)
    at1 = (((-1.0 * VPC.rho) * (trimuvw ** 2.0) * VPC.S * VPC.c) /
           (2.0 * VPC.Jyy)) * ((VPC.CMq * VPC.c) / (2.0 * trimuvw))
    at2 = (((-1.0 * VPC.rho) * (trimuvw ** 2.0) * VPC.S * VPC.c) /
           (2.0 * VPC.Jyy)) * VPC.CMalpha
    at3 = ((VPC.rho * (trimuvw ** 2.0) * VPC.S * VPC.c) /
           (2.0 * VPC.Jyy)) * VPC.CMdeltaE
    ab1 = ((-1.0 * VPC.rho * trimuvw * VPC.S) /
           (2.0 * VPC.mass)) * VPC.CYbeta
    ab2 = ((VPC.rho * trimuvw * VPC.S) /
           (2.0 * VPC.mass)) * VPC.CYdeltaR
    av1 = ((VPC.rho * trimuvw * VPC.S) / VPC.mass) * \
          (VPC.CD0 + (VPC.CDalpha * trimState.alpha) +
           (VPC.CDdeltaE * deltaE)) - ((1.0 / VPC.mass) * dThrust_dVa(trimuvw, Throttle))
    av2 = ((1.0 / VPC.mass) * dThrust_dThrottle(trimuvw, Throttle))
    av3 = VPC.g0 * math.cos(gtrim)
    tf = Linearized.transferFunctions(trimuvw, trimState.alpha, beta, gtrim, trimState.pitch,
                                      ptrim, aphil1, aphil2, ab1, ab2, at1, at2, at3, av1, av2, av3)

    return tf


def dThrust_dThrottle(Va, Throttle, epsilon=0.01):
    model = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    Fx = model.CalculatePropForces(Va, Throttle)
    Fxplus = model.CalculatePropForces(Va, Throttle + epsilon)
    dTdT = (Fxplus[0] - Fx[0]) / epsilon
    return dTdT


def dThrust_dVa(Va, Throttle, epsilon=0.5):
    model = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    Fx = model.CalculatePropForces(Va, Throttle)
    Fxplus = model.CalculatePropForces(Va + epsilon, Throttle)
    dTdV = (Fxplus[0] - Fx[0]) / epsilon
    return dTdV
