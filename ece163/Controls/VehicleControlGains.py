import math
import pickle
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Controls
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Utilities import Rotations


def computeGains(tuningParameters, linearizedModel):
    cgains = Controls.controlGains()
    cgains.kp_roll = (tuningParameters.Wn_roll ** 2) / linearizedModel.a_phi2
    cgains.kd_roll = (2 * tuningParameters.Zeta_roll * tuningParameters.Wn_roll - linearizedModel.a_phi1) / \
                     linearizedModel.a_phi2
    cgains.ki_roll = 0.001
    cgains.kp_sideslip = (2 * tuningParameters.Zeta_sideslip * tuningParameters.Wn_sideslip - linearizedModel.a_beta1) \
                         / linearizedModel.a_beta2
    cgains.ki_sideslip = (1.0 / linearizedModel.a_beta2) * \
                         (((linearizedModel.a_beta1 + linearizedModel.a_beta2 * cgains.kp_sideslip) /
                           (2 * tuningParameters.Zeta_sideslip)) ** 2)
    cgains.kp_course = (2 * tuningParameters.Zeta_course * tuningParameters.Wn_course * linearizedModel.Va_trim) / \
                       VPC.g0
    cgains.ki_course = ((tuningParameters.Wn_course ** 2) * linearizedModel.Va_trim) / VPC.g0
    cgains.kp_pitch = ((tuningParameters.Wn_pitch ** 2) - linearizedModel.a_theta2) / linearizedModel.a_theta3
    cgains.kd_pitch = (2 * tuningParameters.Zeta_pitch * tuningParameters.Wn_pitch - linearizedModel.a_theta1) / \
                      linearizedModel.a_theta3
    kthdc = (cgains.kp_pitch * linearizedModel.a_theta3) / \
            (linearizedModel.a_theta2 + (cgains.kp_pitch * linearizedModel.a_theta3))
    cgains.kp_altitude = (2 * tuningParameters.Zeta_altitude * tuningParameters.Wn_altitude) / \
                         (kthdc * linearizedModel.Va_trim)
    cgains.ki_altitude = (tuningParameters.Wn_altitude ** 2) / (kthdc * linearizedModel.Va_trim)
    cgains.kp_SpeedfromThrottle = ((2 * tuningParameters.Wn_SpeedfromThrottle *
                                    tuningParameters.Zeta_SpeedfromThrottle) -
                                   linearizedModel.a_V1) / linearizedModel.a_V2
    cgains.ki_SpeedfromThrottle = (tuningParameters.Wn_SpeedfromThrottle ** 2) / linearizedModel.a_V2
    cgains.kp_SpeedfromElevator = (linearizedModel.a_V1 - (2 * tuningParameters.Zeta_SpeedfromElevator *
                                                           tuningParameters.Wn_SpeedfromElevator)) / (kthdc * VPC.g0)
    cgains.ki_SpeedfromElevator = (-tuningParameters.Wn_SpeedfromElevator ** 2) / (kthdc * VPC.g0)
    return cgains


def computeTuningParameters(controlGains, linearizedModel):
    ctuning = Controls.controlTuning()
    ctuning.Wn_roll = math.sqrt(controlGains.kp_roll * linearizedModel.a_phi2)
    ctuning.Zeta_roll = (linearizedModel.a_phi1 + (linearizedModel.a_phi2 * controlGains.kd_roll)) / \
                        (2 * ctuning.Wn_roll)
    ctuning.Wn_course = math.sqrt((controlGains.ki_course * VPC.g0) / linearizedModel.Va_trim)
    ctuning.Zeta_course = ((controlGains.kp_course * VPC.g0) / linearizedModel.Va_trim) / (2 * ctuning.Wn_course)
    ctuning.Wn_sideslip = math.sqrt(controlGains.ki_sideslip * linearizedModel.a_beta2)
    ctuning.Zeta_sideslip = (linearizedModel.a_beta1 + (linearizedModel.a_beta2 * controlGains.kp_sideslip)) / \
                            (ctuning.Wn_sideslip * 2)
    ctuning.Wn_pitch = math.sqrt(controlGains.kp_pitch * linearizedModel.a_theta3 + linearizedModel.a_theta2)
    ctuning.Zeta_pitch = (linearizedModel.a_theta1 + (controlGains.kd_pitch * linearizedModel.a_theta3)) / \
                         (2 * ctuning.Wn_pitch)
    kthdc = (controlGains.kp_pitch * linearizedModel.a_theta3) / \
            (linearizedModel.a_theta2 + (controlGains.kp_pitch * linearizedModel.a_theta3))
    ctuning.Wn_altitude = math.sqrt(kthdc * linearizedModel.Va_trim * controlGains.ki_altitude)
    ctuning.Zeta_altitude = (kthdc * linearizedModel.Va_trim * controlGains.kp_altitude) / (2 * ctuning.Wn_altitude)
    ctuning.Wn_SpeedfromThrottle = math.sqrt(linearizedModel.a_V2 * controlGains.ki_SpeedfromThrottle)
    ctuning.Zeta_SpeedfromThrottle = (linearizedModel.a_V1 + (linearizedModel.a_V2 *
                                                              controlGains.kp_SpeedfromThrottle)) / \
                                     (2 * ctuning.Wn_SpeedfromThrottle)
    ctuning.Wn_SpeedfromElevator = math.sqrt(-kthdc * VPC.g0 * controlGains.ki_SpeedfromElevator)
    ctuning.Zeta_SpeedfromElevator = (linearizedModel.a_V1 - (kthdc * VPC.g0 * controlGains.kp_SpeedfromElevator)) / \
                                     (2 * ctuning.Wn_SpeedfromElevator)
    return ctuning
