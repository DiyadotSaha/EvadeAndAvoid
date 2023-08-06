import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Modeling import WindModel
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

check = True

class VehicleAerodynamicsModel():
    def __init__(self, initialSpeed=VPC.InitialSpeed, initialHeight=VPC.InitialDownPosition):
        self.model = VehicleDynamicsModel.VehicleDynamicsModel()
        self.model.state.pn = VPC.InitialNorthPosition
        self.model.state.pe = VPC.InitialEastPosition
        self.model.state.pd = VPC.InitialDownPosition
        self.initialSpeed = initialSpeed
        self.initialHeight = initialHeight
        self.model.state.u = initialSpeed
        self.windmodel = WindModel.WindModel()
        return

    def CalculateAirspeed(self, state, wind):
        # print("Calc Air Speed")
        chiw = math.atan2(wind.We, wind.Wn)
        magw = math.hypot(wind.Wn, wind.We, wind.Wd)
        if magw == 0.0:
            # print("MagW = 0")
            gammaw = 0.0
        else:
            # print("MagW != 0")
            gammaw = -math.asin(wind.Wd / magw)
        wRxg = Rotations.euler2DCM(chiw, gammaw, 0.0)
        iRxg = MatrixMath.transpose(wRxg)
        wgusti = MatrixMath.multiply(iRxg, [[wind.Wu], [wind.Wv], [wind.Ww]])
        Rmul = MatrixMath.add([[wind.Wn], [wind.We], [wind.Wd]], wgusti)
        Wtot = MatrixMath.multiply(state.R, Rmul)
        vr = MatrixMath.subtract([[state.u], [state.v], [state.w]], Wtot)
        Va = math.hypot(vr[0][0], vr[1][0], vr[2][0])
        alpha = math.atan2(vr[2][0], vr[0][0])
        if Va == 0.0:
            beta = 0.0
        else:
            beta = math.asin(vr[1][0] / Va)

        return Va, alpha, beta

    def __sigma__(self, alpha):
        num = (1.0 + math.exp(-1.0 * VPC.M * (alpha - VPC.alpha0)) + math.exp(VPC.M * (alpha + VPC.alpha0)))
        den = (1.0 + math.exp(-1.0 * VPC.M * (alpha - VPC.alpha0))) * (1.0 + math.exp(VPC.M * (alpha + VPC.alpha0)))
        return num / den

    def CalculateCoeff_alpha(self, alpha):
        clattach = VPC.CL0 + VPC.CLalpha * alpha
        cdattach = (VPC.CDp + ((VPC.CL0 + VPC.CLalpha * alpha) ** 2.0) / (math.pi * VPC.e * VPC.AR))
        clsep = 2.0 * math.sin(alpha) * math.cos(alpha)
        cdsep = 2.0 * math.sin(alpha) ** 2
        sig = self.__sigma__(alpha)
        cl = ((1.0 - sig) * clattach) + (sig * clsep)
        cd = ((1.0 - sig) * cdattach) + (sig * cdsep)
        cm = VPC.CM0 + VPC.CMalpha * alpha
        return cl, cd, cm

    def CalculatePropForces(self, Va, Throttle):
        Vin = VPC.V_max * Throttle
        a = (VPC.rho * (VPC.D_prop ** 5.0) * VPC.C_Q0) / (4.0 * (math.pi ** 2.0))
        b = ((VPC.rho * (VPC.D_prop ** 4.0) * Va * VPC.C_Q1) / (2.0 * math.pi)) + ((VPC.KQ ** 2.0) / VPC.R_motor)
        c = (VPC.rho * (VPC.D_prop ** 3.0) * (Va ** 2.0) * VPC.C_Q2) - \
            (VPC.KQ * (Vin / VPC.R_motor)) + (VPC.KQ * VPC.i0)
        try:
            omega = (-b + math.sqrt((b ** 2.0) - (4.0 * a * c))) / (2.0 * a)
        except:
            omega = 100.0
        J = (2.0 * math.pi * Va) / (omega * VPC.D_prop)
        Ct = VPC.C_T0 + (VPC.C_T1 * J) + (VPC.C_T2 * (J ** 2.0))
        Cq = VPC.C_Q0 + (VPC.C_Q1 * J) + (VPC.C_Q2 * (J ** 2.0))
        F = (VPC.rho * (omega ** 2.0) * (VPC.D_prop ** 4.0) * Ct) / (4.0 * (math.pi ** 2.0))
        M = (-1.0 * VPC.rho * (omega ** 2.0) * (VPC.D_prop ** 5.0) * Cq) / (4.0 * (math.pi ** 2.0))
        return F, M

    def Update(self, controls):
        state = self.model.state
        self.windmodel.Update()
        updated_force = self.updateForces(state, controls, self.windmodel.windState)
        self.model.Update(updated_force)
        # print("VAM UPDATE")
        return

    def aeroForces(self, state):
        Calpha = self.CalculateCoeff_alpha(state.alpha)
        Cla = Calpha[0]
        Cda = Calpha[1]
        # Cma = Calpha[2]
        if state.Va == 0:
            new_b = 0.0
            new_c = 0.0
        else:
            new_b = (VPC.b / (2.0 * state.Va))
            new_c = (VPC.c / (2.0 * state.Va))
        Fl = -1.0 * (((1.0 / 2.0) * VPC.rho * (state.Va ** 2.0) * VPC.S) *
                   (Cla + (VPC.CLq * state.q * new_c)))
        Fd = -1.0 * (((1.0 / 2.0) * VPC.rho * (state.Va ** 2.0) * VPC.S) *
                   (Cda + (VPC.CDq * state.q * new_c)))
        Fy = ((1.0 / 2.0) * VPC.rho * (state.Va ** 2.0) * VPC.S) * \
             (VPC.CY0 + (VPC.CYbeta * state.beta) +
              (VPC.CYp * new_b * state.p) +
              (VPC.CYr * new_b * state.r))
        l = ((1.0 / 2.0) * VPC.rho * (state.Va ** 2.0) * VPC.S * VPC.b) * \
            (VPC.Cl0 + (VPC.Clbeta * state.beta) +
             (VPC.Clp * new_b * state.p) +
             (VPC.Clr * new_b * state.r))
        m = ((1.0 / 2.0) * VPC.rho * (state.Va ** 2.0) * VPC.S * VPC.c) * \
            (VPC.CM0 + (VPC.CMalpha * state.alpha) +
             (VPC.CMq * new_c * state.q))
        n = ((1.0 / 2.0) * VPC.rho * (state.Va ** 2.0) * VPC.S * VPC.b) * \
            (VPC.Cn0 + (VPC.Cnbeta * state.beta) +
             (VPC.Cnp * new_b * state.p) +
             (VPC.Cnr * new_b * state.r))
        mbody = [[math.cos(state.alpha), -1.0 * math.sin(state.alpha)],
                 [math.sin(state.alpha), math.cos(state.alpha)]]
        fbody = MatrixMath.multiply(mbody, [[Fd], [Fl]])
        new_moment = Inputs.forcesMoments(Fx=fbody[0][0], Fy=Fy, Fz=fbody[1][0], Mx=l, My=m, Mz=n)
        return new_moment

    def controlForces(self, state, controls):
        prop_force = self.CalculatePropForces(state.Va, controls.Throttle)
        l = ((1.0 / 2.0) * VPC.rho * (state.Va ** 2.0) * VPC.S * VPC.b) * \
            ((VPC.CldeltaA * controls.Aileron) + (VPC.CldeltaR * controls.Rudder))
        m = ((1.0 / 2.0) * VPC.rho * (state.Va ** 2.0) * VPC.S * VPC.c) * \
            (VPC.CMdeltaE * controls.Elevator)
        n = ((1.0 / 2.0) * VPC.rho * (state.Va ** 2.0) * VPC.S * VPC.b) * \
            ((VPC.CndeltaA * controls.Aileron) + (VPC.CndeltaR * controls.Rudder))
        Fl = -1.0 * (((1.0 / 2.0) * VPC.rho * (state.Va ** 2.0) * VPC.S) *
                   (VPC.CLdeltaE * controls.Elevator))
        Fd = -1.0 * (((1.0 / 2.0) * VPC.rho * (state.Va ** 2.0) * VPC.S) *
                   (VPC.CDdeltaE * controls.Elevator))
        fy = ((1.0 / 2.0) * VPC.rho * (state.Va ** 2.0) * VPC.S) * \
             ((VPC.CYdeltaA * controls.Aileron) + (VPC.CYdeltaR * controls.Rudder))
        mbody = [[math.cos(state.alpha), -1.0 * math.sin(state.alpha)],
                 [math.sin(state.alpha), math.cos(state.alpha)]]
        fbody = MatrixMath.multiply(mbody, [[Fd], [Fl]])
        new_moment = Inputs.forcesMoments(Fx=fbody[0][0] + prop_force[0], Fy=fy, Fz=fbody[1][0],
                                          Mx=l + prop_force[1], My=m, Mz=n)
        return new_moment

    def getVehicleDynamicsModel(self):
        # print("Testing1\n")
        return self.model

    def getVehicleState(self):
        # print("Testing2\n")
        return self.model.state

    def getWindModel(self):
        return self.windmodel

    def gravityForces(self, state):
        scmR = MatrixMath.scalarMultiply(VPC.mass, state.R)
        fg = MatrixMath.multiply(scmR, [[0], [0], [VPC.g0]])
        temp_fmoment = Inputs.forcesMoments(Fx=fg[0][0], Fy=fg[1][0], Fz=fg[2][0])
        return temp_fmoment

    def reset(self):
        new_state = States.vehicleState()
        self.model.state = new_state
        self.model.dot = new_state
        self.model.pd = self.initialHeight
        self.model.u = self.initialSpeed
        self.windmodel.reset()
        return

    def setVehicleState(self, state):
        self.model.state = state
        return

    def setWindModel(self, windModel):
        self.windmodel = windModel
        return

    def updateForces(self, state, controls, wind=None):
        if wind:
            # print("Wind is not None")
            # print(wind)
            airspeed = self.CalculateAirspeed(state, wind)
            state.Va = airspeed[0]
            state.alpha = airspeed[1]
            state.beta = airspeed[2]
        else:
            # print("Wind: None")
            # print(wind)
            state.Va = math.hypot(state.u, state.v, state.w)
            state.alpha = math.atan2(state.w, state.u)
            if math.isclose(state.Va, 0.0):  # Sideslip Angle, no airspeed
                state.beta = 0.0
            else:
                state.beta = math.asin(state.v / state.Va)  # Sideslip Angle, normal definition
        aero = self.aeroForces(state)
        control = self.controlForces(state, controls)
        gravity = self.gravityForces(state)
        update_forces = Inputs.forcesMoments(Fx=aero.Fx + control.Fx + gravity.Fx,
                                             Fy=aero.Fy + control.Fy + gravity.Fy,
                                             Fz=aero.Fz + control.Fz + gravity.Fz,
                                             Mx=aero.Mx + control.Mx,
                                             My=aero.My + control.My,
                                             Mz=aero.Mz + control.Mz)
        return update_forces
