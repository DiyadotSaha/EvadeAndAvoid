import math
from ..Containers import States
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC


class VehicleDynamicsModel():
    # Initializes the class, and sets the time step (needed for Rexp and integration). Instantiates attributes for
    # vehicle state, and time derivative of vehicle state.
    def __init__(self, dT=VPC.dT):
        self.dT = dT
        self.dot = States.vehicleState()
        self.state = States.vehicleState()

    def ForwardEuler(self, dT, state, dot):
        # Function to do the simple forwards integration of the state using the input dot.
        # State is integrated using formula from lectures/book. Updated state returned by function.
        fn = state.pn + dot.pn * dT
        fe = state.pe + dot.pe * dT
        fd = state.pd + dot.pd * dT
        fu = state.u + dot.u * dT
        fv = state.v + dot.v * dT
        fw = state.w + dot.w * dT
        fp = state.p + dot.p * dT
        fq = state.q + dot.q * dT
        fr = state.r + dot.r * dT
        new_state = States.vehicleState(pn=fn, pe=fe, pd=fd, u=fu,
                                        v=fv, w=fw, p=fp, q=fq, r=fr)
        return new_state

    def IntegrateState(self, dT, state, dot):
        # Updates the state given the derivative, and a time step. Attitude propagation is implemented
        # as a DCM matrix exponential solution, all other state params are advanced via forward euler
        # integration(Forward Euler). The integrated state is returned from the function
        feuler = self.ForwardEuler(dT, state, dot)
        ex = self.Rexp(dT, state, dot)
        rex = MatrixMath.multiply(ex, state.R)
        new_state = States.vehicleState(pn=feuler.pn, pe=feuler.pe, pd=feuler.pd, u=feuler.u, v=feuler.v,
                                        w=feuler.w, p=feuler.p,
                                        q=feuler.q, r=feuler.r, dcm=rex)
        new_state.alpha = state.alpha           # Derived variables in the state are copied
        new_state.beta = state.beta
        new_state.Va = state.Va
        new_state.chi = math.atan2(dot.pe, dot.pn)
        return new_state

    def Rexp(self, dT, state, dot):
        # Calculates the matrix exponential using formula from attitude cheat sheet.
        p = state.p + (dot.p * (dT / 2.0))
        q = state.q + (dot.q * (dT / 2.0))
        r = state.r + (dot.r * (dT / 2.0))        # Equation from lecture as well as Carlos' programming section
        skewmat = MatrixMath.skew(p, q, r)
        sqskew = MatrixMath.multiply(skewmat, skewmat)
        omega = math.hypot(p, q, r)
        if omega < 0.2:                         # Taylor-Series approximation from cheat sheet
            p0 = dT - (((dT ** 3.0) * (omega ** 2.0)) / 6.0) + \
                 (((dT ** 5.0) * (omega ** 4.0)) / 120.0)
            p2 = ((dT ** 2.0) / 2.0) - (((dT ** 4.0) * (omega ** 2.0)) / 24.0) + \
                 (((dT ** 6.0) * (omega ** 4.0)) / 720.0)
        else:
            p0 = (math.sin(omega * dT)) / omega
            p2 = (1 - math.cos(omega * dT)) / (omega ** 2.0)

        idmat = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        p1 = MatrixMath.scalarMultiply(p0, skewmat)
        p3 = MatrixMath.scalarMultiply(p2, sqskew)
        p4 = MatrixMath.subtract(idmat, p1)
        fmat = MatrixMath.add(p4, p3)
        return fmat

    def Update(self, forcesMoments):
        # Function that implements the integration such that the state is updated using forces
        # and moments passed in as the arguments. State is updated in place. I mostly used the
        # handy diagram within our lab doc for what to include in Update()
        state = self.getVehicleState()
        dot = self.derivative(state, forcesMoments)
        self.state = self.IntegrateState(self.dT, state, dot)
        return

    def derivative(self, state, forcesMoments):
        # Function to compute the time-derivative of the state given body frame forces and moments
        # All formulas are from lecture/book
        new_state = States.vehicleState()
        sro = math.sin(state.roll)
        cro = math.cos(state.roll)
        cpi = math.cos(state.pitch)
        tpi = math.tan(state.pitch)
        tdcm = MatrixMath.transpose(state.R)
        uvwmat = [[state.u], [state.v], [state.w]]
        dp = MatrixMath.multiply(tdcm, uvwmat)
        new_state.pn = dp[0][0]
        new_state.pe = dp[1][0]
        new_state.pd = dp[2][0]
        rm = [[1.0, sro * tpi, cro * tpi],
              [0.0, cro, -1.0 * sro], [0.0, sro / cpi, cro / cpi]]
        pqrmat = [[state.p], [state.q], [state.r]]
        dypr = MatrixMath.multiply(rm, pqrmat)
        new_state.roll = dypr[0][0]
        new_state.pitch = dypr[1][0]
        new_state.yaw = dypr[2][0]
        skewmat = MatrixMath.skew(state.p, state.q, state.r)
        scskew = MatrixMath.scalarMultiply(-1.0, skewmat)
        new_state.R = MatrixMath.multiply(scskew, state.R)
        scuvw = MatrixMath.multiply(scskew, uvwmat)
        fmat = [[forcesMoments.Fx], [forcesMoments.Fy], [forcesMoments.Fz]]
        mforce = MatrixMath.scalarDivide(VPC.mass, fmat)
        duvw = MatrixMath.add(scuvw, mforce)
        new_state.u = duvw[0][0]
        new_state.v = duvw[1][0]
        new_state.w = duvw[2][0]
        jw = MatrixMath.multiply(skewmat, VPC.Jbody)
        jskew = MatrixMath.multiply(jw, pqrmat)
        mmat = [[forcesMoments.Mx], [forcesMoments.My], [forcesMoments.Mz]]
        mdpqr = MatrixMath.subtract(mmat, jskew)
        dpqr = MatrixMath.multiply(VPC.JinvBody, mdpqr)
        new_state.p = dpqr[0][0]
        new_state.q = dpqr[1][0]
        new_state.r = dpqr[2][0]
        new_state.chi = math.atan2(new_state.pe, new_state.pn)
        return new_state

    def getVehicleDerivative(self):
        return self.dot

    def getVehicleState(self):
        return self.state

    def reset(self):
        new_state = States.vehicleState()
        self.setVehicleState(new_state)
        self.setVehicleDerivative(new_state)
        return

    def setVehicleDerivative(self, dot):
        self.dot = dot
        return

    def setVehicleState(self, state):
        self.state = state
        return
