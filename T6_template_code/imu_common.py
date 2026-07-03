import numpy as np
import quaternion


def _todo(message):
    raise NotImplementedError(message)


class IMUFusionCommon:
    def __init__(self):
        self.AccelerometerNoise = 2e-6 * (gms2(1) ** 2)
        self.GyroscopeNoise = 0.01
        self.GyroscopeDriftNoise = 0.0003
        self.LinearAccelerationNoise = 1e-4 * (gms2(1) ** 2)
        self.MagnetometerNoise = 1e-4
        self.LinearAccelerationDecayFactor = 0.5
        self.OrientationCorrectionGain = 0.1
        self.MaxOrientationCorrection = 0.03
        self.MaxGyroOffsetCorrection = 0.0002
        self.OrientationFormat = 'quaternion'
        self.ReferenceFrame = 'NED'
        self.DecimationFactor = 1
        self.SampleRate = 200

        # Internal EKF state, named to match the HSA Lecture 6 notation.
        self.is_first_sample = True
        self.P_minus = self.getInitialProcCov(empirical=True)
        self.q_hat_plus = None
        self.q_hat_minus = None
        self.ref_frame = ReferenceFrame.getMathObject(self.ReferenceFrame)
        self.dt_sensor = 1.0 / self.SampleRate
        self.dt = self.dt_sensor * self.DecimationFactor
        self.b_g_hat = np.zeros((1, 3))
        self.a_lin_hat_minus = np.zeros((1, 3))
        self.a_lin_hat_plus = np.zeros((1, 3))
        self.m_w = np.array([1.0, 0.0, 0.0])
        self.input_prototype = None
        self.R_a = self.updateMeasurementErrCov()

    def set_AccelerometerNoise(self, val):
        self.AccelerometerNoise = val

    def set_GyroscopeNoise(self, val):
        self.GyroscopeNoise = val

    def set_GyroscopeDriftNoise(self, val):
        self.GyroscopeDriftNoise = val

    def set_LinearAccelerationNoise(self, val):
        self.LinearAccelerationNoise = val

    def set_LinearAccelerationDecayFactor(self, val):
        self.LinearAccelerationDecayFactor = val

    def saveObjectImpl(self):
        s = {
            'P_minus': self.P_minus,
            'R_a': self.R_a,
            'q_hat_plus': self.q_hat_plus,
            'q_hat_minus': self.q_hat_minus,
            'is_first_sample': self.is_first_sample,
            'ref_frame': self.ref_frame,
            'dt_sensor': self.dt_sensor,
            'dt': self.dt,
            'b_g_hat': self.b_g_hat,
            'a_lin_hat_minus': self.a_lin_hat_minus,
            'a_lin_hat_plus': self.a_lin_hat_plus,
            'm_w': self.m_w,
            'input_prototype': self.input_prototype
        }
        return s

    def loadObjectImpl(self, s):
        self.P_minus = s['P_minus']
        self.R_a = s['R_a']
        self.q_hat_plus = s['q_hat_plus']
        self.q_hat_minus = s['q_hat_minus']
        self.is_first_sample = s['is_first_sample']
        self.ref_frame = s['ref_frame']
        self.dt_sensor = s['dt_sensor']
        self.dt = s['dt']
        self.b_g_hat = s['b_g_hat']
        self.a_lin_hat_minus = s['a_lin_hat_minus']
        self.a_lin_hat_plus = s['a_lin_hat_plus']
        self.m_w = s['m_w']
        self.input_prototype = s['input_prototype']

    def processTunedPropertiesImpl(self):
        self.setupPeriods()

    def setupImpl(self, accelIn, *args):
        self.input_prototype = accelIn
        self.setupPeriods()
        self.ref_frame = ReferenceFrame.getMathObject(self.ReferenceFrame)

    def setupPeriods(self):
        self.dt_sensor = 1.0 / self.SampleRate
        self.dt = self.DecimationFactor * self.dt_sensor

    def validateFrameSize(self, x):
        nrows = x.shape[0]
        assert nrows % self.DecimationFactor == 0, \
            'DecimationFactor must divide the frame size.'

    def buildHPart(self, v):
        h = np.zeros((3,3), dtype=v.dtype)
        if v.shape[0] == 3:
            h[0, 1] = v[2]
            h[0, 2] = -v[1]
            h[1, 2] = v[0]
        else:
            h[0, 1] = v[0][2]
            h[0, 2] = -v[0][1]
            h[1, 2] = v[0][0]
        h -= h.T
        return h

    def rotmat2gravity(self, R):
        gravity = gms2(1)
        ref = self.ref_frame
        # TODO EKF-2:
        # Implement this directly from the rotation matrix: select
        # R[:, ref.GravityIndex], apply ref.GravityAxisSign, and scale by
        # gravity. No quaternion conversion is needed here.
        g = ref.GravityAxisSign * gravity * R[:, ref.GravityIndex]
        return g

    def rotmat2magnetic(self, R):
        m_w = np.asarray(self.m_w, dtype=float)
        m_w = m_w / np.linalg.norm(m_w)
        m = R @ m_w
        norm = np.linalg.norm(m)
        if norm == 0:
            return m
        return m / norm

    def allocateOutputs(self, numiters):
        if self.OrientationFormat.lower() == 'quaternion':
            orientOut = np.empty((numiters,), dtype=object)
        else:
            orientOut = np.zeros((3, 3, numiters))
        return orientOut

    def predictOrientation(self, omega_m, b_g_hat, q_hat):
        omega_m = np.asarray(omega_m, dtype=float).reshape(-1, 3)
        b_g_hat = np.asarray(b_g_hat, dtype=float).reshape(-1, 3)
        if b_g_hat.shape[0] == 1 and omega_m.shape[0] > 1:
            b_g_hat = np.repeat(b_g_hat, omega_m.shape[0], axis=0)

        # TODO EKF-1:
        # Remove the estimated gyro bias, integrate angular velocity over one
        # sensor period, call quaternion.from_rotation_vector(delta_theta[ii])
        # for each delta angle, and update the orientation as q_hat * delta_q.
        delta_theta = (omega_m - b_g_hat) * self.dt_sensor
        for ii in range(delta_theta.shape[0]):
            delta_q = quaternion.from_rotation_vector(delta_theta[ii])
            q_hat = q_hat * delta_q
        if np.all(quaternion.as_float_array(q_hat) < 0):
            q_hat = -q_hat
        return q_hat
    
    def updateMeasurementErrCov(self):
        accelMeasNoiseVar = self.AccelerometerNoise +\
            self.LinearAccelerationNoise +\
            (self.dt ** 2) * (self.GyroscopeDriftNoise + self.GyroscopeNoise)
        return accelMeasNoiseVar * np.eye(3)    

    def getInitialProcCov(self, empirical = False):

        diags_1_to_3 = np.arange(3)
        diags_4_to_6 = np.arange(3, 6)
        diags_7_to_9 = np.arange(6, 9)
        if empirical:
            covinit = np.zeros((9,9))
            covinit[diags_1_to_3,diags_1_to_3] = 0.000006092348396
            covinit[diags_4_to_6,diags_4_to_6] = 0.000076154354947
            covinit[diags_7_to_9,diags_7_to_9] = 0.00962361
        else:
            cOrientErrVar = (1 * (3.14159 / 180) * 1 * (3.14159 / 180) * 2000e-5)  # var in init orientation error estim.
            cGyroBiasErrVar = (1 * (3.14159 / 180) * 1 * (3.14159 / 180) * 250e-3)  # var in init gyro bias error estim
            cOrientGyroBiasErrVar = 0  # covar orient - gyro bias error estim
            cAccErrVar = 10e-5 * (gms2(1) ** 2)  # var in linear accel drift error estim
            covinit = np.zeros((9, 9))
            covinit[diags_1_to_3, diags_1_to_3] = cOrientErrVar 
            covinit[diags_4_to_6, diags_4_to_6] = cGyroBiasErrVar 
            covinit[diags_1_to_3, diags_4_to_6] = cOrientGyroBiasErrVar 
            covinit[diags_4_to_6, diags_1_to_3] = covinit[diags_1_to_3, diags_4_to_6]
            covinit[diags_7_to_9, diags_7_to_9] = cAccErrVar 
            covinit = covinit * self.getInitialProcCovMask()
        return covinit

    def getInitialProcCovMask(self):
        msk = np.zeros((9, 9), dtype=bool)
        msk[np.arange(3), np.arange(3)] = True
        msk[np.arange(3, 6), np.arange(3, 6)] = True
        msk[np.arange(3), np.arange(3, 6)] = True
        msk[np.arange(3, 6), np.arange(3)] = msk[np.arange(3), np.arange(3, 6)]
        msk[np.arange(6, 9), np.arange(6, 9)] = True
        return msk




class ReferenceFrame:
    @staticmethod
    def getMathObject(ref_frame):
        if ref_frame == 'ENU':
            return ENUReferenceFrame()
        elif ref_frame == 'NED':
            return NEDReferenceFrame()
        else:
            raise ValueError("Invalid reference frame. Supported options: 'ENU', 'NED'.")


class ENUReferenceFrame:
    NorthIndex = 1
    EastIndex = 0
    GravityIndex = 2
    NorthAxisSign = 1
    GravityAxisSign = -1
    GravitySign = -1
    ZAxisUpSign = 1
    LinAccelSign = -1


class NEDReferenceFrame:
    NorthIndex = 0
    EastIndex = 1
    GravityIndex = 2
    NorthAxisSign = 1
    GravityAxisSign = 1
    GravitySign = 1
    ZAxisUpSign = -1
    LinAccelSign = -1


def gms2(x):
    gravity = 9.81
    y = x * gravity
    return y
