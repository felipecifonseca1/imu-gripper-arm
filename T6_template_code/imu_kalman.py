import numpy as np
import quaternion
from ecompass import ecompass


def _todo(message):
    raise NotImplementedError(message)


def limit_vector_norm(v, max_norm):
    norm = np.linalg.norm(v)
    if norm > max_norm:
        return v * (max_norm / norm)
    return v


def stepImpl(obj, accelIn, gyroIn, magIn):
    # Fuse accelerometer, gyroscope, and magnetometer readings with an
    # indirect EKF.
    # accelIn - Nx3 matrix of accel samples in m/s^2
    # gyroIn - Nx3 matrix of gyro samples in rad/s
    # magIn - Nx3 matrix of normalized magnetometer samples

    accelIn = np.reshape(np.transpose(accelIn), (3, obj.DecimationFactor, -1))
    gyroIn = np.reshape(np.transpose(gyroIn), (3, obj.DecimationFactor, -1))
    magIn = np.reshape(np.transpose(magIn), (3, obj.DecimationFactor, -1))
    accelIn = np.transpose(accelIn, (1, 0, 2))
    gyroIn = np.transpose(gyroIn, (1, 0, 2))
    magIn = np.transpose(magIn, (1, 0, 2))
    ref = obj.ref_frame

    numiters = accelIn.shape[2]

    orientOut = obj.allocateOutputs(numiters)

    for iter in range(numiters):
        # The EKF state is the small error vector
        # [delta_theta, delta_b_g, delta_a_lin]. The nominal quaternion, gyro
        # bias, and linear acceleration are corrected after the update.

        omega_m = gyroIn[:, :, iter]

        a_m = accelIn[:, :, iter]
        m_m = magIn[:, :, iter]

        if obj.is_first_sample:
            # Basic tilt corrected ecompass orientation algorithm.
            # Do this the first time only. Need inputs, so not in setupImpl.

            obj.q_hat_plus = ecompass(a_m, m_m)
            obj.is_first_sample = False
        
        # Update the orientation quaternion based on the gyroscope readings.
        obj.q_hat_minus = obj.predictOrientation(omega_m, obj.b_g_hat, obj.q_hat_plus)

        # TODO EKF-3:
        # Use quaternion.as_rotation_matrix(obj.q_hat_minus) for this
        # conversion.
        R_q_hat_minus = quaternion.as_rotation_matrix(obj.q_hat_minus)

        z_a_hat_minus = obj.rotmat2gravity(R_q_hat_minus).T

        obj.a_lin_hat_minus = obj.LinearAccelerationDecayFactor * obj.a_lin_hat_plus
        z_a = ref.GravitySign * a_m + obj.a_lin_hat_minus
        # TODO EKF-4:
        # Build the accelerometer residual:
        # measured gravity estimate - gyro-predicted gravity estimate.
        r_a = z_a - z_a_hat_minus

        # TODO EKF-5:
        # Normalize m_m row-wise, predict the magnetic direction with
        # obj.rotmat2magnetic(R_q_hat_minus).reshape(1, 3), and subtract the
        # prediction from the normalized measurement.
        norms = np.linalg.norm(m_m, axis=1, keepdims=True)
        norms[norms == 0] = 1.0
        m_m_norm = m_m / norms
        m_s_hat_minus = obj.rotmat2magnetic(R_q_hat_minus).reshape(1, 3)
        r_m = m_m_norm - m_s_hat_minus

        # TODO EKF-6:
        # Assemble the 6x9 observation matrix. The first three rows come from
        # gravity; the last three rows come from the magnetic field. Use
        # obj.buildHPart(...) for the -[v]_x blocks.
        H_a_theta =  -obj.buildHPart(z_a_hat_minus)   
        H_a_bias = obj.dt_sensor * obj.buildHPart(z_a_hat_minus)
        H_a = np.concatenate((H_a_theta, H_a_bias, np.zeros((3, 3))), axis=1)
        H_m_theta = -obj.buildHPart(m_s_hat_minus)
        H_m_bias = obj.dt_sensor * obj.buildHPart(m_s_hat_minus)
        H_m = np.concatenate((H_m_theta, H_m_bias, np.zeros((3, 3))), axis=1)
        H_k = np.concatenate((H_a, H_m), axis=0)

        # TODO EKF-7:
        # Stack the residual vector and build the 6x6 measurement covariance.
        r_k = np.concatenate((r_a, r_m), axis=0).reshape(-1, 1)
        R_k = np.block([
            [obj.R_a, np.zeros((3, 3))],
            [np.zeros((3, 3)), obj.MagnetometerNoise * np.eye(3)]
        ])
        P_minus = obj.P_minus

        # TODO EKF-8:
        # Compute innovation covariance S, Kalman gain K, and posterior
        # error-state estimate delta_x_hat.
        S_k = H_k @ P_minus @ H_k.T + R_k
        K_k = P_minus @ H_k.T @ np.linalg.inv(S_k)
        delta_x_hat = K_k @ r_k
        
        # Corrected error estimates
        delta_theta_hat = limit_vector_norm(delta_x_hat[0:3, 0], obj.MaxOrientationCorrection)
        delta_b_g_hat = limit_vector_norm(delta_x_hat[3:6, 0], obj.MaxGyroOffsetCorrection).reshape(1, 3)
        delta_a_lin_hat = delta_x_hat[6:, 0].reshape(1, 3)

        # TODO EKF-9:
        # Call quaternion.from_rotation_vector(-delta_theta_hat) to convert
        # orientation error into a correction quaternion, then right-multiply
        # it onto obj.q_hat_minus and normalize.
        delta_q = quaternion.from_rotation_vector(-delta_theta_hat)
        obj.q_hat_plus = obj.q_hat_minus * delta_q
        obj.q_hat_plus /= np.linalg.norm(quaternion.as_float_array(obj.q_hat_plus))

        q_meas = ecompass(a_m, m_m)
        q_delta = obj.q_hat_plus.conjugate() * q_meas
        delta_theta_meas = limit_vector_norm(
            quaternion.as_rotation_vector(q_delta),
            obj.MaxOrientationCorrection,
        )
        delta_q_meas = quaternion.from_rotation_vector(obj.OrientationCorrectionGain * delta_theta_meas)
        obj.q_hat_plus = obj.q_hat_plus * delta_q_meas
        obj.q_hat_plus /= np.linalg.norm(quaternion.as_float_array(obj.q_hat_plus))
        
        # TODO EKF-10:
        # Under this residual convention, subtract the estimated vector errors
        # from the nominal gyro bias and linear acceleration.
        obj.b_g_hat = obj.b_g_hat - delta_b_g_hat
        obj.a_lin_hat_plus = obj.a_lin_hat_plus - delta_a_lin_hat

        # TODO EKF-11:
        # Compute posterior error covariance.
        P_plus = (np.eye(P_minus.shape[0]) - K_k @ H_k) @ P_minus
        P_minus_next = np.zeros((9, 9))

        diags_1_to_3 = np.arange(3)
        diags_4_to_6 = np.arange(3, 6)
        diags_7_to_9 = np.arange(6, 9)
        P_minus_next[diags_1_to_3, diags_1_to_3] = P_plus[diags_1_to_3, diags_1_to_3] + \
            (obj.dt**2) * (P_plus[diags_4_to_6, diags_4_to_6] + \
            (obj.GyroscopeDriftNoise + obj.GyroscopeNoise))

        P_minus_next[diags_4_to_6, diags_4_to_6] = P_plus[diags_4_to_6, diags_4_to_6] + obj.GyroscopeDriftNoise

        off_diag = -obj.dt * P_minus_next[diags_4_to_6, diags_4_to_6]
        P_minus_next[diags_4_to_6, diags_1_to_3] = off_diag
        P_minus_next[diags_1_to_3, diags_4_to_6] = off_diag

        
        P_minus_next[diags_7_to_9, diags_7_to_9] = (obj.LinearAccelerationDecayFactor**2) * \
            P_plus[diags_7_to_9, diags_7_to_9] + obj.LinearAccelerationNoise

        obj.P_minus = P_minus_next

        if obj.OrientationFormat.lower() == 'quaternion':
            orientOut[iter] = obj.q_hat_plus
        else:
            pass

    return orientOut
