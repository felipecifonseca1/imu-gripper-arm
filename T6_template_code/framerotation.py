import numpy as np
import quaternion


def solveQ2E(q):
    q_array = np.atleast_2d(quaternion.as_float_array(q))
    qa = np.reshape(q_array[:, 0], (-1, 1))
    qb = np.reshape(q_array[:, 1], (-1, 1))
    qc = np.reshape(q_array[:, 2], (-1, 1))
    qd = np.reshape(q_array[:, 3], (-1, 1))
    the1 = np.ones(1, dtype=qa.dtype)
    the2 = 2 * the1
    tmp = qb * qd * the2 - qa * qc * the2
    tmp[tmp > the1] = the1
    tmp[tmp < -the1] = -the1
    tolA = float(0.5 * np.pi - 10 * np.finfo(the1.dtype).eps)
    tolB = float(-0.5 * np.pi + 10 * np.finfo(the1.dtype).eps)

    b = -np.arcsin(tmp)
    if np.all(b >= tolA):
        a = -2 * np.arctan2(qb, qa)
        c = np.zeros_like(a)
    elif np.all(b <= tolB):
        a = 2 * np.arctan2(qb, qa)
        c = np.zeros_like(a)
    else:
        a = np.arctan2((qa * qd * the2 + qb * qc * the2), (qa**2 * the2 - the1 + qb**2 * the2))
        c = np.arctan2((qa * qb * the2 + qc * qd * the2), (qa**2 * the2 - the1 + qd**2 * the2))

    return a, b, c
