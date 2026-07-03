import quaternion
import numpy as np


def ecompass(a, m):
    a = np.asarray(a, dtype=float).reshape(-1, 3)[0]
    m = np.asarray(m, dtype=float).reshape(-1, 3)[0]
    R = np.zeros((3,3))
    R1 = np.cross(np.cross(a,m),a)
    R2 = np.cross(a,m)
    if np.linalg.norm(R1) == 0 or np.linalg.norm(R2) == 0:
        raise ValueError("Accelerometer and magnetic reference vectors must not be parallel.")
    R[:,0] = R1/np.linalg.norm(R1)
    R[:,1] = R2/np.linalg.norm(R2)
    R[:,2] = a/np.linalg.norm(a)
    ori = quaternion.from_rotation_matrix(R)
    return ori
