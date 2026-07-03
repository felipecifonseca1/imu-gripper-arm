import numpy as np
from scipy.spatial.transform import Rotation


class quaternion:
    __slots__ = ("w", "x", "y", "z")

    def __init__(self, w, x, y, z):
        self.w = float(w)
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __mul__(self, other):
        if not isinstance(other, quaternion):
            return NotImplemented
        return quaternion(
            self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        )

    def __truediv__(self, scalar):
        return quaternion(self.w / scalar, self.x / scalar, self.y / scalar, self.z / scalar)

    def __neg__(self):
        return quaternion(-self.w, -self.x, -self.y, -self.z)

    def __repr__(self):
        return f"quaternion({self.w}, {self.x}, {self.y}, {self.z})"

    def conjugate(self):
        return quaternion(self.w, -self.x, -self.y, -self.z)


def from_rotation_vector(rotvec):
    rotvec = np.asarray(rotvec, dtype=float)
    if rotvec.ndim == 1:
        q = Rotation.from_rotvec(rotvec).as_quat()
        return quaternion(q[3], q[0], q[1], q[2])

    quats = Rotation.from_rotvec(rotvec.reshape(-1, 3)).as_quat()
    return np.array([quaternion(q[3], q[0], q[1], q[2]) for q in quats], dtype=object)


def from_rotation_matrix(matrix):
    q = Rotation.from_matrix(matrix).as_quat()
    return quaternion(q[3], q[0], q[1], q[2])


def as_rotation_matrix(q):
    vals = as_float_array(q)
    return Rotation.from_quat([vals[1], vals[2], vals[3], vals[0]]).as_matrix()


def as_rotation_vector(q):
    vals = as_float_array(q)
    return Rotation.from_quat([vals[1], vals[2], vals[3], vals[0]]).as_rotvec()


def as_float_array(q):
    if isinstance(q, quaternion):
        return np.array([q.w, q.x, q.y, q.z], dtype=float)

    arr = np.asarray(q, dtype=object)
    if arr.ndim == 0 and isinstance(arr.item(), quaternion):
        return as_float_array(arr.item())

    return np.array([[item.w, item.x, item.y, item.z] for item in arr.flat], dtype=float).reshape(arr.shape + (4,))


def as_quat_array(values):
    values = np.asarray(values, dtype=float)
    if values.ndim == 1:
        return quaternion(values[0], values[1], values[2], values[3])
    return np.array([quaternion(v[0], v[1], v[2], v[3]) for v in values.reshape(-1, 4)], dtype=object)


np.quaternion = quaternion
