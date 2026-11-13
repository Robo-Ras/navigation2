# Copyright (c) 2026 Robo-Ras
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Subset of tf_transformations used by the loopback simulator (Humble port).

Added so the package only needs numpy instead of ros-humble-tf-transformations and
python3-transforms3d. Same function names and conventions: quaternions are [x, y, z, w],
euler angles are static 'sxyz' (roll, pitch, yaw).
"""

import math

import numpy as np


def quaternion_from_euler(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def euler_from_quaternion(q):
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def quaternion_multiply(q1, q0):
    """Return q1 * q0 (same argument order as tf_transformations)."""
    x1, y1, z1, w1 = q1
    x0, y0, z0, w0 = q0
    return np.array([
        w1 * x0 + x1 * w0 + y1 * z0 - z1 * y0,
        w1 * y0 - x1 * z0 + y1 * w0 + z1 * x0,
        w1 * z0 + x1 * y0 - y1 * x0 + z1 * w0,
        w1 * w0 - x1 * x0 - y1 * y0 - z1 * z0,
    ])


def quaternion_matrix(q):
    """4x4 homogeneous rotation matrix from quaternion [x, y, z, w]."""
    q = np.array(q, dtype=float)
    n = np.dot(q, q)
    m = np.eye(4)
    if n < 1e-12:
        return m
    x, y, z, w = q * math.sqrt(2.0 / n)
    m[:3, :3] = [
        [1.0 - y * y - z * z, x * y - z * w, x * z + y * w],
        [x * y + z * w, 1.0 - x * x - z * z, y * z - x * w],
        [x * z - y * w, y * z + x * w, 1.0 - x * x - y * y],
    ]
    return m


def quaternion_from_matrix(matrix):
    """Quaternion [x, y, z, w] from the rotation part of a 3x3 or 4x4 matrix."""
    m = np.array(matrix, dtype=float)[:3, :3]
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    if q[3] < 0.0:
        q = -q
    return q


def inverse_matrix(matrix):
    return np.linalg.inv(matrix)


def concatenate_matrices(*matrices):
    m = np.eye(4)
    for i in matrices:
        m = np.dot(m, i)
    return m
