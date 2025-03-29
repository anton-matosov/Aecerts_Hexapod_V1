# # Copyright (c) 2017-2025 Anton Matosov
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import numpy as np
from point import Line3D, Point3D
from scipy.spatial.transform import Rotation as R


class Transform:
    """
    An affine transform class.

    Uses 4x4 matrix to represent a transform.
    """

    def __init__(self, matrix):
        self._matrix = matrix

    @classmethod
    def make(cls, rotation, translation):
        return cls(
            np.block(
                [
                    [rotation.as_matrix(), np.array(translation).reshape(3, 1)],
                    [np.zeros(3), 1],
                ]
            )
        )

    @classmethod
    def identity(cls):
        return cls(np.identity(4))

    @classmethod
    def from_rotvec(cls, rotvec, degrees=False):
        return cls.make(R.from_rotvec(rotvec, degrees=degrees), [0, 0, 0])

    @classmethod
    def from_rotmatrix(cls, rotation_matrix):
        return cls(
            np.block(
                [
                    [np.array(rotation_matrix), np.zeros((3, 1))],
                    [np.zeros(3), 1],
                ]
            )
        )

    @classmethod
    def from_translation(cls, translation):
        return cls.make(R.identity(), translation)

    @property
    def rotation(self):
        return self._matrix[:3, :3]

    @property
    def translation(self):
        return self._matrix[:3, 3]

    @property
    def matrix(self):
        return self._matrix

    def apply_point(self, point):
        return Point3D(self.apply_nd(point.numpy()))

    def apply_line(self, line: Line3D):
        return Line3D(self.apply_point(line.start), self.apply_point(line.end), line.label)

    def apply_nd(self, nd_point):
        point4d = np.append(nd_point, 1)
        transformed_point = self._matrix @ point4d
        return transformed_point[:3]

    def inverse(self):
        return Transform(np.linalg.inv(self._matrix))

    # operator @
    def __matmul__(self, other):
        return Transform(self._matrix @ other._matrix)

    def __repr__(self):
        rotation = R.from_matrix(self.rotation).as_rotvec(degrees=True)
        translation = self.translation
        return f'Transform({translation=}, {rotation=}, as matrix:\n{self._matrix})'
