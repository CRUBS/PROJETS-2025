#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

class Madgwick:
    def __init__(self):
        self.q0 = 1.
        self.q1 = 0.
        self.q2 = 0.
        self.q3 = 0.
        self.beta = 0.02


    def madgwick_ahrs_update_mag( self, gx, gy, gz, ax, ay, az, mx, my, mz, sample_freq):
        s0, s1, s2, s3 = 0.0, 0.0, 0.0, 0.0
        _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, = \
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        q_dot1 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
        q_dot2 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
        q_dot3 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
        q_dot4 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)

        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            recip_norm = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
            ax *= recip_norm
            ay *= recip_norm
            az *= recip_norm

            recip_norm = 1.0 / math.sqrt(mx * mx + my * my + mz * mz)
            mx *= recip_norm
            my *= recip_norm
            mz *= recip_norm

            _2q0mx = 2.0 * self.q0 * mx
            _2q0my = 2.0 * self.q0 * my
            _2q0mz = 2.0 * self.q0 * mz
            _2q1mx = 2.0 * self.q1 * mx
            _2q0 = 2.0 * self.q0
            _2q1 = 2.0 * self.q1
            _2q2 = 2.0 * self.q2
            _2q3 = 2.0 * self.q3
            _2q0q2 = 2.0 * self.q0 * self.q2
            _2q2q3 = 2.0 * self.q2 * self.q3
            q0q0 = self.q0 * self.q0
            q0q1 = self.q0 * self.q1
            q0q2 = self.q0 * self.q2
            q0q3 = self.q0 * self.q3
            q1q1 = self.q1 * self.q1
            q1q2 = self.q1 * self.q2
            q1q3 = self.q1 * self.q3
            q2q2 = self.q2 * self.q2
            q2q3 = self.q2 * self.q3
            q3q3 = self.q3 * self.q3

            # Reference direction of Earth's magnetic field
            hx = mx * q0q0 - _2q0my * self.q3 + _2q0mz * self.q2 + mx * q1q1 + _2q1 * my * self.q2 + _2q1 * mz * self.q3 - mx * q2q2 - mx * q3q3
            hy = _2q0mx * self.q3 + my * q0q0 - _2q0mz * self.q1 + _2q1mx * self.q2 - my * q1q1 + my * q2q2 + _2q2 * mz * self.q3 - my * q3q3
            _2bx = math.sqrt(hx * hx + hy * hy)
            _2bz = -_2q0mx * self.q2 + _2q0my * self.q1 + mz * q0q0 + _2q1mx * self.q3 - mz * q1q1 + _2q2 * my * self.q3 - mz * q2q2 + mz * q3q3
            _4bx = 2.0 * _2bx
            _4bz = 2.0 * _2bz

            s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * self.q2 * (
                    _2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * self.q3 + _2bz * self.q1) * (
                        _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * self.q2 * (
                        _2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)

            s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * self.q1 * (
                    1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * self.q3 * (
                        _2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * self.q2 + _2bz * self.q0) * (
                        _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * self.q3 - _4bz * self.q1) * (
                        _2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)

            s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * self.q2 * (
                    1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * self.q2 - _2bz * self.q0) * (
                        _2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * self.q1 + _2bz * self.q3) * (
                        _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * self.q0 - _4bz * self.q2) * (
                        _2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)

            s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * self.q3 + _2bz * self.q1) * (
                    _2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * self.q0 + _2bz * self.q2) * (
                        _2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * self.q1 * (
                        _2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - mz)

            recip_norm = 1 / math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)  # Normalise step magnitude
            s0 *= recip_norm
            s1 *= recip_norm
            s2 *= recip_norm
            s3 *= recip_norm

            # Apply feedback step
            q_dot1 -= self.beta * s0
            q_dot2 -= self.beta * s1
            q_dot3 -= self.beta * s2
            q_dot4 -= self.beta * s3

        # Integrate rate of change of quaternion to yield quaternion
        self.q0 += q_dot1 * (1.0 / sample_freq)
        self.q1 += q_dot2 * (1.0 / sample_freq)
        self.q2 += q_dot3 * (1.0 / sample_freq)
        self.q3 += q_dot4 * (1.0 / sample_freq)
        recip_norm = 1 / math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)

        self.q0 *= recip_norm
        self.q1 *= recip_norm
        self.q2 *= recip_norm
        self.q3 *= recip_norm

        return [self.q0, self.q1, self.q2, self.q3]

    """
    madgwick_ahrs_update takes Imu data and transforms it into a quaternion
    @param G (gx,gy,gz) the gyroscopes data in rad/s
    @param A (ax,ay,az) the gyroscopes data in m/s²
    @return Q a quaternion which gives the imus orientation
    """

    def madgwick_ahrs_update(self, gx, gy, gz, ax, ay, az, sample_freq):
        s0, s1, s2, s3 = 0.0, 0.0, 0.0, 0.0

        q_dot1 = 0.5 * (-self.q1 * gx - self.q2 * gy - self.q3 * gz)
        q_dot2 = 0.5 * (self.q0 * gx + self.q2 * gz - self.q3 * gy)
        q_dot3 = 0.5 * (self.q0 * gy - self.q1 * gz + self.q3 * gx)
        q_dot4 = 0.5 * (self.q0 * gz + self.q1 * gy - self.q2 * gx)

        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            recip_norm = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
            ax *= recip_norm
            ay *= recip_norm
            az *= recip_norm

            _2q0 = 2.0 * self.q0
            _2q1 = 2.0 * self.q1
            _2q2 = 2.0 * self.q2
            _2q3 = 2.0 * self.q3
            _4q0 = 4.0 * self.q0
            _4q1 = 4.0 * self.q1
            _4q2 = 4.0 * self.q2
            _8q1 = 8.0 * self.q1
            _8q2 = 8.0 * self.q2
            q0q0 = self.q0 * self.q0
            q1q1 = self.q1 * self.q1
            q2q2 = self.q2 * self.q2
            q3q3 = self.q3 * self.q3

            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * self.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az
            s2 = 4.0 * q0q0 * self.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az
            s3 = 4.0 * q1q1 * self.q3 - _2q1 * ax + 4.0 * q2q2 * self.q3 - _2q2 * ay

            recip_norm = 1 / math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)  # Normalise step magnitude
            s0 *= recip_norm
            s1 *= recip_norm
            s2 *= recip_norm
            s3 *= recip_norm

            # Apply feedback step
            q_dot1 -= self.beta * s0
            q_dot2 -= self.beta * s1
            q_dot3 -= self.beta * s2
            q_dot4 -= self.beta * s3

        # Integrate rate of change of quaternion to yield quaternion
        self.q0 += q_dot1 * (1.0 / sample_freq)
        self.q1 += q_dot2 * (1.0 / sample_freq)
        self.q2 += q_dot3 * (1.0 / sample_freq)
        self.q3 += q_dot4 * (1.0 / sample_freq)
        recip_norm = 1 / math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)

        self.q0 *= recip_norm
        self.q1 *= recip_norm
        self.q2 *= recip_norm
        self.q3 *= recip_norm

        return [self.q0, self.q1, self.q2, self.q3]


if __name__ == '__main__':
    madgwick_ahrs_update(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 100)
