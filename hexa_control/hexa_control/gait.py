#!/usr/bin/env python3
import math
import numpy as np

LEG_ORDER = ['FH_LH', 'FH_RH', 'MH_LH', 'MH_RH', 'BH_LH', 'BH_RH']
TRIPOD_A  = ['FH_LH', 'MH_RH', 'BH_LH']
TRIPOD_B  = ['FH_RH', 'MH_LH', 'BH_RH']

def _build_phase_matrix():
    n = len(LEG_ORDER)
    phi = np.zeros((n, n))
    for i, li in enumerate(LEG_ORDER):
        for j, lj in enumerate(LEG_ORDER):
            if (li in TRIPOD_A) != (lj in TRIPOD_A):
                phi[i, j] = math.pi
    return phi

PHASE_BIAS_MATRIX = _build_phase_matrix()

class CpgGait:
    def __init__(self, frequency=1.0, amplitude=0.08, normal_x=0.215, normal_z=-0.191, phase=0.0, duty_factor=0.70):
        self.frequency   = frequency
        self.amplitude   = amplitude
        self.normal_x    = normal_x
        self.normal_z    = normal_z
        self.phase       = phase
        self.duty_factor = duty_factor  

        self.Tripod_A = TRIPOD_A
        self.Tripod_B = TRIPOD_B

        self.leg_mount_angles = {
            'FH_LH':  0.0,
            'MH_LH':  1.13,
            'BH_LH':  2.27,
            'BH_RH':  3.14,
            'MH_RH': -2.01,
            'FH_RH': -0.90,
        }

        self.default_foot_pos = {
            leg: np.array([self.normal_x, 0.0, self.normal_z])
            for leg in LEG_ORDER
        }

        self.mu    = 1.0
        self.alpha = 5.0
        self.w     = 1.5
        self.omega = 2.0 * math.pi * self.frequency   
        self.omega_stance = self.omega / (2.0 * self.duty_factor)
        self.omega_swing  = self.omega / (2.0 * (1.0 - self.duty_factor))

        n = len(LEG_ORDER)
        self._x = np.zeros(n)
        self._y = np.zeros(n)
        r0 = math.sqrt(self.mu)

        for i, leg in enumerate(LEG_ORDER):
            if leg in TRIPOD_A:
                self._x[i] = r0
                self._y[i] = 0.0
            else:
                self._x[i] = -r0
                self._y[i] = 0.0

    def _derivatives(self, x_state, y_state, speed_scale):
        n = len(LEG_ORDER)
        dx = np.zeros(n)
        dy = np.zeros(n)

        for i in range(n):
            xi, yi = x_state[i], y_state[i]
            r2 = xi**2 + yi**2
            if yi <= 0.0:
                omega_i = self.omega_stance * speed_scale
            else:
                omega_i = self.omega_swing * speed_scale

            hopf_x = self.alpha * (self.mu - r2) * xi - omega_i * yi
            hopf_y = self.alpha * (self.mu - r2) * yi + omega_i * xi

            coupling_x = 0.0
            coupling_y = 0.0
            for j in range(n):
                if i == j:
                    continue
                phi_ij = PHASE_BIAS_MATRIX[i, j]
                xj, yj = x_state[j], y_state[j]
                cos_p = math.cos(phi_ij)
                sin_p = math.sin(phi_ij)

                xj_rot =  cos_p * xj - sin_p * yj
                yj_rot =  sin_p * xj + cos_p * yj

                coupling_x += (xj_rot - xi)
                coupling_y += (yj_rot - yi)

            dx[i] = hopf_x + self.w * coupling_x
            dy[i] = hopf_y + self.w * coupling_y

        return dx, dy

    def cpg_phase(self, dt, velocity):
        speed_scale = min(1.0, abs(velocity) / 0.02) if velocity > 0.001 else 0.0
        x, y = self._x.copy(), self._y.copy()

        k1x, k1y = self._derivatives(x,                    y,                    speed_scale)
        k2x, k2y = self._derivatives(x + 0.5 * dt * k1x, y + 0.5 * dt * k1y, speed_scale)
        k3x, k3y = self._derivatives(x + 0.5 * dt * k2x, y + 0.5 * dt * k2y, speed_scale)
        k4x, k4y = self._derivatives(x + dt * k3x,       y + dt * k3y,       speed_scale)

        self._x += (dt / 6.0) * (k1x + 2.0 * k2x + 2.0 * k3x + k4x)
        self._y += (dt / 6.0) * (k1y + 2.0 * k2y + 2.0 * k3y + k4y)

        self.phase = math.atan2(self._y[0], self._x[0]) % (2.0 * math.pi)
        return self.phase

    def foot_trajectory(self, leg, vx, vy, wz):
        i = LEG_ORDER.index(leg)
        xi, yi = self._x[i], self._y[i]
        
        leg_phase = math.atan2(yi, xi) % (2.0 * math.pi)

        shift_x_body = vx * (1.0 / self.frequency) * 0.5 * 1.5
        shift_y_body = vy * (1.0 / self.frequency) * 0.5 * 1.5

        alpha = self.leg_mount_angles[leg]
        shift_x_local =  math.cos(-alpha) * shift_x_body - math.sin(-alpha) * shift_y_body
        shift_y_local =  math.sin(-alpha) * shift_x_body + math.cos(-alpha) * shift_y_body

        shift_y_local += wz * 0.05

        base_pos = self.default_foot_pos[leg]

        if leg_phase < math.pi:
            progress = leg_phase / math.pi
            dx = -shift_x_local * (progress - 0.5)
            dy = -shift_y_local * (progress - 0.5)
            dz = 0.0
        else:
            progress = (leg_phase - math.pi) / math.pi
            dx = shift_x_local * (progress - 0.5)
            dy = shift_y_local * (progress - 0.5)
            dz = self.amplitude * (math.sin(math.pi * progress) ** 0.8)

        return float(base_pos[0] + dx), float(base_pos[1] + dy), float(base_pos[2] + dz)