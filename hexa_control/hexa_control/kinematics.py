#!/usr/bin/env python3
import math

class Kinematics:
    def __init__(self, l1=0.051, l2=0.110, l3=0.148):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

        self.femur_zero_angle = math.radians(-35.0)
        self.tibia_zero_angle = math.radians(-60.0)
        self.psi_zero = math.pi - abs(self.tibia_zero_angle - self.femur_zero_angle)

        self.limit_min = -2.094
        self.limit_max = 2.094

    def inverse_kinematics(self, x, y, z):

        theta1 = math.atan2(y, x)

        d = math.sqrt(x**2 + y**2) - self.l1
        D = math.sqrt(d**2 + z**2)

        cos_psi = (self.l2**2 + self.l3**2 - D**2) / (2.0 * self.l2 * self.l3)
        cos_psi = max(-1.0, min(1.0, cos_psi))
        psi = math.acos(cos_psi)

        theta3 = self.psi_zero - psi

        alpha1 = math.atan2(z, d)
        cos_alpha2 = (self.l2**2 + D**2 - self.l3**2) / (2.0 * self.l2 * D)
        cos_alpha2 = max(-1.0, min(1.0, cos_alpha2))
        alpha2 = math.acos(cos_alpha2)

        phi_femur = alpha1 + alpha2
        theta2 = phi_femur - self.femur_zero_angle

        theta1 = max(self.limit_min, min(self.limit_max, theta1))
        theta2 = max(self.limit_min, min(self.limit_max, theta2))
        theta3 = max(self.limit_min, min(self.limit_max, theta3))

        return theta1, theta2, theta3

    def forward_kinematics(self, theta1, theta2, theta3):
        phi_femur = theta2 + self.femur_zero_angle
        psi = self.psi_zero - theta3
        phi_tibia = phi_femur - (math.pi - psi)
        d = self.l2 * math.cos(phi_femur) + self.l3 * math.cos(phi_tibia)
        z = self.l2 * math.sin(phi_femur) + self.l3 * math.sin(phi_tibia)
        r_total = self.l1 + d
        x = r_total * math.cos(theta1)
        y = r_total * math.sin(theta1)

        return x, y, z