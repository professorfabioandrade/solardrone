import math
import numpy as np
from typing import Optional, List

class Georeferencing:
    def __init__(self, k_matrix: List[List[float]], solar_panel_height: float, solar_panel_tilt: float):
        self.utm: List[Optional[float]] = [None, None, None]
        self.psi_NED: Optional[float] = None

        self.gimbal: List[Optional[float]] = [None, None, None] # roll, pitch, yaw

        # Camera intrinsic matrix
        self.K = np.array(k_matrix)

        self.z_ENU = solar_panel_height
        self.solar_panel_tilt = solar_panel_tilt
        
    def update_position(self, utm: List[Optional[float]]) -> None:
        self.utm = utm

    def update_heading(self, psi_NED: Optional[float]) -> None:
        self.psi_NED = psi_NED

    def update_gimbal(self, roll: float, pitch: float, yaw: float) -> None:
        self.gimbal = [roll, pitch, yaw]

    def __call__(self, u: float, v: float) -> np.ndarray:
        if self.psi_NED is None or any(coord is None for coord in self.utm):
            raise ValueError("Missing UTM or heading data.")

        
        i = np.array([[u], [v], [1]])

        # Camera frame vector prime
        P_C_prime = np.linalg.inv(self.K) @ i

        # Camera frame to Gimbal frame
        R_C_to_G = np.array([
            [0, 0, 1],
            [1, 0, 0],
            [0, 1, 0]
        ])
        P_G_prime = R_C_to_G @ P_C_prime

        # Gimbal to UAS frame (with predefined yaw, pitch, roll)
        roll = 0 * math.pi/180
        pitch = -self.solar_panel_tilt * math.pi/180 
        yaw = 0 * math.pi/180 
        
        

        R_G_to_UAS = self.create_rotation_matrix(yaw, pitch, roll)
        P_UAS_prime = R_G_to_UAS @ P_G_prime

        # UAS to NED frame
        yaw = self.psi_NED
        pitch = 0
        roll = 0

        R_UAS_to_NED = self.create_rotation_matrix(yaw, pitch, roll)
        P_NED_prime = R_UAS_to_NED @ P_UAS_prime

        # NED to ENU frame
        R_NED_to_ENU = np.array([
            [0, 1, 0],
            [1, 0, 0],
            [0, 0, -1]
        ])
        T_NED_to_ENU = np.array([
            [self.utm[0]],
            [self.utm[1]],
            [self.utm[2]]
        ])

        P_ENU_prime = R_NED_to_ENU @ P_NED_prime + T_NED_to_ENU

        # Translation matrix T
        T = T_NED_to_ENU + R_NED_to_ENU @ R_UAS_to_NED @ R_G_to_UAS @ R_C_to_G @ np.array([[0], [0], [0]])

        # Calculating z_C
        self.z_ENU = 2.15  # Object height ## REFATORAR
        z_T = T[2]
        z_ENU_prime = P_ENU_prime[2]
        z_C = (self.z_ENU - z_T) / (z_ENU_prime - z_T)

        # Final ENU coordinates
        P_ENU = z_C * P_ENU_prime - z_C * T + T

        return P_ENU

    @staticmethod
    def create_rotation_matrix(yaw: float, pitch: float, roll: float) -> np.ndarray:
        return np.array([
            [math.cos(yaw) * math.cos(pitch),
             -math.sin(yaw) * math.cos(roll) + math.cos(yaw) * math.sin(pitch) * math.sin(roll),
             math.sin(yaw) * math.sin(roll) + math.cos(yaw) * math.cos(roll) * math.sin(pitch)],

            [math.sin(yaw) * math.cos(pitch),
             math.cos(yaw) * math.cos(roll) + math.sin(roll) * math.sin(pitch) * math.sin(yaw),
             -math.cos(yaw) * math.sin(roll) + math.sin(pitch) * math.sin(yaw) * math.cos(roll)],

            [-math.sin(pitch),
             math.cos(pitch) * math.sin(roll),
             math.cos(pitch) * math.cos(roll)]
        ])
