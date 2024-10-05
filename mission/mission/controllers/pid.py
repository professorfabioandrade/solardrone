from typing import List, Tuple
import math
import numpy as np


class PID():
    def __init__(self, kp: float, ki: float, kd: float, max_integral: float = 9999) -> None:
        
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.max_integral = max_integral
        
        self.reset()

    # ------ Properties ------ #

    @property
    def kp(self) -> float:
        return self._kp
    
    @property
    def ki(self) -> float:
        return self._ki
    
    @property
    def kd(self) -> float:
        return self._kd
    
    # ------ Setters ------ #

    @kp.setter
    def kp(self, new_kp: float) -> None:
        if not -100 <= new_kp <= 100:
            raise ValueError(f"Kp value wrong! Expecting value between -100 and 100, received {new_kp}")
        self._kp = new_kp

    @ki.setter
    def ki(self, new_ki: float) -> None:
        if not -100 <= new_ki <= 100:
            raise ValueError(f"Ki value wrong! Expecting value between -100 and 100, received {new_ki}")
        self._ki = new_ki

    @kd.setter
    def kd(self, new_kd: float) -> None:
        if not -100 <= new_kd <= 100:
            raise ValueError(f"Kd value wrong! Expecting value between -100 and 100, received {new_kd}")
        self._kd = new_kd

    # ------ PID Functions ------ #
    
    def calculate(self, error, dt):
        self.integral += error * dt
        if self.integral > self.max_integral:
            self.integral = self.max_integral
        elif self.integral < -self.max_integral:
            self.integral = -self.max_integral
        derivative = (error - self.previous_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output
    
    def reset(self) -> None:
        self.previous_error = 0
        self.integral = 0

    @staticmethod
    # Calculate the lateral shift error (perpendicular distance) from the drone to the line
    def calculate_lateral_error(drone_position: Tuple[float,float], line_start: List[float], line_end: List[float]):
        
        line_vec = np.subtract(line_end, line_start)
        drone_vec = np.subtract(drone_position, line_start)
        
        # Normalize the line vector
        line_unit_vec = line_vec / np.linalg.norm(line_vec)
        
        # Project the drone's position onto the line
        projection_length = np.dot(drone_vec, line_unit_vec)
        projection_vec = projection_length * line_unit_vec
        
        # Calculate the lateral shift error (perpendicular distance)
        error_vec = np.subtract(drone_vec, projection_vec)
        lateral_error = np.linalg.norm(error_vec)

        # Check if the error is to the left or right of the line (sign of error matters)
        if np.cross(line_vec, drone_vec) > 0:  # If z-component of cross product > 0
            lateral_error = -lateral_error  # Drone is to the left of the line
        
        return lateral_error

    @staticmethod
    # Calculate the yaw error (heading difference) between the drone's orientation and the line
    def calculate_yaw_error(coordinates_heading, drone_heading: float, line_start: List[float], line_end: List[float]):
        line_vec = np.subtract(line_end, line_start)
        desired_heading = np.arctan2(line_vec[1], line_vec[0]) % (2 * math.pi)  # Desired angle of the line
        
        if (coordinates_heading > 0 and coordinates_heading < math.pi):              
            psi_traj = drone_heading - math.pi/2
        else:
            psi_traj = drone_heading + math.pi/2
            
        yaw_error = desired_heading - psi_traj
        return desired_heading, yaw_error
    
    @staticmethod
    def calculate_heading(waypoint1: dict, waypoint2: dict):
        """
        Calculate the heading between two points using UTM coordinates.
        """
        # Convert the starting and ending lat/lon coordinates to UTM
        utm_start = (waypoint1['x'], waypoint1['y'])
        utm_end = (waypoint2['x'], waypoint2['y'])

        # Extract UTM Easting (x) and Northing (y) coordinates
        x1, y1 = utm_start[0], utm_start[1]
        x2, y2 = utm_end[0], utm_end[1]

        # Calculate the difference in the UTM coordinates
        delta_x = x2 - x1
        delta_y = y2 - y1

        # Calculate the heading using atan2 (which considers the quadrant)
        
        return math.atan2(delta_y, delta_x)
