import numpy as np

from typing import List, Tuple

def normalized(v: List[float]) -> np.ndarray:
    norm = (v[0]**2 + v[1]**2)**0.5
    return np.array([v[0]/norm, v[1]/norm])

def normalvector(v: List[float], CC: bool = True, NORMALIZE: bool = True) -> np.ndarray:
    """
    Calculate normalized, normal vector, in counter-clockwise direction be default
    """
    vx = [v[1], -v[0]] if CC else [-v[1], v[0]]
    if NORMALIZE:        
        return normalized(vx)
    else:
        return vx

class RunningExponentialVectorAverage:
    """
    Calculate running exponential vector average (REVA)
    """
    def __init__(self, mu: np.ndarray = np.array([0, 0]), var: np.ndarray = np.array([0, 0]), rho: float = 0.1):
        self.mu, self.var, self.rho = mu, var, rho

    def push(self, v: float) -> None:
        """
        Add vector. If w is None, the vector length is used as weight.
        """
        self.mu = self.rho*v + (1 - self.rho)*self.mu
        d        = abs(v - self.mu)
        self.var = (d**2)*self.rho + (1 - self.rho)*self.var

    def __repr__(self) -> str:
        return("REWA: mu = [%0.1f, %0.1f], var = [%0.1f, %0.1f], rho = %0.1f" % (self.mu[0], self.mu[1], self.var[0], self.var[1], self.rho))


def calc_heading_vector(heading_deg: float, dtype=np.float64) -> np.ndarray:
    """
    Calculate a heading vector, given compass heading
    0, 90, 180, 270 - due north, east, south and west
    """
    t = np.radians(heading_deg)
    return np.array([np.sin(t), np.cos(t)], dtype)

def calc_heading(heading_vec: Tuple[float]) -> float:
    """
    Calculate heading from 2D heading vector.
    Heading is defined from 0 to 360 deg
    """
    ve, vn = heading_vec    
    return (np.degrees(np.arctan2(ve, vn)) + 360) % 360.0