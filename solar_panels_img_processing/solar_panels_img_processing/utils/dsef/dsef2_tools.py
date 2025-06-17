import logging
import math
from typing import List, Tuple, Any
import numpy as np
from scipy import stats

import solar_panels_img_processing.utils.dsef.linetools as linetools

log = logging.getLogger( "Debug" )

def get_t_critical(df: int = 10, p: float = 0.001) -> np.ndarray:
    # Get critical value for one sided t test
    return stats.t.ppf(1-p, df)


def calc_dof(Lvar: float, Rvar: float, Neff: np.ndarray) -> np.ndarray:
    """
    Calculate degrees of freedom (dof)for weighted average.
    Lvar, Rvar - variance for the two sample sets
    Neff       - effective sample set
    https://codingdisciple.com/hypothesis-testing-welch-python.html
    """
    return np.floor(((Lvar+Rvar)/Neff)**2 / (((Lvar/Neff)**2)/(Neff-1) + ((Rvar/Neff)**2)/(Neff-1))).astype(int)
    
# Compass heading tools
def calc_heading_vector(heading_deg: float, dtype=np.float64) -> np.ndarray:
    """
    Calculate a heading vector, given compass heading
    0, 90, 180, 270 - due north, east, south and west
    """
    t = np.radians(heading_deg)
    v = np.array([np.sin(t), np.cos(t)], dtype)
    return v/np.linalg.norm(v)

def calc_heading(heading_vec: np.ndarray) -> np.ndarray:
    """
    Calculate heading from 2D heading vector.
    Heading is defined from 0 to 360 deg
    """
    ve, vn = heading_vec    
    return (np.degrees(np.arctan2(ve, vn)) + 360) % 360.0

def heading2rotM(heading_deg: float) -> np.ndarray:
    # Calculate counter clockwise rotation matrix
    heading_rad = math.radians(heading_deg)
    c = math.cos(heading_rad)
    s = math.sin(heading_rad)
    return np.array([[c, s], [-s, c]])

def epanechnikov1D(u, epa_norm = 3/4.0):
    return epa_norm*(1 - u**2)*(abs(u) <= 1)*1.0

def gen_epanechnikov2D_kernel(r2: np.ndarray, INDEX: bool = False) -> np.ndarray:
    # Generate the kernel
    x,y  = np.meshgrid(range(0,2*r2+1), range(0,2*r2+1))
    u    = (np.sqrt(((x - r2)/r2) ** 2 + ((y - r2)/r2) ** 2))
    ep   = epanechnikov1D(u)
    ep  /= np.sum(ep)
    if INDEX:
        ind  = np.where(abs(u.ravel()) <= 1)
        return x.ravel()[ind], y.ravel()[ind], ep.ravel()[ind]
    else:
        return ep

class FlutDir:
    """
    Super Fast lookup of, items and direction vectors, 
    divided up into equidistant bins for a full 360 degree coverage. 
    """
    def __init__(self, d_theta: float, center: int = 0):
        self.dir_span = 360
        self.center   = center
        k             = int(self.dir_span/(2*d_theta) - 0.5) + 1
        self.d_theta  = self.dir_span/(2*k) # Make sure there are no remainder
        self.thetas   = [-k*self.d_theta + t*self.d_theta + self.center for t in range(0, 2*k+1)]    
        self.items    = [None]*len(self.thetas)
        self.dir_vec  = [linetools.calc_heading_vector(theta) for theta in self.thetas]
        self.reset_span()

    def reset_span(self) -> None:
        """
        Reset to initial span
        """
        self.dir_span = 360
        self.inds = [ind for ind in range(len(self.thetas))]     

    def set_item(self, theta: float, item: float) -> None:
        """
        Assign item to angle bin
        """
        index = self.index(theta)
        self.items[index] = item 

    def set_span(self, center: float, dir_span: int) -> Tuple[float, int, List[float]]:
        """
        Set span of LUT, as [center-dir_span/2, center+dir_span/2]
        """
        if dir_span >= 360 - self.d_theta:
            raise IOError("You cant set the FLUT span >= 360-d_theta")
        self.dir_span = dir_span
        center    = self.wrap_angle(center)  # Just to make sure angle is within -180, 180
        ind       = int((center - self.thetas[0])/self.d_theta + 0.5)
        center    = self.thetas[ind]         # neares actual center angle
        kk        = int(dir_span/(2*self.d_theta) + 0.5)+1# Number of cells on each side of center   
        thetas    = [self.wrap_angle(center + k*self.d_theta) for k in range(-kk+1, kk)] # The directions
        self.inds = [int((theta - self.thetas[0])/self.d_theta + 0.5) for theta in thetas] # The indeces
        return center, self.inds, [self.thetas[ind] for ind in self.inds]

    def get_span(self) -> Tuple[List[float], List[float], List[float]]:
        """
        Get current angle, item and dir_vecs span in LUT.
        """
        return [self.thetas[ind] for ind in self.inds], [self.items[ind] for ind in self.inds], [self.dir_vec[ind] for ind in self.inds]

    def unwrap_angle(self, angle: float) -> float:
        """
        Unwrap to list of monotonic increasing angles. 
        Pay special attention to end points.
        """
        if angle < self.thetas[self.inds[0]]:
            if abs(angle - self.thetas[self.inds[0]]) > abs(angle - self.thetas[self.inds[-1]]):
                return angle + 360
            else:
                return self.thetas[self.inds[0]]
        else:
            return angle

    def wrap_angle(self, angle: float) -> float:
        """
        Keep angle between -180, 180
        """
        angle = angle % 360
        if (angle <= -180):
            angle += 360
        elif (angle > 180): 
            angle -= 360
        return angle

    def index(self, direction: float) -> float:
        """
        Return index of nearest angle in LUT, relative to current span.      
        clip to prevent index overflow   
        """        
        direction = self.wrap_angle(direction)
        ind       = int((self.unwrap_angle(direction) - self.thetas[self.inds[0]])/self.d_theta + 0.5)
        return self.inds[min(max(0, ind), len(self.inds)-1)]

    def get_nearest(self, direction: float) -> float:        
        """
        INPUTS:
            direction      : direction relative to current span
        Return nearest angle, item and direction vector in LUT
        """
        ind              = self.index(direction)
        return self.thetas[ind], self.items[ind], self.dir_vec[ind]
    
class DsefFilters:
    """
    Directional Step Edge Follower (DSEF) Class. Represented as a filter bank.
    """
    def __init__(self, edge_direction: int = 0, radius: int = 15, N: int = 3, bu: int = 0, bv: int = 0, dir_span: int = 90):
        self.radius = radius                         # Radius per circular filter area
        self.N      = N                              # Number of filters in each direction
        self.bu     = bu                             # Offset along u axis
        self.bv     = bv                             # Offset along v axis
        u1          = bu+radius                      # filter center right
        v1          = bv+(2*self.N-1)*radius         # filter center top        
        self.R      = (u1**2 + v1**2)**0.5           # Radius of entire filter bank        
        d_theta     = np.degrees(self.radius/self.R) # step size between filter bank directions
        self.flut   = FlutDir(d_theta)               # Initialize fast LUT
        # Initialize the filter bank. Full 360 degrees
        self._ini_filters()                         
        # Set filter bank span and direction
        self.set_direction(edge_direction, dir_span) # Set filterbank main direction and span
        
    def set_direction(self, edge_direction: float, dir_span: float) -> None:
        """
        Set filterbank main direction, calculate normal vector and calculare 
        relative positions along edge normal for edge position refinement
        """
        # Set FLUT span
        self.edge_direction        = edge_direction
        self.flut.set_span(self.edge_direction, dir_span)
        # Relative positions along normal vector, used for refining position
        self.edge_direction_vector = linetools.calc_heading_vector(edge_direction)
        self.edge_normal_vector    = linetools.normalvector(self.edge_direction_vector)
        u0,v0 = -self.radius*self.edge_normal_vector
        u1,v1 = self.radius*self.edge_normal_vector
        length = int(np.hypot(u1-u0, v1-v0) + 0.5)            
        self.edge_refine_pos = [np.linspace(u0, u1, length, dtype=int), np.linspace(v0, v1, length, dtype=int)]

    def _calc_regs(self, heading_deg: float, dtype=int) -> Tuple[float, Tuple[float,float,float,float], np.ndarray]:
        """
        Calculate all region coordinates given a specific compass heading in degrees
        """
        R  = heading2rotM(heading_deg)
        RU = np.round(R.dot(self._RU.T).T).astype(dtype)  # Right-upper
        RD = np.round(R.dot(self._RD.T).T).astype(dtype)  # Right-down
        LU = np.round(R.dot(self._LU.T).T).astype(dtype)  # Left-upper
        LD = np.round(R.dot(self._LD.T).T).astype(dtype)  # Left-down
        return heading_deg, (RU, RD, LU, LD), linetools.calc_heading_vector(heading_deg)
    
    def _ini_filters(self) -> None:
        # Initialize rois for 0 heading
        # RU - right-up, RD - right-down, LU - left up, LD - left down
        self._RU   = np.c_[self.N*[self.bu  + self.radius], [(1+2*k)*self.radius   + self.bv for k in range(0, self.N)]]
        self._RD   = np.c_[self.N*[self.bu  + self.radius], [-(1+2*k)*self.radius  - self.bv for k in range(0, self.N)]]
        self._LU   = np.c_[self.N*[-self.bu - self.radius], [(1+2*k)*self.radius  + self.bv for k in range(0, self.N)]]
        self._LD   = np.c_[self.N*[-self.bu - self.radius], [-(1+2*k)*self.radius - self.bv for k in range(0, self.N)]]
        self.max_r = np.ceil(np.linalg.norm(self._RU[-1]) + self.radius - 1).astype(int)  # Max radius of filter bank, for any angles        
        for ind in self.flut.inds:
            theta, (RU, RD, LU, LD), v = self._calc_regs(self.flut.thetas[ind])     # FIXME
            self.flut.set_item(theta, [RU, RD, LU, LD])
    
class DataClass():
    """
    My very own DataClass.
    Return the attribute value if defined, if not return None
    """
    def __init__(self,**kwargs):
        self.__dict__.update(kwargs)
    def __getattr__(self, item): # All unset attributes defaults to None
        return None
    def __repr__(self):
        return str(self.__dict__)

def nw(c: np.ndarray, r: np.ndarray, kernel: np.ndarray, im_pad: np.ndarray, mask_pad: np.ndarray, dtype=np.float64) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculate nadaraya watson estimate for mean and variance
    within roi = [us, vs]
    In camera coordinates; u,v -> in numpy matrix; row, column
    """
    Nw_r, Nw_c     = kernel.shape    
    minn_r, maxx_r = r - (Nw_r // 2), r + Nw_r - (Nw_r // 2)
    minn_c, maxx_c = c - (Nw_c // 2), c + Nw_c - (Nw_c // 2)
    den            = np.sum(kernel * mask_pad[minn_r: maxx_r, minn_c: maxx_c].astype(dtype))
    if den != 0:
        nw     = np.sum(kernel * im_pad[minn_r : maxx_r, minn_c : maxx_c]) / den
        nw_sq  = np.sum((kernel * im_pad[minn_r : maxx_r, minn_c : maxx_c]**2.0)) / den                        
        nw_var = nw_sq - nw**2.0
    else:
        nw     = 0
        nw_var = 0
    return nw, nw_var


def nw_calc(E: Any, rcL: np.ndarray, rcR: np.ndarray, dtype=np.float64) -> float:
    """
    Nadaraya-Watson. Coordinates are relative to the padded numpy array.
    """
    L_nw          = [nw(ri,ci,E.ep2D,E.im_pad, E.mask_pad) for ri,ci in rcL]
    L_mu, L_var   = np.mean(np.array(L_nw, dtype)[:,0]), np.mean(np.array(L_nw, dtype)[:,1])
    R_nw          = [nw(ri,ci,E.ep2D,E.im_pad, E.mask_pad) for ri,ci in rcR]
    R_mu, R_var   = np.mean(np.array(R_nw, dtype)[:,0]), np.mean(np.array(R_nw, dtype)[:,1])
    RL_var        = L_var/E.Neff + R_var/E.Neff
    if RL_var < 1e-6:
        # One or more filters are in the zero padded region
        return 0
    else:
        return (R_mu - L_mu)/np.sqrt(RL_var)    

def dsef_test(E: Any, u: int, v: int, direction: float, ALL: bool = False, FULL: bool = False, FORWARD: bool = False, REAR: bool = False, END: bool = False, USE_LUT: bool = True) -> DataClass:
    """
    Calculate statistical tests for DSEF searcher.
    u,v are relative to padded image.
    Direction is relative to current span.
    """
    T = DataClass()
    if USE_LUT:
        theta, (RU, RD, LU, LD), _ = E.DF.flut.get_nearest(direction)
    else: 
        theta, (RU, RD, LU, LD), _ = E.DF._calc_regs(direction)  

    # Full edge test
    if FULL:
        rcL           = np.r_[LU, LD] + np.array([u, v])
        rcR           = np.r_[RU, RD] + np.array([u, v])
        T.FULL        = nw_calc(E, rcL, rcR)
    # Forward edge test    
    if FORWARD:
        rcL           = np.r_[LU] + np.array([u, v])
        rcR           = np.r_[RU] + np.array([u, v])
        T.FORWARD    = nw_calc(E, rcL, rcR)
    # Rear edge test
    if REAR:
        rcL           = np.r_[LD] + np.array([u, v])
        rcR           = np.r_[RD] + np.array([u, v])
        T.REAR        = nw_calc(E, rcL, rcR)
    # End test - Test one single regions from Right up (RU[0]) side against right down side (RD)
    if END:
        rcD           = np.r_[RD] + np.array([u, v])
        rcU           = np.r_[RU] + np.array([u, v])
        rcU           = np.r_[[RU[-1]]] + np.array([u, v])        
        T.END         = nw_calc(E, rcU, rcD)

    # Test if all right > left
    if ALL:
        rcL           = np.r_[LU, LD] + np.array([u, v])
        rcR           = np.r_[RU, RD] + np.array([u, v])
        T.ALL         = [nw_calc(E, [l], [r]) for l,r in zip(rcL, rcR)]

    
    return T

