import math
import numpy as np
import cv2
from typing import Optional, Tuple, List

import solar_panels_img_processing.utils.dsef.linetools as linetools
import solar_panels_img_processing.utils.dsef.dsef2_tools as df2t

class Dsef:
    def __init__(self, im: np.ndarray, edge_direction: float, dir_span: int = 90, t_crit: Optional[float] = None, dtype=np.float64):
        self.dtype=dtype
        # Initialize the directional step edge filter bank
        self.DF                = df2t.DsefFilters(edge_direction=edge_direction, dir_span=dir_span)
        self.edge_direction    = edge_direction
        # Pad image
        # PS! numpy arrays are row major
        self.upad, self.vpad   = math.ceil(self.DF.R+self.DF.radius), math.ceil(self.DF.R+self.DF.radius)
        self.Nv, self.Nu       = im.shape[0:2]  # TODO: Check that this is correct
        self.im_pad            = np.pad(im, (self.vpad, self.upad))
        self.mask_pad          = np.pad(np.ones_like(im),(self.vpad, self.upad))
        # kernels                
        self.u, self.v         = None, None
        self.ep2D              = df2t.gen_epanechnikov2D_kernel(self.DF.radius)
        self.ep2D_sq_sum       = np.sum(self.ep2D**2)
        # Calculate the threshold values based on image statistics
        # For a Poisson distribution, the variance equals the mean
        # TODO: PoiVar should be calculated from the image, somehow?
        self.Neff              = (np.sum(self.ep2D)**2) / np.sum(self.ep2D**2)  # Kish Effective sample size
        PoiVar                 = 150.0
        dof_edge               = df2t.calc_dof(PoiVar, PoiVar, self.Neff)
        dof_end                = df2t.calc_dof(PoiVar, PoiVar, self.Neff)
        CRIT_FAC=2
        self.crit_edge         = t_crit if t_crit is not None else CRIT_FAC * df2t.get_t_critical(df=dof_edge)
        self.crit_end          = t_crit if t_crit is not None else CRIT_FAC * df2t.get_t_critical(df=dof_end)
        
    def _within_bounds(self, u: int, v: int) -> None:
        """
        Check if pixel coordinate are within image bounds
        """
        if u >= self.upad and u < self.Nu + self.upad and v >= self.vpad and v < self.Nv + self.vpad:
            return True
        return False

    def step(self, du: int, dv: int) -> bool:
        """
        Step filter in given direction. In pixel coordinates.
        """
        return self.move(self.u_float + du - self.upad, self.v_float + dv - self.vpad)
    
    def move(self, u: int, v: int) -> bool:
        """
        Move filter to new location, in pixel coordinates.
        """
        u_new = round(u + self.upad)
        v_new = round(v + self.vpad)
        if not self._within_bounds(u_new, v_new):
            return False
        else:
            self.u = u_new
            self.v = v_new
            self.u_float = u + self.upad
            self.v_float = v + self.upad
            return True
    
    def get_pos(self) -> Tuple[int, int]:
        """ 
        Return position of filter in pixel coordinates (unpadded)
        """
        return self.u-self.upad, self.v-self.vpad

    def find_best_direction(self) -> Tuple[int, int]:
        """
        Find the best direction given the current LUT direction and span
        """
        # Get span
        sel_dirs, sel_items, sel_dirvecs = self.DF.flut.get_span()
        # Calculate T_FORWARD for current flut span
        t = [df2t.dsef_test(self, self.u, self.v, direction, edge_direction = 0, FORWARD=True).FORWARD for direction in sel_dirs]    
        # Find best direction to follow
        ind_max = np.argmax(t)
        if (t[ind_max] < t[len(sel_dirs)//2] + self.crit_edge):  # Stay on previous track
            ind_max = len(sel_dirs)//2
        return sel_dirs[ind_max], sel_dirvecs[ind_max]
     
    def EdgeSearch(self, start: Tuple[int, int], stop: Tuple[int, int], img: np.ndarray, OPTIMIZE_SEARCH_DIRECTION: bool = False) -> Tuple[bool, Tuple[int,int], np.ndarray]:
        """
        Search for edge. 
        Can return multiple edge candidates.
        """
        # Step along profile until edge is found, or end of profile/image is reached
        step = (self.DF.radius + min(self.DF.bu, self.DF.bv))

        # Calculate search direction vector
        u_new, v_new   = start
        u_edge, v_edge = start
        v_dir          = [stop[0]-u_new, stop[1]-v_new]
        d              = (v_dir[0]**2 + v_dir[1]**2)**0.5
        v_heading      = [v_dir[0]/d, v_dir[1]/d]

        MAX_EDGE = 0
        v = v_dir

        while v_heading[0]*v[0] + v_heading[1]*v[1] > 0:       # until we have passed the stop point
            
            if not self.step(step*v_heading[0], step*v_heading[1]):
                break
            u_new, v_new = self.get_pos()
            image = cv2.circle(img,(int(u_new),int(v_new)),1,(0,0,0),-1)
            
            if OPTIMIZE_SEARCH_DIRECTION:
                self.edge_direction = self.find_best_direction()
            c,r    = self.get_pos()
            T_FULL = df2t.dsef_test(self, self.u, self.v, self.edge_direction, FULL=True).FULL
            EDGE   = T_FULL > self.crit_edge
            # Return if edge is found
            # Refine position nefore returning            
            if EDGE:
                if T_FULL > MAX_EDGE + self.crit_edge:
                    MAX_EDGE = T_FULL
                    u_edge, v_edge = u_new, v_new
                    image = cv2.circle(img,(int(u_edge),int(v_edge)),1,(0,0,0),-1)
                elif MAX_EDGE > 0 and T_FULL < MAX_EDGE - self.crit_edge:
                    # Edge found !!!
                    self.move(u_edge, v_edge)                    
                    image = cv2.circle(img,(int(u_edge),int(v_edge)),1,(0,0,0),-1)
                    break
                
            # Calculate vector from current position to end of profile
            v            = [stop[0]-u_new, stop[1]-v_new]  
            image = cv2.circle(img,(int(u_edge),int(v_edge)),3,(0,0,0),-1)      
        return EDGE, self.get_pos(), image

    def EdgeFollow(self, img: np.ndarray, Ntest_edge: int = 5, MAX_ITT: int = 9000) -> Tuple[bool, bool, List[float], str, List[int], List[int], np.ndarray]:
        """
        Follow edge until end of line, or image boundary, or maximum number of iterations reached.
        """
        # Step lenght must equal a full filter kernel width
        step   = 2*self.DF.radius
        #self.DF.flut.set_span(self.edge_direction, dir_span)
        sel_dirs, sel_items, sel_dirvecs = self.DF.flut.get_span()
        consec_edge, consec_no_edge = 0, 0
        CONSEC_END = 0
        message = None
        self.u_edge, self.v_edge = self.get_pos()
        EDGE_FOUND, END_FOUND, END      = False, False, False
        
        # Abort edge search if accurate enough. Set to false will run until end
        ABORT_WHEN_ACCURATE = True
        # Running Weighted Vector Average (REWA)
        REWA = linetools.RunningExponentialVectorAverage(var=np.array([2,2]), rho=0.1)
        REWA.push(sel_dirvecs[len(sel_dirvecs)//2])
        MAX_T = self.crit_end

        Nitt = 0
        us, vs = [], []
        while Nitt < MAX_ITT:
            Nitt += 1            
            ui, vi = self.get_pos()
            image = cv2.circle(img,(int(ui),int(vi)),1,(128,128,128),-1)
            us.append(ui)
            vs.append(vi)

            # Calc T_FORWARD for all directions in current flut span
            t = [df2t.dsef_test(self, self.u, self.v, direction, FORWARD=True, FULL=True).FULL for direction in sel_dirs]    
            
            # Find best direction to follow
            ind_max = np.argmax(t)
            if (t[ind_max] < t[len(sel_dirs)//2] + self.crit_edge):  # Stay on previous track
                ind_max = len(sel_dirs)//2
            v     = sel_dirvecs[ind_max]
            T_FORWARD = t[ind_max]

            T_ALL = df2t.dsef_test(self, self.u, self.v, sel_dirs[ind_max], ALL=True).ALL            
            ALL_EDGE    = np.all(np.array(T_ALL) > self.crit_edge)

            # FORWARD_EDGE = T_FORWARD > self.crit_edge

            # Pixel edge test
            if ALL_EDGE:
                consec_edge = min(consec_edge+1, Ntest_edge)
                consec_no_edge = 0
            else:
                consec_edge = max(0, consec_edge-1)
                consec_no_edge += 1
            
            
            # Check if we lost the edge
            if consec_no_edge >= 2*self.DF.N+1:
                message = "CANCEL. WE LOST THE EDGE"
                print(message)
                break

            # If on a continous edge, we update the edge statistics.
            # Keep first point, truly on the edge.
            if consec_edge >= Ntest_edge:
                if not EDGE_FOUND:
                    self.u_edge, self.v_edge = self.get_pos()
                    EDGE_FOUND=True
                REWA.push(v)
                mu_direction  = self.DF.flut.wrap_angle(linetools.calc_heading(REWA.mu))
                var_direction = REWA.var[0]**2 
                var_direction = np.degrees(np.arctan(REWA.var[1]/(1+REWA.var[0])))
                
                #var_direction = self.DF.flut.wrap_angle(linetools.calc_heading(REWA.var))
                # Change average direction
                if var_direction > self.DF.flut.d_theta:
                    self.DF.flut.set_span(mu_direction, 4*var_direction**0.5)
                    sel_dirs, sel_items, sel_dirvecs = self.DF.flut.get_span()
                elif ABORT_WHEN_ACCURATE:
                    print("accurate")
                    # Abort if edge direction estimate is good enough
                    message = "ABORTING. Accurate edge direction estimated within +/- %0.1f degrees" % (var_direction**0.5)
                    break 

            # Step the filter forward
            if not self.step(v[0]*step, v[1]*step):
                END_FOUND = False
                message = "we reached END of image"
                print(message)
                break     
        # Return edge_direction_vector       
        #mu_dir = self.DF.flut.wrap_angle(linetools.calc_heading(REWA.mu))
        return EDGE_FOUND, END_FOUND, REWA, message, us, vs, image