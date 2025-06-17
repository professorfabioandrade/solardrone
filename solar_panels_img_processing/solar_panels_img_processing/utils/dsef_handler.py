import time
import cv2
import numpy as np

from typing import Optional, List, Tuple

import solar_panels_img_processing.utils.dsef.dsef2 as dsef2

class DSEFHandler():
    def __init__(self, wp_heading: float) -> None:
        self.wp_heading = wp_heading

    def __call__(self, img: np.ndarray) -> Optional[List[Tuple[float, float]]]:
        return self.get_center_line(img)
    
    @staticmethod
    def convert_image(img: np.ndarray, ORG: bool = False) -> Tuple[np.ndarray,np.ndarray,np.ndarray]:
        im_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        mask   = np.ones(im_rgb[:, :, 0].shape, dtype=int)
        im_hsv = cv2.cvtColor(im_rgb, cv2.COLOR_RGB2HSV_FULL)
        if not ORG:
            return im_hsv[:, :, 0], mask # Only Hue and mask
        else:
            return im_hsv[:, :, 0], mask, im_rgb
        
    def get_star_trajectory(self,img,start,stop,edge_direction,index):
        
        # Step-1: Convert image from GBR to RGB
        im, _, _ = self.convert_image(img, ORG=True)
        
        if im is not None:
            cv2.imshow("HSV", im)
            cv2.waitKey(1)  # Add a delay to allow the image to be rendered
        # Step-2: Initialize the DSEF
        E  = dsef2.Dsef(im, edge_direction, dir_span=90)
        
        # Step-3: Find the edge
        t0 = time.time()
        E.move(start[0], start[1])
        _, [u_edge, v_edge], image = E.EdgeSearch(start, stop, img, OPTIMIZE_SEARCH_DIRECTION=False)
                
        # Step-4: Follow the edge
        EDGE_FOUND, _, REWA, _, _, _, image = E.EdgeFollow(image, Ntest_edge=2)
        
        if EDGE_FOUND:
            print(f"REWA.mu: {REWA.mu}")
            if index == 1:
                image = cv2.line(image,(int(u_edge),int(v_edge)),(int(u_edge+100*REWA.mu[0]),int(v_edge+100*REWA.mu[1])),(0,0,255),2)
                image = cv2.circle(image,(int(u_edge),int(v_edge)),4,(0,0,255),-1)
            else:
                image = cv2.line(img,(int(u_edge),int(v_edge)),(int(u_edge+100*REWA.mu[0]),int(v_edge+100*REWA.mu[1])),(255,0,0),2)
                image = cv2.circle(image,(int(u_edge),int(v_edge)),4,(255,0,0),-1)
            return ([int(u_edge),int(v_edge),int(u_edge+500*REWA.mu[0]),int(v_edge+500*REWA.mu[1])]), image
        else:
            return None, image
    
    def get_center_line(self, img: np.ndarray) -> Optional[List[Tuple[float, float]]]:
        #star1, _ = self.get_star_trajectory(img,[1920/2,1080/2],[1920,1080],90.0,1)
        star2, img2 = self.get_star_trajectory(img,[200, 200],[1920,3*1080/4],270.0,2)        
        
        self.star2_img = img2
        self.center_image = None
        if star2:
            return [(float(star2[0]),float(star2[1])),(float(star2[2]),float(star2[3]))]
        
        else:
            return None