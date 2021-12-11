#!/usr/bin/python3

import time
import cv2
import numpy as np
from picamera import PiCamera

class Ball_Trajectory:
    '''

    '''
    
    ### CLASS VARIABLES ###
    
    # define framerate, threshold value, and iso
    fr = 30
    thres = 150
    iso = 250

    # define camera full frame width and height in pixels
    cam_w = 1920
    cam_h = 1088

    # define resolution scaling factor
    res_factor = 1 # must evenly divide cam_w and cam_h
    res_w = int(cam_w/res_factor)
    res_h = int(cam_h/res_factor)
    res = (res_w, res_h)

    # define table bounds from full resolution
    cam_left = 351
    cam_right = 1605
    cam_top = 38
    cam_bottom = 1066

    # define table bound with scaled resolution
    left = int(cam_left/res_factor)
    right = int(cam_right/res_factor)
    top = int(cam_top/res_factor)
    bottom = int(cam_bottom/res_factor)
    
    # define frame width and height
    frame_w = right-left
    frame_h = bottom-top
    
    # define physical parameters
    bumper_h = 85 #[mm], bumper height
    table_h = 520#498 #[mm], table height (19.625in)
    pulley_d = 44 #[mm], pulley diameter with belt
    pulley_c = pulley_d*np.pi #[mm], pulley circumference
    cpr = 1250 # encoder counter per revolution
    
    # define conversion factors
    mm_per_pix = table_h/frame_h
    counts_per_mm = cpr/pulley_c
    
    # define counts offset due to bumper height
    counts_offset = counts_per_mm*bumper_h/2
    
    # define maximum and minimum setpoints
    max_setpoint = (table_h-bumper_h/2)*counts_per_mm
    min_setpoint = 0
    
    # define trajectory compute sensitivity
    d_min = 50 #[mm], minimum ball displacement for trajectory computation
    d_min_pix = d_min/mm_per_pix #[pix]
    
    ### PUBLIC METHODS ###
    
    def __init__(self, setpoint_q, print_pos=False):
        '''

        '''
        # create variables for setpoint queue
        self.setpoint_q = setpoint_q
        
        # create boolean variable to print positions
        self.print_pos = print_pos
        
        # create variables for image buffers
        self.raw_img = np.empty((self.res_h,self.res_w,3), dtype=np.uint8)
        self.bin_img = np.empty((self.frame_h,self.frame_w))
        
        # create variables for ball positions
        self.pos_prev = (int(0), int(0))
        self.pos_new = (int(0), int(0))
        self.pos_bumper = int(0)
        self.setpoint = int(0)
        
        # create list for trajectory points
        self.traj_list = []
        
        return
    
    def run(self):
        '''

        '''
        # create camera
        print('Creating camera...\n')
        self.camera = PiCamera(resolution=self.res, framerate=self.fr)
        
        # run camera setup sequence
        self.__camera_setup()
        
        while True:
            # update image
            self.__update_image()
            
            # get ball position
            self.__update_ball_position()
            
            # get bumper position
            self.__get_bumper_position()
            
            # put setpoint
            self.__put_setpoint()
        
        return
    
    def test(self):
        '''

        '''
        # create camera
        self.camera = PiCamera(resolution=self.res, framerate=self.fr)
        
        # run camera setup sequence
        self.__camera_setup()
        
        ### being test code here ###
        
        print('\nStarting in 0.5 seconds...\n')
        time.sleep(0.5)
        
        # get ball positions from 3 images
        self.__update_image()
        self.__update_ball_position()
        
        self.__update_image()
        self.__update_ball_position()
        
        self.__update_image()
        self.__update_ball_position()
        
        # get trajectory and bumper position
        self.__get_bumper_position()
        
        # plot trajectory
        self.__plot_trajectory()
        
        return
        
    def close(self):
        '''

        '''
        # close camera
        print('\nClosing camera.')
        self.camera.close()
        
        return
    
    ### PRIVATE METHODS ###
    
    def __camera_setup(self):
        '''

        '''
        print('\nSetting up camera...')
        
        # set camera iso
        self.camera.iso = self.iso

        # allow auto gain control to settle and fix parameters
        time.sleep(2)
        self.camera.shutter_speed = self.camera.exposure_speed
        self.camera.exposure_mode = 'off'
        gains = self.camera.awb_gains
        self.camera.awb_mode = 'off'
        self.camera.awb_gains = gains
        
        print('Finished camera setup.\n')
        
        return
    
    def __update_image(self):
        '''

        '''
        # update raw image from camera
        self.camera.capture(self.raw_img, format='bgr', use_video_port=True)
        
        # crop image
        cropped = self.raw_img[self.top:self.bottom,self.left:self.right,:]
        
        # convert to grayscale and blur
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3,3), 30, 30)
        
        # threshold to binary image
        _, self.bin_img = cv2.threshold(gray, self.thres, 255, cv2.THRESH_BINARY)
        
        return
    
    def __update_ball_position(self):
        '''

        '''
        # update previous position
        self.pos_prev = self.pos_new
        
        # find contours of binary image
        contours, _ = cv2.findContours(self.bin_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # find ball center using moments if contours exsists
        if contours:
            ball_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(ball_contour)
            if M["m00"] == 0.0:
                print('\nNo contours found.')
            else:
                self.pos_new = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                if self.print_pos:
                    print('\nprevious position: '+str(self.pos_prev))
                    print('new position:      '+str(self.pos_new))
        else:
            if self.print_pos:
                print('\nNo contours found.')
        
        return
    
    def __get_bumper_position(self):
        '''

        '''
        # define starting x and y positions and slope
        self.x0 = self.pos_new[0]
        self.y0 = self.pos_new[1]
        xdiff = self.pos_new[0]-self.pos_prev[0]
        ydiff = self.pos_new[1]-self.pos_prev[1]
        ddiff = np.sqrt(xdiff**2+ydiff**2)
        
        # check for division by zero and very small movement
        if xdiff == 0 or ddiff < self.d_min_pix:
            self.pos_bumper = self.y0
            return
        
        # compute slope
        self.slope = (self.pos_new[1]-self.pos_prev[1])/(self.pos_new[0]-self.pos_prev[0])
        
        # define direction relative to x axis
        if self.pos_new[0]>self.pos_prev[0]:
            self.xdir = True
        else:
            self.xdir = False
            
        # check for zero slope
        if self.slope == 0:
            self.pos_bumper = self.y0
            
            # add trajectory
            if self.xdir:
                self.traj_list.append((self.x0, self.y0))
                self.traj_list.append((self.frame_w, self.y0))
            else:
                self.traj_list.append((self.x0, self.y0))
                self.traj_list.append((0, self.y0))
                self.traj_list.append((self.frame_w, self.y0))
            
            return
        
        # compute trajectory if slope is not too steep
        if self.slope < self.frame_h/2:
            self.__compute_trajectory()
        else:
            self.pos_bumper = self.y0
        
        return
    
    def __put_setpoint(self):
        '''

        '''
        # compute setpoint
        #print(self.pos_bumper)
        self.setpoint = self.pos_bumper*self.mm_per_pix*self.counts_per_mm-self.counts_offset
        
        # check setpoint
        if self.setpoint > self.max_setpoint:
            self.setpoint = self.max_setpoint
        
        if self.setpoint < self.min_setpoint:
            self.setpoint = self.min_setpoint
        
        # put setpoint in queue
        #print('putting')
        self.setpoint_q.put(self.setpoint)
        
        return
    
    def __compute_trajectory(self):
        '''
        note: something weird going on with or-and if statement....
        '''
        # define starting x and y positions and slope
        x0 = self.x0
        y0 = self.y0
        slope = self.slope
        
        # add to list
        self.traj_list = []
        self.traj_list.append((int(x0), int(y0)))
        
        # define counter
        counter = 0
        
        # compute logic of ball bouncing off walls
        while True:
#            print('\n')
#            print(slope)
#            print(self.xdir)
            
            # check if beyond counter
            if counter > 10:
                self.pos_bumper = self.y0
                break
            
            # get top and bottom intersection
            xt, xb = self.__find_horiz_intersect(x0, y0, slope)
        
            # check if travelling beyond right side
            if xt > self.frame_w or xb > self.frame_w and self.xdir:
#                print('beyond right')
                
                # get vertical intersections and bumper position
                yl, yr = self.__find_vert_intersect(xt, 0, slope)
                self.pos_bumper = yr
                self.traj_list.append((int(self.frame_w), int(yr)))
                break
            
            # check if going right
            if self.xdir:
#                print('going right')
                
                # up and right, bumps top
                if xt > xb:
                    x0 = xt
                    y0 = 0
                # down and right, bumps bottom
                else:
                    x0 = xb
                    y0 = self.frame_h
                
                # flip slope and increment counter
                slope *= -1
                counter += 1
                self.traj_list.append((int(x0), int(y0)))
                continue
                
            # check if travelling beyond left side
            if xt < 0 or xb < 0 and not self.xdir:
#                print('beyond left')
                
                # get vertical intersections
                yl, yr = self.__find_vert_intersect(xt, 0, slope)
                
                # update point
                x0 = 0
                y0 = yl
                
                # flip slope, flip direction, and increment counter
                slope *= -1
                self.xdir = True
                counter += 1
                self.traj_list.append((int(x0), int(y0)))
                continue
            
            # check if going left
            if not self.xdir:
#                print('going left')
                
                # up and left, bumps top
                if xt < xb:
                    x0 = xt
                    y0 = 0
                # down and left, bumps bottom
                else:
                    x0 = xb
                    y0 = self.frame_h
                    
                # flip slope and increment counter
                slope *= -1
                counter += 1
                self.traj_list.append((int(x0), int(y0)))
                continue
                    
        return
    
    def __find_horiz_intersect(self, x0, y0, slope):
        '''

        '''
        # get intersection with top of frame: y=0
        xt = (0-y0)/slope+x0
        
        # get intersection with bottom of frame: y=frame_h
        xb = (self.frame_h-y0)/slope+x0
        
        return xt, xb
    
    def __find_vert_intersect(self, x0, y0, slope):
        '''

        '''
        # get intersection with left of frame: x=0
        yl = slope*(0-x0)+y0
        
        # get intesection with right of frame: x=frame_w
        yr = slope*(self.frame_w-x0)+y0
        
        return yl, yr
    
    def __plot_trajectory(self):
        '''

        '''
        if self.traj_list:
            # plot each line on binary image
            for i in range(len(self.traj_list)-1):
                cv2.line(self.bin_img, self.traj_list[i], self.traj_list[i+1], 100, 2)
                
            # mark previous point
            cv2.circle(self.bin_img, self.pos_prev, 10, 100, 1)
                
            # mark start point
            cv2.circle(self.bin_img, self.traj_list[0], 20, 100, 1)
            
            # mark final point
            cv2.circle(self.bin_img, self.traj_list[-1], 30, 100, 1)
        
        return
        
    
if __name__ == "__main__":
    bt = Ball_Trajectory(1, print_pos=True)
    print('max setpoint: ', bt.max_setpoint)
    print('counts offset: ', bt.counts_offset)
    
    
    try:
        bt.test()
        
    finally:
        # display image using cv2
        cv2.imshow('raw', bt.raw_img)
        cv2.imshow('bin', bt.bin_img)
        cv2.waitKey(0) # waits for any key press
        cv2.destroyAllWindows()
        
        # print trajectory
        print(bt.traj_list)
        
        # close ball trajectory
        bt.close()
