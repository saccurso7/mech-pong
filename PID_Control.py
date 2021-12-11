#!/usr/bin/python3

import board
import busio
import adafruit_mcp4725
import RPi.GPIO as GPIO
from LS7366R import LS7366R
from time import sleep

class PID_Controller:
    '''

    '''
    
    ### CLASS VARIABLES ###
    
    # max and min integer input to DAC
    dac_val_min = 19500
    dac_val_max = 49500-27000
    
    # max and min PID input
    u_min = -(dac_val_max-dac_val_min)
    u_max = dac_val_max-dac_val_min
    
    # max and min setpoints
    setpoint_min = 0
    setpoint_max = 4050
    
    ### PUBLIC METHODS ###
    
    def __init__(self, setpoint_q, Kp=2.8, Ki=0.18, Kd=0.4, dt=0.02):
        '''

        '''
        # create variable for setpoint queue
        self.setpoint_q = setpoint_q
        
        # create variables for CT gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        # create variables for DT gains and sampling time
        self.K1 = Kp+Ki+Kd
        self.K2 = -Kp-2*Kd
        self.K3 = Kd
        self.dt = dt
        
        # create variables for errors
        self.e1 = 0.0
        self.e2 = 0.0
        self.e3 = 0.0
        
        # create variables for position and setpoint
        self.pos = 0.0
        self.setpoint = 0.0
        
        # create variable for input
        self.u = 0.0
        
        # start up peripherals
        self.__start_peripherals()
        
        # run homing procedure
        try:
            self.__home()
        except:
            self.dac.value = 0
        
        
        return
        
    def run(self): 
        '''
        
        '''
        while True:
            # update setpoint
            self.__get_setpoint()
            print('\nSetpoint: '+str(self.setpoint))
            
            # compute PID input
            self.__get_input()
            
            # send PID input
            self.__send_input()
            
            # sleep for sampling time
            sleep(self.dt)
        
        return
    
    def close(self):
        '''

        '''
        print('Closing PID controller.')
        
        # set DAC value to zero
        self.dac.value = int(0)
        
        # close encoder
        print('Final encoder count: ' + str(self.enc.readCounter()))
        self.enc.close()
        
        # clean up GPIO
        GPIO.cleanup()
        
        return
    
    ### PRIVATE METHODS ###
        
    def __get_setpoint(self):
        '''

        '''
        # get next setpoint in queue if not empty
        if not self.setpoint_q.empty():
            self.setpoint = self.setpoint_q.get()
        
        # bound setpoints
        if self.setpoint > self.setpoint_max:
            self.setpoint = self.setpoint_max
        if self.setpoint < self.setpoint_min:
            self.setpoint = self.setpoint_min
        
        return
    
    def __get_input(self):
        '''

        '''
        # update past errors
        self.e3 = self.e2
        self.e2 = self.e1
        
        # measure new position
        self.pos = self.enc.readCounter()
        print('Position: '+str(self.pos))
        
        # compute current error
        self.e1 = self.setpoint-self.pos
        
        # compute DT PID input
        self.u += self.K1*self.e1+self.K2*self.e2+self.K3*self.e3
        
        # clip to max or min
        if self.u > self.u_max:
            self.u = self.u_max
        if self.u < self.u_min:
            self.u = self.u_min
        
        return
    
    def __send_input(self):
        '''

        '''
        # set direction and compute dac value
        if self.u >= 0: #CCW
            GPIO.output(self.dir_pin, GPIO.LOW)
            self.dac_val = self.dac_val_min+self.u
        else: #CW
            GPIO.output(self.dir_pin, GPIO.HIGH)
            self.dac_val = self.dac_val_min+abs(self.u)
        
        # set DAC value
        self.dac.value = int(self.dac_val)
        
        return
    
    def __start_peripherals(self):
        '''

        '''
        # set up DAC for MC and set input to zero
        i2c = busio.I2C(board.SCL, board.SDA)
        self.dac = adafruit_mcp4725.MCP4725(i2c)
        self.dac_val = 0.0
        self.dac.value = int(self.dac_val)
        
        # set up GPIO for direction, brake, and switch control
        self.dir_pin = 16
        self.brake_pin = 20
        self.far_pin = 23
        self.home_pin = 24
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dir_pin, GPIO.OUT)
        GPIO.setup(self.brake_pin, GPIO.OUT)
        
        GPIO.setup(self.far_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.home_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # add interrupts
        GPIO.add_event_detect(self.far_pin, GPIO.FALLING,
                              callback=self.__far_callback, bouncetime=300)
        GPIO.add_event_detect(self.home_pin, GPIO.FALLING,
                              callback=self.__home_callback, bouncetime=300)
        
        # turn off brake and set direction CCW
        GPIO.output(self.brake_pin, GPIO.HIGH)
        GPIO.output(self.dir_pin, GPIO.LOW)
        
        # set up encoder counter
        self.enc = LS7366R(0, 10000000, 4)
        
        return
    
    def __home(self):
        '''
        move bumper slowly to home position, clear encoder counts
        '''
        # move bumper very slowly to home position
        GPIO.output(self.dir_pin, GPIO.HIGH)
        
        self.homing = True
        print('\nHoming...')
        while self.homing:
            self.dac.value = self.dac_val_min+1080
        
        # homing interrupt sets input to zero
        self.__send_input()
        
        # clear encoder counts
        self.enc.clearCounter()
        self.enc.clearStatus()
        
        print('\nHoming finished')
        
        return
    
    def __home_callback(self, channel):
        '''

        '''
        # wait for signal to stabilize
        sleep(.03)
        
        # check if signal is low and set motor input to zero
        if not GPIO.input(self.home_pin):
            self.u = 0
            
            # if homing, set homing to false
            if self.homing:
                self.homing = False
#             # clear encoder counts otherwise
#             else:
#                 # clear encoder counts
#                 self.enc.clearCounter()
#                 self.enc.clearStatus()
        
        return
    
    def __far_callback(self, channel):
        '''

        '''
        # wait for signal to stabilize
        sleep(.03)
        
        # check if signal is low and set motor input to zero
        if not GPIO.input(self.far_pin):
            self.u = 0
        
        return
    
if __name__  == '__main__':
    pid = PID_Controller(1)
    
    sleep(1)
    print(pid.enc.readCounter())