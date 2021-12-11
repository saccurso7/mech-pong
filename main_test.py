#!/usr/bin/python3

from time import sleep
from PID_Control import PID_Controller
from Ball_Trajectory import Ball_Trajectory
from multiprocessing import Process, Queue

if __name__ == '__main__':
    try:
        # create setpoint queue
        setpoint_q = Queue()
        
        # create PID controller object
        pid = PID_Controller(setpoint_q)
        
        # create ball trajectory object
        bt = Ball_Trajectory(setpoint_q)
        
        # create processes
        pid_process = Process(target=pid.run)
        bt_process = Process(target=bt.run)
        
        # start and join
        pid_process.start()
        bt_process.start()
        
        pid_process.join()
        bt_process.join()
        
    except KeyboardInterrupt:
        # terminate processes
        pid_process.terminate()
        bt_process.terminate()
        
        # close PID controller
        pid.close()
        

