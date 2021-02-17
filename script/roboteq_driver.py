#!/usr/bin/env python
import rospy
from PyRoboteq import RoboteqHandler
from PyRoboteq import roboteq_commands as cmds

if __name__ == "__main__":
    controller = RoboteqHandler(debug_mode=False, exit_on_interrupt=False) 
	connected = controller.connect("/dev/ttyACM0") # Insert your COM port (for windows) or /dev/tty{your_port} (Commonly /dev/ttyACM0) for linux.

    while connected:
        
        if keyboard.is_pressed('s'):
            print("S pressed")
            drive_speed_motor_one = 200
            drive_speed_motor_two = 200
            
        elif keyboard.is_pressed('d'):
            print("D pressed")
            drive_speed_motor_one = 200
            drive_speed_motor_two = 200

        elif keyboard.is_pressed('x'):
            print("X is pressed")
            drive_speed_motor_one = -200
            drive_speed_motor_two = 200

        elif keyboard.is_pressed('c'):
            print("C is pressed")
            drive_speed_motor_one = 200
            drive_speed_motor_two = -200

        # Motor will automatically stop if no command is sent.
        else:
            drive_speed_motor_one = 0
            drive_speed_motor_two = 0

        controller.send_command(cmds.DUAL_DRIVE, drive_speed_motor_one, drive_speed_motor_two)
        
            
            

        
        

