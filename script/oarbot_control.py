#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from oarbot_control.msg import Twist2D, MotorStatus, MotorCmd
from PyRoboteq import RoboteqHandler
from PyRoboteq import roboteq_commands as cmds

class OarbotKinematics():
    def__init__(self):
        # robot kinematic parameters in m
        self.r = 101.6/1000 # wheel radius
        self.l = 214.63/1000 # half length of chassis
        self.w = 220.08/1000 # half width of the chassis
        self.ratio = 15/21 # gear ratio of chain drive

class OarbotControl():
    def __init__(self):
        rospy.init_node('oarbot_ctrl', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        self.vel_feedback = Twist2D() # velocity feedback
        self.motor_cmd = MotorCmd() # motor velocity command
        # actual velocity publisher
        self.vel_pub = rospy.Publisher('vel_feedback', Twist2D, queue_size=1)
        self.motor_cmd_pub = rospy.Publisher('motor_command', MotorCmd, queue_size=1)
        rospy.Subscriber("cmd_vel", Twist, self.callback)
        # connection to Roboteq robot controller
        self.controller_f = RoboteqHandler(debug_mode=False, exit_on_interrupt=False) 
        self.controller_b = RoboteqHandler(debug_mode=False, exit_on_interrupt=False) 
        self.connected_f = controller_f.connect("/dev/ttyACM0")
        self.connected_b = controller_b.connect("/dev/ttyACM1")

        self.oarbot = OarbotKinematics()

    def callback(self, msg):
        # do the motor control here
        self.inverse_kin(msg)

    def inverse_kin(self, msg):
        if self.connected_f and self.connected_b:
            v_lin = msg.linear
            v_ang = msg.angular

            # angular velocity of front left motor
            u1 = 1/self.oarbot.r * (v_lin.x - v_lin.y - (self.oarbot.l+self.oarbot.w)*v_ang.z) * 1/self.oarbot.ratio
            # angular velocity of front right motor
            u2 = 1/self.oarbot.r * (v_lin.x + v_lin.y + (self.oarbot.l+self.oarbot.w)*v_ang.z) * 1/self.oarbot.ratio
            # angular velocity of rear right motor
            u3 = 1/self.oarbot.r * (v_lin.x - v_lin.y + (self.oarbot.l+self.oarbot.w)*v_ang.z) * 1/self.oarbot.ratio
            # angular velocity of rear left motor
            u4 = 1/self.oarbot.r * (v_lin.x + v_lin.y - (self.oarbot.l+self.oarbot.w)*v_ang.z) * 1/self.oarbot.ratio

            # TODO: add emergency stop if v too large
            if abs(u1) > 1000 or abs(u2) > 1000 or abs(u3) > 1000 or abs(u4) > 1000:
                u1 = u2 = u3 = u4 = 0
            
            #TODO: check units
            self.motor_cmd.v_fl = u1
            self.motor_cmd.v_fr = u2
            self.motor_cmd.v_rl = u4
            self.motor_cmd.v_rr = u3

            self.motor_cmd_pub.publish(self.motor_cmd)

            # front motors velocity control
            # TODO: check if cmds must be integer
            controller_f.send_command(cmds.DUAL_DRIVE, u1, u2)
            # rear motors velocity control
            controller_b.send_command(cmds.DUAL_DRIVE, u4, u3)
            
    # using forward kinematics to compute actual v
    def forward_kin(self):
        #TODO: read actual velocity from roboteq
        u1a = u1
        u2a = u2
        u3a = u3
        u4a = u4

        self.vel_feedback.vx = self.oarbot.r/4 * (u1a + u2a + u3a + u4a) * self.oarbot.ratio
        self.vel_feedback.vy = self.oarbot.r/4 * (-u1a + u2a - u3a + u4a) * self.oarbot.ratio
        self.vel_feedback.wz = self.oarbot.r/(4*(self.oarbot.l + self.oarbot.w)) * (-u1a + u2a + u3a - u4a) * self.oarbot.ratio
        self.vel_pub.publish(self.vel_feedback)

    #TODO: add motor status feedback

    def run(self):
        while not rospy.is_shutdown():
            if self.connected_f and self.connected_b:
                self.forward_kin()
                self.rate.sleep()



if __name__ == "__main__":
    oarbot = OarbotControl()
    oarbot.run()