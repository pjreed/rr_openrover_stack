#!/usr/bin/env python3
from __future__ import division

# Author: Nick Fragale
# Description: This script converts Joystick commands into Joint Velocity commands
# Monitors A, B, X and Y buttons and toggles their state (False on startup) publishes 
# a latched Bool() to /joystick/<button> where button is A, B, Y, or X
# these can be remapped to different topics to control various things like E-stoping the robot
# or starting to record a bagfile, or taking a picture.

# Xbox controller mapping:
#   axes: [l-stick horz,l-stick vert, l-trigger, r-stick horz, r-stick vert, r-trigger]
#   buttons: [a,b,x,y,lb,rb,back,start,xbox,l-stick,r-stick,l-pad,r-pad,u-pad,d-pad]  

import time

import rclpy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import Float32, String
import sys


class XboxMapper:
    def __init__(self):
        self.node = rclpy.create_node('xbox_mapper_node')
        
        self.cmd = TwistStamped()
        self.last_a_button = time.time()
        self.last_b_button = time.time()
        self.last_x_button = time.time()
        self.last_y_button = time.time()
        self.last_joycb_device_check = time.time()
        self.button_msg = String()
        
        # Declare parameters
        self.node.declare_parameter('driver', 'xboxdrv')
        self.node.declare_parameter('wired_or_wireless', 'wireless')
        self.node.declare_parameter('max_vel_drive', 2.6)
        self.node.declare_parameter('max_vel_turn', 9.0)
        self.node.declare_parameter('max_vel_flipper', 1.4)
        self.node.declare_parameter('default_drive_throttle', 0.15)
        self.node.declare_parameter('default_flipper_throttle', 0.6)
        self.node.declare_parameter('adjustable_throttle', True)
        self.node.declare_parameter('a_button_toggle', False)
        self.node.declare_parameter('b_button_toggle', False)
        self.node.declare_parameter('x_button_toggle', False)
        self.node.declare_parameter('y_button_toggle', False)
        
        # difference between xboxdrv USB driver and xpad.ko kernel module
        # 1) xboxdrv has only 11 indices
        # 2) U/D_PAD_BUTTON is the 7th axis
        # 3) xpad module uses 2, xboxdrv uses 3
        self.driver = self.node.get_parameter('driver').value
        self.wired_or_wireless = self.node.get_parameter('wired_or_wireless').value
        
        if self.driver == 'xpad':
            self.node.get_logger().fatal('[{node_name}] xpad driver not supported.'.format(
                node_name=self.node.get_name()))
            rclpy.shutdown()
            exit(-1)
        
        elif self.wired_or_wireless == 'wired' and self.driver == 'xboxdrv':
            self.node.get_logger().info('XBOX CONFIG: wired & xboxdrv')
            self.node.get_logger().warn('If the wired controller becomes unplugged during operation '
                                        'xboxdrv may continue to publish the last command from the '
                                        'controller, causing the vehicle to run away.')
        
        elif self.wired_or_wireless == 'wireless' and self.driver == 'xboxdrv':
            self.node.get_logger().info('XBOX CONFIG: wireless & xboxdrv')
        
        else:
            self.node.get_logger().fatal('Unsupported controller configuration: {driver}, {connection}'.format(
                driver=self.driver, connection=self.wired_or_wireless))
            rclpy.shutdown()
            exit(-1)
        
        self.L_STICK_H_AXES = 0
        self.L_STICK_V_AXES = 1
        self.L_TRIG_AXES = 2
        self.R_STICK_H_AXES = 3
        self.R_STICK_V_AXES = 4
        self.R_TRIG_AXES = 4
        self.DPAD_H_AXES = 6
        self.DPAD_V_AXES = 7

        self.A_BUTTON = 0
        self.B_BUTTON = 1
        self.X_BUTTON = 2
        self.Y_BUTTON = 3
        self.LB_BUTTON = 4
        self.RB_BUTTON = 5
        self.BACK_BUTTON = 6
        self.START_BUTTON = 7
        self.POWER_BUTTON = 8
        self.L_STICK_BUTTON = 9
        self.R_STICK_BUTTON = 10

        self.prev_fwd = 0
        self.prev_trn = 0

        self.PREV_CMD_TIME = 0

        self.MAX_VEL_FWD = self.node.get_parameter('max_vel_drive').value
        self.MAX_VEL_TURN = self.node.get_parameter('max_vel_turn').value
        self.MAX_VEL_FLIPPER = self.node.get_parameter('max_vel_flipper').value
        self.DRIVE_THROTTLE = self.node.get_parameter('default_drive_throttle').value
        self.FLIPPER_THROTTLE = self.node.get_parameter('default_flipper_throttle').value
        self.ADJ_THROTTLE = self.node.get_parameter('adjustable_throttle').value
        self.A_BUTTON_TOGGLE = self.node.get_parameter('a_button_toggle').value
        self.B_BUTTON_TOGGLE = self.node.get_parameter('b_button_toggle').value
        self.X_BUTTON_TOGGLE = self.node.get_parameter('x_button_toggle').value
        self.Y_BUTTON_TOGGLE = self.node.get_parameter('y_button_toggle').value
        self.MIN_TOGGLE_DUR = 0.5  #
        self.DRIVE_INCREMENTS = float(20)
        self.FLIPPER_INCREMENTS = float(20)
        self.DEADBAND = 0.2
        self.FWD_ACC_LIM = 0.2
        self.TRN_ACC_LIM = 0.4
        self.DPAD_ACTIVE = False
        
        self.a_button_msg = Bool()
        self.a_button_msg.data = False
        self.b_button_msg = Bool()
        self.b_button_msg.data = False
        self.x_button_msg = Bool()
        self.x_button_msg.data = False
        self.y_button_msg = Bool()
        self.y_button_msg.data = False
        
        # define publishers
        self.pub = self.node.create_publisher(TwistStamped, '/cmd_vel/joystick', 1)
        
        latching_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.a_button_pub = self.node.create_publisher(Bool, '/joystick/a_button', latching_qos)
        self.b_button_pub = self.node.create_publisher(Bool, '/joystick/b_button', latching_qos)
        self.x_button_pub = self.node.create_publisher(Bool, '/joystick/x_button', latching_qos)
        self.y_button_pub = self.node.create_publisher(Bool, '/joystick/y_button', latching_qos)
        self.pub_delay = self.node.create_publisher(Float32, '/joystick/delay', 3)
        self.pub_cancel_move_base = self.node.create_publisher(GoalID, '/move_base/cancel', 10)

        self.sub_cmds = self.node.create_subscription(Joy, 'joystick', self.joy_cb, 10)

    def run(self):
        self.a_button_pub.publish(self.a_button_msg)
        self.b_button_pub.publish(self.b_button_msg)
        self.x_button_pub.publish(self.x_button_msg)
        self.y_button_pub.publish(self.y_button_msg)

        rclpy.spin(self.node)

    def limit_acc(self, fwd, trn):
        fwd_acc = fwd - self.prev_fwd
        if fwd_acc > self.FWD_ACC_LIM:
            fwd = self.prev_fwd + self.FWD_ACC_LIM
        elif fwd_acc < -self.FWD_ACC_LIM:
            fwd = self.prev_fwd - self.FWD_ACC_LIM

        trn_acc = trn - self.prev_trn
        if trn_acc > self.TRN_ACC_LIM:
            trn = self.prev_trn + self.TRN_ACC_LIM
        elif trn_acc < -self.TRN_ACC_LIM:
            trn = self.prev_trn - self.TRN_ACC_LIM

        self.prev_fwd = fwd
        self.prev_trn = trn

        return fwd, trn
    
    def joy_cb(self, joy_msg):
        """
        @type joy_msg: Joy
        @param joy_msg: Incoming joystick command
        """
        cmd_time = float(joy_msg.header.stamp.sec) + (float(joy_msg.header.stamp.nanosec) / 1000000000)
        rbt_time = time.time()
        signal_delay = rbt_time - cmd_time
    
        joy_delay = Float32()
        joy_delay.data = signal_delay
        self.pub_delay.publish(joy_delay)
    
        # Record timestamp and seq for use in next loop
        self.PREV_CMD_TIME = cmd_time

        # check for other two user-defined buttons. We only debounce them and monitor on/off status on a latched pub
        # (green/A)
        if self.A_BUTTON_TOGGLE:
            if joy_msg.buttons[self.A_BUTTON] == 1:
                if time.time() - self.last_a_button > self.MIN_TOGGLE_DUR:
                    self.last_a_button = time.time()
                    a_button_state = not self.a_button_msg.data
                    self.node.get_logger().info('A button toggled: {state}'.format(state=a_button_state))
                    self.a_button_msg.data = a_button_state
        else:
            if joy_msg.buttons[self.A_BUTTON] == 1:
                self.a_button_msg.data = True
            else:
                self.a_button_msg.data = False
        self.a_button_pub.publish(self.a_button_msg)
    
        # (red/B)
        if self.B_BUTTON_TOGGLE:
            if joy_msg.buttons[self.B_BUTTON] == 1:
                if time.time() - self.last_b_button > self.MIN_TOGGLE_DUR:
                    self.last_b_button = time.time()
                    b_button_state = not self.b_button_msg.data
                    self.node.get_logger().info('B button toggled: {state}'.format(state=b_button_state))
                    self.b_button_msg.data = b_button_state
        else:
            if joy_msg.buttons[self.B_BUTTON] == 1:
                self.b_button_msg.data = True
            else:
                self.b_button_msg.data = False
        self.b_button_pub.publish(self.b_button_msg)
    
        # (blue/X)
        if self.X_BUTTON_TOGGLE:
            if joy_msg.buttons[self.X_BUTTON] == 1:
                if time.time() - self.last_x_button > self.MIN_TOGGLE_DUR:
                    self.last_x_button = time.time()
                    x_button_state = not self.x_button_msg.data
                    self.node.get_logger().info('X button toggled: {state}'.format(state=x_button_state))
                    self.x_button_msg.data = x_button_state
        else:
            if joy_msg.buttons[self.X_BUTTON] == 1:
                self.x_button_msg.data = True
            else:
                self.x_button_msg.data = False
        self.x_button_pub.publish(self.x_button_msg)
    
        # (yellow/Y)
        if self.Y_BUTTON_TOGGLE:
            if joy_msg.buttons[self.Y_BUTTON] == 1:
                if time.time() - self.last_y_button > self.MIN_TOGGLE_DUR:
                    self.last_y_button = time.time()
                    y_button_state = not self.y_button_msg.data
                    self.node.get_logger().info('Y button toggled: {state}'.format(state=y_button_state))
                    self.y_button_msg.data = y_button_state
        else:
            if joy_msg.buttons[self.Y_BUTTON] == 1:
                self.y_button_msg.data = True
            else:
                self.y_button_msg.data = False
        self.y_button_pub.publish(self.y_button_msg)
    
        if self.ADJ_THROTTLE:
            # Increase/Decrease Max Speed
            if self.driver == 'xpad':
                if int(joy_msg.axes[self.DPAD_V_AXES]) == 1 and not self.DPAD_ACTIVE:
                    self.DRIVE_THROTTLE += (1. / self.DRIVE_INCREMENTS)
                    self.DPAD_ACTIVE = True
                if int(joy_msg.axes[self.DPAD_V_AXES]) == -1 and not self.DPAD_ACTIVE:
                    self.DRIVE_THROTTLE -= (1. / self.DRIVE_INCREMENTS)
                    self.DPAD_ACTIVE = True
            elif self.driver == 'xboxdrv':
                if int(joy_msg.axes[self.DPAD_V_AXES]) == 1 and not self.DPAD_ACTIVE:
                    self.DRIVE_THROTTLE += (1. / self.DRIVE_INCREMENTS)
                    self.DPAD_ACTIVE = True
                if int(joy_msg.axes[self.DPAD_V_AXES]) == -1 and not self.DPAD_ACTIVE:
                    self.DRIVE_THROTTLE -= (1. / self.DRIVE_INCREMENTS)
                    self.DPAD_ACTIVE = True
    
            if joy_msg.buttons[self.LB_BUTTON] == 1:
                self.FLIPPER_THROTTLE -= (1. / self.FLIPPER_INCREMENTS)
                self.node.get_logger().info("%f" % self.FLIPPER_THROTTLE)
            if joy_msg.buttons[self.RB_BUTTON] == 1:
                self.FLIPPER_THROTTLE += (1. / self.FLIPPER_INCREMENTS)
                self.node.get_logger().info("%f" % self.FLIPPER_THROTTLE)
    
            # If the user tries to decrease full throttle to 0
            # Then set it back up to 0.2 m/s
            if self.DRIVE_THROTTLE <= 0.001:
                self.DRIVE_THROTTLE = (1. / self.DRIVE_INCREMENTS)
            if self.FLIPPER_THROTTLE <= 0.001:
                self.FLIPPER_THROTTLE = (1. / self.FLIPPER_INCREMENTS)
    
            # If the user tries to increase the velocity limit when its at max
            # then set velocity limit to max allowed velocity
            if self.DRIVE_THROTTLE >= 1:
                self.DRIVE_THROTTLE = 1
            if self.FLIPPER_THROTTLE >= 1:
                self.FLIPPER_THROTTLE = 1
    
            # Update DEADBAND
            fwd_deadband = 0.2 * self.DRIVE_THROTTLE * self.MAX_VEL_FWD
            turn_deadband = 0.2 * self.DRIVE_THROTTLE * self.MAX_VEL_TURN
            flipper_deadband = 0.2 * self.FLIPPER_THROTTLE * self.MAX_VEL_FLIPPER
    
            if self.DPAD_ACTIVE:
                self.node.get_logger().info('Drive Throttle: %f' % self.DRIVE_THROTTLE)
    
            if (joy_msg.axes[self.DPAD_V_AXES], joy_msg.axes[self.DPAD_H_AXES]) == (0, 0):
                self.DPAD_ACTIVE = False
    
        # Drive Forward/Backward commands
        drive_cmd = self.DRIVE_THROTTLE * self.MAX_VEL_FWD * joy_msg.axes[self.L_STICK_V_AXES]  # left joystick
        if fwd_deadband > drive_cmd > -fwd_deadband:
            drive_cmd = 0
    
            # Turn left/right commands
        turn_cmd = (1.1 - (drive_cmd / self.MAX_VEL_FWD)) * self.DRIVE_THROTTLE * self.MAX_VEL_TURN * joy_msg.axes[
            self.R_STICK_H_AXES]  # right joystick
        if turn_deadband > turn_cmd > -turn_deadband:
            turn_cmd = 0
    
        # Flipper up/down commands
        flipper_cmd = (self.FLIPPER_THROTTLE * self.MAX_VEL_FLIPPER * joy_msg.axes[self.L_TRIG_AXES]) - (
                self.FLIPPER_THROTTLE * self.MAX_VEL_FLIPPER * joy_msg.axes[self.R_TRIG_AXES])
        if flipper_deadband > flipper_cmd > -flipper_deadband:
            flipper_cmd = 0
    
        # Limit acceleration
        # drive_cmd, turn_cmd = limit_acc(drive_cmd, turn_cmd)
    
        # update the last time joy_cb was called
        if (drive_cmd != 0) or (turn_cmd != 0):
            self.last_joycb_device_check = time.time()
    
        # Publish move commands
        self.cmd.header.stamp = self.node.get_clock().now().to_msg()
        self.cmd.twist.linear.x = float(drive_cmd)
        self.cmd.twist.angular.y = float(flipper_cmd)
        self.cmd.twist.angular.z = float(turn_cmd)
        self.pub.publish(self.cmd)


# Main Function
def joystick_main():
    rclpy.init(args=sys.argv)
    mapper = XboxMapper()
    mapper.run()


if __name__ == '__main__':
    joystick_main()
