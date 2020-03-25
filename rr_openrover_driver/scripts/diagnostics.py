#!/usr/bin/env python3

import sys
import rclpy
from std_msgs.msg import String, Bool
from rr_openrover_driver_msgs.msg import RawRrOpenroverDriverSlowRateData, RawRrOpenroverDriverMedRateData,\
    SmartBatteryStatus


class RoverDiagnostic:

    def __init__(self, node):
        """
        @type node: rclpy.Node
        @param node: 
        """
        self.node = node
        self.node.get_logger().warn("InOrbit doesn't work with ROS2 yet!")
        self.pub = node.create_publisher(String, "/inorbit/custom_data/data0", 5)
        self.slow_data_sub = node.create_subscription(RawRrOpenroverDriverSlowRateData, "/raw_slow_rate_data", self.slow_data_cb)
        self.med_data_sub = node.create_subscription(RawRrOpenroverDriverMedRateData, "/rr_openrover_driver/raw_med_rate_data", self.med_data_cb)
        self.battery_a_sub = node.create_subscription(SmartBatteryStatus, "/rr_openrover_driver/battery_status_a", self.battery_status_a_cb)
        self.battery_b_sub = node.create_subscription(SmartBatteryStatus, "/rr_openrover_driver/battery_status_b", self.battery_status_b_cb)
        self.charging_sub = node.create_subscription(Bool, '/rr_openrover_driver/charging', self.openrover_charging_cb)

    def slow_data_cb(self, data):
        warn_msg = "Battery Levels [" + str(data.reg_robot_rel_soc_a) + ", " + str(
            data.reg_robot_rel_soc_b) + "]    Motor temps [" + str(data.reg_motor_temp_left) + ", " + str(
            data.reg_motor_temp_right) + "]"
        #        rospy.logwarn_throttle(60,warn_msg)

        self.pub.publish("Battery Cell 1 SOC=" + str(data.reg_robot_rel_soc_a))
        self.pub.publish("Battery Cell 2 SOC=" + str(data.reg_robot_rel_soc_b))
        self.pub.publish("Left Motor Temp=" + str(data.reg_motor_temp_left))
        self.pub.publish("Right Motor Temp=" + str(data.reg_motor_temp_right))
        self.pub.publish("Battery Mode A=" + str(data.battery_mode_a))
        self.pub.publish("Battery Mode B=" + str(data.battery_mode_b))
        self.pub.publish("Battery Temp A=" + str((data.battery_temp_a/10)-273))
        self.pub.publish("Battery Temp B=" + str((data.battery_temp_b/10)-273))
        self.pub.publish("Power Bat Voltage A=" + str(data.reg_power_bat_voltage_a/58.33))
        self.pub.publish("Power Bat Voltage B=" + str(data.reg_power_bat_voltage_b/58.33))
        self.pub.publish("Battery Voltage A=" + str(data.battery_voltage_a/1000))
        self.pub.publish("Battery Voltage B=" + str(data.battery_voltage_b/1000))

    def med_data_cb(self, data):
        self.pub.publish("Motor Feedback Current Left=" + str(data.reg_motor_fb_current_left))
        self.pub.publish("Motor Feedback Current Right=" + str(data.reg_motor_fb_current_right))
        self.pub.publish("Power A Current=" + str(data.reg_power_a_current))
        self.pub.publish("Power B Current=" + str(data.reg_power_b_current))
        self.pub.publish("Battery Current A=" + str(data.battery_current_a))
        self.pub.publish("Battery Current B=" + str(data.battery_current_b))

    def battery_status_a_cb(self, msg):
        for k in type(msg).__slots__:
            if k != 'header':
                self.pub.publish("Battery Status A." + k + '=' + str(int(getattr(msg, k))))

    def battery_status_b_cb(self, msg):
        for k in type(msg).__slots__:
            if k != 'header':
                self.pub.publish("Battery Status B." + k + '=' + str(int(getattr(msg, k))))

    def openrover_charging_cb(self, msg):
        self.pub.publish("Open Rover Charging=" + str(msg.data))


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("openrover_diagnostics_node")
    node.get_logger().info("Starting node")
    
    my_diagnostic = RoverDiagnostic(node)
    
    rclpy.spin(node)
