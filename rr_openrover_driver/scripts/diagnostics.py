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
        self.pub = node.create_publisher(String, "/inorbit/custom_data/data0", qos_profile=5)
        self.slow_data_sub = node.create_subscription(RawRrOpenroverDriverSlowRateData,
                                                      "raw_slow_rate_data",
                                                      self.slow_data_cb)
        self.med_data_sub = node.create_subscription(RawRrOpenroverDriverMedRateData,
                                                     "raw_med_rate_data",
                                                     self.med_data_cb)
        self.battery_a_sub = node.create_subscription(SmartBatteryStatus,
                                                      "battery_status_a",
                                                      self.battery_status_a_cb)
        self.battery_b_sub = node.create_subscription(SmartBatteryStatus,
                                                      "battery_status_b",
                                                      self.battery_status_b_cb)
        self.charging_sub = node.create_subscription(Bool, 'charging', self.openrover_charging_cb)

    @staticmethod
    def to_msg(data):
        """
        Converts a Python str into a ROS std_msgs/msg/String
        @type data: str
        :param data:
        :return String:
        """
        msg = String()
        msg.data = data
        return msg

    def slow_data_cb(self, data):
        """
        @type data: RawRrOpenroverDriverSlowRateData
        @param data:
        """
        warn_msg = "Battery Levels [" + str(data.reg_robot_rel_soc_a) + ", " + str(
            data.reg_robot_rel_soc_b) + "]    Motor temps [" + str(data.reg_motor_temp_left) + ", " + str(
            data.reg_motor_temp_right) + "]"
        #        rospy.logwarn_throttle(60,warn_msg)

        self.pub.publish(RoverDiagnostic.to_msg("Battery Cell 1 SOC=" + str(data.reg_robot_rel_soc_a)))
        self.pub.publish(RoverDiagnostic.to_msg("Battery Cell 2 SOC=" + str(data.reg_robot_rel_soc_b)))
        self.pub.publish(RoverDiagnostic.to_msg("Left Motor Temp=" + str(data.reg_motor_temp_left)))
        self.pub.publish(RoverDiagnostic.to_msg("Right Motor Temp=" + str(data.reg_motor_temp_right)))
        self.pub.publish(RoverDiagnostic.to_msg("Battery Mode A=" + str(data.battery_mode_a)))
        self.pub.publish(RoverDiagnostic.to_msg("Battery Mode B=" + str(data.battery_mode_b)))
        self.pub.publish(RoverDiagnostic.to_msg("Battery Temp A=" + str((data.battery_temp_a/10)-273)))
        self.pub.publish(RoverDiagnostic.to_msg("Battery Temp B=" + str((data.battery_temp_b/10)-273)))
        self.pub.publish(RoverDiagnostic.to_msg("Power Bat Voltage A=" + str(data.reg_power_bat_voltage_a/58.33)))
        self.pub.publish(RoverDiagnostic.to_msg("Power Bat Voltage B=" + str(data.reg_power_bat_voltage_b/58.33)))
        self.pub.publish(RoverDiagnostic.to_msg("Battery Voltage A=" + str(data.battery_voltage_a/1000)))
        self.pub.publish(RoverDiagnostic.to_msg("Battery Voltage B=" + str(data.battery_voltage_b/1000)))

    def med_data_cb(self, data):
        """
        @type data: RawRrOpenroverDriverMedRateData
        @param data:
        """
        self.pub.publish(RoverDiagnostic.to_msg("Motor Feedback Current Left=" + str(data.reg_motor_fb_current_left)))
        self.pub.publish(RoverDiagnostic.to_msg("Motor Feedback Current Right=" + str(data.reg_motor_fb_current_right)))
        self.pub.publish(RoverDiagnostic.to_msg("Power A Current=" + str(data.reg_power_a_current)))
        self.pub.publish(RoverDiagnostic.to_msg("Power B Current=" + str(data.reg_power_b_current)))
        self.pub.publish(RoverDiagnostic.to_msg("Battery Current A=" + str(data.battery_current_a)))
        self.pub.publish(RoverDiagnostic.to_msg("Battery Current B=" + str(data.battery_current_b)))

    def battery_status_a_cb(self, msg):
        """
        @type msg: SmartBatteryStatus
        @param msg:
        """
        for k in type(msg).__slots__:
            if k != 'header':
                self.pub.publish(RoverDiagnostic.to_msg("Battery Status A.%s=%d" % (k, int(getattr(msg, k) is True))))

    def battery_status_b_cb(self, msg):
        """
        @type msg: SmartBatteryStatus
        @param msg:
        """
        for k in type(msg).__slots__:
            if k != 'header':
                self.pub.publish(RoverDiagnostic.to_msg("Battery Status B.%s=%d" % (k, int(getattr(msg, k) is True))))

    def openrover_charging_cb(self, msg):
        """
        @type msg: Bool
        @param msg:
        """
        self.pub.publish(RoverDiagnostic.to_msg("Open Rover Charging=" + str(msg.data)))


if __name__ == '__main__':
    rclpy.init(args=sys.argv)
    node = rclpy.create_node("openrover_diagnostics_node")
    node.get_logger().info("Starting node")
    
    my_diagnostic = RoverDiagnostic(node)
    
    rclpy.spin(node)
