#!/usr/bin/env python3

import rclpy
import rclpy.time
from geometry_msgs.msg import TwistStamped, Twist
import sys
from yaml import safe_load
HALT = Twist()


class CommandHandle(object):
    """Manages timeouts and relaying incoming messages for a given control input"""
    # Class variables
    prev_cmd = None

    def __init__(self, node, sub_topic, pub_topic, timeout, stamped, sub_is_stamped=True, publish_redundant_halts=True,
                 frame_id=''):
        """

        @type node: rclpy.Node
        @param node:
        @type sub_topic: str
        @param sub_topic:
        @type pub_topic: str
        @param pub_topic:
        @param timeout:
        @param stamped:
        @type sub_is_stamped: bool
        @param sub_is_stamped:
        @type publish_redundant_halts: bool
        @param publish_redundant_halts:
        @type frame_id: str
        @param frame_id:
        """
        self.node = node
        self.sub_topic = sub_topic
        self.pub_topic = pub_topic
        self.timeout = float(timeout)
        self.is_stamped = stamped
        self.sub_is_stamped = sub_is_stamped
        self.publish_redundant_halts = publish_redundant_halts
        self.frame_id = frame_id
        self._pub_type = TwistStamped if stamped else Twist
        self._sub_type = TwistStamped if sub_is_stamped else Twist

        try:
            self.timeout = float(timeout)
        except ValueError:
            raise ValueError('Invalid `timeout` value for control_input: {TIMEOUT}'
                             .format(TIMEOUT=timeout))

        if not isinstance(stamped, bool):
            raise ValueError('Invalid `stamped` value for control_input: {STAMPED}'
                             .format(STAMPED=stamped))

        self.pub = self.node.create_publisher(self._pub_type, self.pub_topic, 1)
        self.sub = self.node.create_subscription(self._sub_type, self.sub_topic, self.callback, 5)

        self.node.get_logger().info(
            'Configuring Command Input Handler\n' +
            'Subscribed Topic: {TOPIC}\n'.format(TOPIC=self.sub_topic) +
            'Published Topic: {TOPIC}\n'.format(TOPIC=self.pub_topic) +
            'Timeout: {TIMEOUT}\n'.format(TIMEOUT=self.timeout) +
            'Input Topic Stamped: {STAMPED}\n'.format(STAMPED=self.sub_is_stamped) +
            'Output Topic Stamped: {STAMPED}\n'.format(STAMPED=self.is_stamped) +
            'Publishing Redundant Halt Commands: {HALT}\n'.format(HALT=self.publish_redundant_halts)
        )

    def callback(self, data):
        """Passes through message and strip strips header if necessary"""
        if self.sub_is_stamped:
            twist_cmd = data.twist
        else:
            twist_cmd = data

        if not self.publish_redundant_halts and (self.prev_cmd == HALT and twist_cmd == HALT):
            return

        self.prev_cmd = twist_cmd

        if type(data) == TwistStamped and self.expired(data):
            return

        self.pub.publish(self.format_cmd_data(data))

    def expired(self, data):
        """Tests message stamp to see if it has timeout"""
        now = self.node.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(data.header.stamp, self.node.get_clock().clock_type)
        return 0.0 < self.timeout <= (now - msg_time).nanoseconds / 1e9

    def format_cmd_data(self, data):
        if (self._sub_type == TwistStamped and self._pub_type == TwistStamped) or \
                (self._sub_type == Twist and self._pub_type == Twist):
            return data
        elif self._sub_type == TwistStamped and self._pub_type == Twist:
            return data.twist
        else:
            data_stamped = TwistStamped()
            data_stamped.header.frame_id = self.frame_id
            data_stamped.header.stamp = self.node.get_clock().now()
            data_stamped.twist = data
            return data_stamped


class ControlInputManager:
    """Creates a handle for each input.  Drop inputs if timeout is not a number or
    stamped is not a boolean.
    """

    def __init__(self, node, control_inputs):
        self.input_handles = []
        self.node = node
        for idx, control_input in enumerate(control_inputs):
            try:
                command_handle = CommandHandle(node, **control_input)
                self.input_handles.append(command_handle)
            except ValueError as e:
                self.node.get_logger().fatal('control_input manager ({IDX}) was not created. {params}'
                                             .format(IDX=idx, params=control_input))
                self.node.get_logger().fatal(e)

    def run(self):
        rclpy.spin(self.node)


def check_params(control_inputs, node):
    required_parameters = {'pub_topic', 'sub_topic', 'timeout', 'stamped'}
    optional_parameters = {'sub_is_stamped', 'publish_redundant_halts'}
    for control_input in control_inputs:
        params = set(control_input.keys())
        missing_params = required_parameters - params
        unrecognized_params = params - required_parameters - optional_parameters
        if len(missing_params) != 0:
            err_msg = "Missing the parameters: " + ', '.join(list(missing_params))
            raise ValueError(err_msg)
        elif len(unrecognized_params) != 0:
            node.get_logger().error('Unrecognized parameters: ' + ', '.join(list(unrecognized_params)))


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('control_input_manager')
    node.declare_parameter('control_inputs_file')

    control_inputs_file = node.get_parameter('control_inputs_file').value
    with open(control_inputs_file) as infile:
        control_inputs = safe_load(infile)['control_inputs']

    check_params(control_inputs, node)

    cim = ControlInputManager(node, control_inputs)
    cim.run()


if __name__ == '__main__':
    main()
