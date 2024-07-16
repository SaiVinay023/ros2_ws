#!/usr/bin/env python

import json
import yaml
import websocket
import sys
import rclpy
from rclpy.node import Node
from threading import Lock
from rosmonitoring_interfaces.msg import MonitorError
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import rosidl_runtime_py

class ROSMonitor_monitor_0(Node):

    def callback_input_scan(self, data):
        self.get_logger().info("monitor has observed " + str(data))
        dict_msg = rosidl_runtime_py.convert_message_to_ordereddict(data)
        dict_msg['topic'] = '/input_scan'
        dict_msg['time'] = float(self.get_clock().now().to_msg().sec)
        self.ws_lock.acquire()
        self.logging(dict_msg)
        self.ws_lock.release()
        self.get_logger().info("event successfully logged")

    def callback_output_vel(self, data):
        self.get_logger().info("monitor has observed " + str(data))
        dict_msg = rosidl_runtime_py.convert_message_to_ordereddict(data)
        dict_msg['topic'] = '/output_vel'
        dict_msg['time'] = float(self.get_clock().now().to_msg().sec)
        self.ws_lock.acquire()
        self.logging(dict_msg)
        self.ws_lock.release()
        self.get_logger().info("event successfully logged")

    def __init__(self, monitor_name, log, actions):
        super().__init__(monitor_name)
        self.monitor_publishers = {}
        self.config_publishers = {}
        self.config_subscribers = {}
        self.config_client_services = {}
        self.config_server_services = {}
        self.services_info = {}
        self.dict_msgs = {}
        self.ws_lock = Lock()
        self.name = monitor_name
        self.actions = actions
        self.logfn = log
        self.topics_info = {}

        # creating the verdict and error publishers for the monitor
        self.monitor_publishers['error'] = self.create_publisher(MonitorError, self.name + '/monitor_error', 10)
        self.monitor_publishers['verdict'] = self.create_publisher(String, self.name + '/monitor_verdict', 10)

        # done creating monitor publishers
        self.publish_topics = False
        self.topics_info['/input_scan'] = {'package': 'sensor_msgs/msg', 'type': 'sensor_msgs/msg/LaserScan'}
        self.topics_info['/output_vel'] = {'package': 'geometry_msgs/msg', 'type': 'geometry_msgs/msg/Twist'}

        self.config_subscribers['/input_scan'] = self.create_subscription(LaserScan, '/input_scan', self.callback_input_scan, 10)
        self.config_subscribers['/output_vel'] = self.create_subscription(Twist, '/output_vel', self.callback_output_vel, 10)

        self.get_logger().info('Monitor ' + self.name + ' started and ready')
        self.get_logger().info('Logging at ' + self.logfn)

    def logging(self, json_dict):
        try:
            with open(self.logfn, 'a+') as log_file:
                log_file.write(json.dumps(json_dict) + '\n')
            self.get_logger().info('Event logged')
        except Exception as e:
            self.get_logger().info('Unable to log the event: ' + str(e))

def main(args=None):
    rclpy.init(args=args)
    log = './log.txt'
    actions = {}
    actions['/input_scan'] = ('log', 0)
    actions['/output_vel'] = ('log', 0)
    monitor = ROSMonitor_monitor_0('monitor_0', log, actions)
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
