#!/usr/bin/env python3
# coding: utf-8
# SPDX-FileCopyrightText: KAWAHARA Shuji (2022)
# SPDX-License-Identifier: MIT License

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

def cb(msg):
    global node
    node.get_logger().info("Listen: %d" % msg.data)

rclpy.init()
node = Node("listener")
pub = node.create_subscription(Int16, "countup", cb, 10)
rclpy.spin(node)

