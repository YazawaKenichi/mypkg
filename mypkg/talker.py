#!/usr/bin/env python3
# coding: utf-8
# SPDX-FileCopyrightText: KAWAHARA Shuji (2022)
# SPDX-License-Identifier: MIT License

import rclpy                     #ROS2のクライアントのためのライブラリ
from rclpy.node import Node      #ノードを実装するためのNodeクラス（クラスは第10回で）
from std_msgs.msg import Int16   #通信の型（16ビットの符号付き整数）

rclpy.init()
node = Node("talker")            #ノード作成（nodeという「オブジェクト」を作成）
pub = node.create_publisher(Int16, "countup", 10)   #パブリッシャのオブジェクト作成
n = 0 #カウント用変数

def cb():                       # コールバック関数
    global n                    # n のグローバル化
    msg = Int16()               # メッセージのインスタンス化
    msg.data = n                # インスタンスに n を記入
    pub.publish(msg)            # msg のパブリッシュ
    n += 1

node.create_timer(0.5, cb)      # タイマー設定
rclpy.spin(node)                # 実行（無限ループ）

