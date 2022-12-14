#!/usr/bin/python
# coding: utf-8
# talker.py

import rclpy                     #ROS2のクライアントのためのライブラリ
from rclpy.node import Node      #ノードを実装するためのNodeクラス（クラスは第10回で）
# from std_msgs.msg import Int16   #通信の型（16ビットの符号付き整数）
# from person_msgs.msg import Person # 使う型を Person 型に変更
from person_msgs.srv import Query   # 使う型を更に Query に変更

# pub = node.create_publisher(Person, "person", 10)   #パブリッシャのオブジェクト作成
# n = 0 #カウント用変数

# request には --- の上側の情報 response には --- の下側の情報
def cb(request, response):                       # コールバック関数
    if request.name == "川原 脩慈":
        response.age = 18   # 僕は永遠の 18 歳です
    else:
        response.age = 255
    return response
    """
    global n                    # n のグローバル化
    msg = Person()               # メッセージのインスタンス化
    msg.name = "川原 脩慈"                # インスタンスの属性 attribute に記入
    msg.age = n
    pub.publish(msg)            # msg のパブリッシュ
    if not n >= 200:
        n += 1
    else:
        n = 0
    """

rclpy.init()
node = Node("talker")            #ノード作成（nodeという「オブジェクト」を作成）
srv = node.create_service(Query, "query", cb)   # サービスの作成    # Query 型のメッセージを送受信するサービスとも言える？
# ↑ これで情報提供側が立ち上がる    依頼を受け取ったら cb() を実行する
# node.create_timer(0.5, cb)      # タイマー設定
rclpy.spin(node)                # 実行（無限ループ）

