import rclpy
from rclpy.node import Node
# from std_msgs.msg import Int16
# from person_msgs.msg import Person  # 型の変更
from person_msgs.srv import Query

def main():
    rclpy.init()
    node = Node("listener")
    client = node.create_client(Query, 'query') # サービスクライアントの作成
    while not client.wait_for_service(timeout_sec = 1.0):   # サービス待ち
        node.get_logger().info('waiting ... ')

    req = Query.Request()
    req.name = "川原 脩慈"
    future = client.call_async(req) # 非同期でサービスを呼び出し
    while rclpy.ok():
        rclpy.spin_once(node)   # 一回だけサービスを呼び出したら終わる
        if future.done():   # 終わってたら
            try:
                response = future.result()  # 結果を受取
            except:
                node.get_logger().info('Failure x_x ')
            else:   # except じゃなかったら
                node.get_logger().info("age: {}".format(response.age))

            break   # exit while
    node.destroy_node() # 削除
    rclpy.shutdown()    # 停止

if __name__ == '__main__':  # ライブラリを区別するための記法らしい
    main()

"""
def cb(msg):
    global node
    node.get_logger().info("Listen: %s" % msg)

rclpy.init()
node = Node("listener")
pub = node.create_subscription(Person, "person", cb, 10)
rclpy.spin(node)
"""

