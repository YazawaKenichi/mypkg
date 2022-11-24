# ROS2 の講義

## ROS2
ROS1 があって、それでは物足りない場面が出てきたから ROS2 が作られた。

場合によってはほとんどプログラムを書くことなくロボットを動かせるようになる。

ロボットの各機能を別のプログラムとして実行する。

プログラム同士はデータをやり取りするだけで大きなプログラムが動く。

データのやり取りのための型は自分で決めることができる。

topic : 道の名前

message : 道を流れるデータ本体

publisher : ノードから出る矢印

listener : ノードに入る矢印

topic を chatter 、 node を talker とすると、tolker ノードは publisher の chatter と言うことができる。

publisher はデータをとにかく垂れ流す。subscriber は必要なときにデータを見に行く。

package.xml はパッケージがどのようなものか説明するためのファイル

パッケージの説明をするとか、ライセンスの記述とか、第三者に説明するだけなら README.md で良くない？

[package.xml の必要性](https://qiita.com/np_hsgw/items/de6316d69ab984c44a82) 

上記サイトによると ROS のシステム内でも package.xml を参照しているため、xml ファイルでちゃんとしっかり記述するべき。

package.xml がパッケージのルートディレクトリとして扱われるらしい。

### ROS2 のセットアップ
```
git clone https://github.com/ryuichiueda/ros2_setup_scripts
cd ros2_setup_scripts
./setup.bash
source ~/.bashrc
```

### 動作確認
```
ros2 run demo_nodes_py talker
ros2 run demo_nodes_py listener
```
listener が hear していたら問題なく動作している。

talker だけ起動したとき、talker ノードが message の publish を垂れ流していることがわかる。

listener だけ起動したとき、publisher が存在しないことから何も受け取れていないことがわかる。

```
ros2 run rqt_graph rqt_graph
```
を実行することで、ROS 通信の構造が確認できる。

### パッケージの作成
```
~/ros2_ws/src$ ros2 pkg create mypkg --build-type ament_python
```

### ライセンスやメンテナーの設定
package.xml だけでなく、setup.py も変更する。

### ~/.bashrc の追記
```
source ~/ros2_ws/install/setup.bash
source ~/ros2_ws/install/local_setup.bash
```
```
source ~/.bashrc
```

### パッケージのリストを表示
```
ros2 pkg list | grep mypkg
```

### スクリプトの作成
#### パブリッシャを持つノード
```
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

rclpy.init()
node = Node("talker")
pub = node.create_publisher(Int16, "countup", 10)
n = 0

def cb():
    global n
    msg = Int16()
    msg.data = n
    pub.publish(msg)
    n += 1

node.create_timer(0.5, cb)
rclpy.spin(node)
```
##### `package.xml` に利用するモジュールを追加する
```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```
##### `setup.py` にエントリポイントを登録
```
entry_points={
'console_scripts': [
    'talker = mypkg.talker:main',
    #'listener = mypkg.llistener:main'
],
```
##### 利用するパッケージを確認してインストール
Ubuntu 22.04 : humble
Ubuntu 20.04 : foxy
```
sudo rosdep install -i --from-path src --rosdistro humble -y
```
##### ビルド
```
colcon build
source ~/.bashrc
```

##### 実行
```
ros2 run mypkg talker
```
何も表示されないのが正しい。
##### 他の端末でサブスクライブ
```
ros2 topic echo /countup
```

#### サブスクライバを持つノードの記述
`listener.py`

```
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
```
##### talker と listener の実行
listener.py について、setup.py へ記述する。

colcon build する。

```
ros2 run mypkg talker
ros2 run mypkg listener
```

### launch ファイルの作成
launch file : 複数のノードを立ち上げることができる






