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
これによって `mypkg.talker.py` および `mypkg.listener.py` は `def main():` を記述してその中に処理を書く必要がある。

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

#### launch ファイルを作成して使えるようにするまでの最短手順
1. パッケージのディレクトリに `launch` ディレクトリの作成
    ```
    mkdir launch
    ```
2. setup.py に launch ファイルの場所を記載
    ```
    import os
    from glob import glob
    ... 中略 ...
    data_files = [
    ... 中略 ...
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    ```
    `os.path.join(arg1, arg2)` は `arg1` のパスに `arg2` のパスを連結することができる。

    `glob(arg1)` は正規表現で検索する。

    `'launch/*.launch.py'` を検索することになる。

3. `package.xml` に依存関係を記述
    ```
    <exec_depend>launch_ros</exec_depend>
    ```

4. `launch` ファイルの作成
    ```
    import launch
    import launch.actions
    import launch.substitutions
    import launch_ros.actions

    def generate_launch_description():
        talker = launch_ros.actions.Node(
            package='mypkg',
            executable='talker',
        )
        listener = launch_ros.actions.Node(
            package='mypkg',
            executable='listener',
            output='screen'
        )
        return launch.LaunchDescription([talker, listener])
    ```

5. `launch` ファイルを作成したら `colcon build` する
    ```
    colcon build
    ```
1. `ros2 launch` する
    ```
    ros2 launch mypkg talk_listen.launch.py
    ```
    これを実行すると `executable` で指定したノードが同時に立ち上がる

#### エラー対処
##### name 'os' is not defined

###### 原文

```
Traceback (most recent call last):s]
  File "<string>", line 1, in <module>
  File "/usr/lib/python3.10/distutils/core.py", line 215, in run_setup
    exec(f.read(), g) [mypkg - 0.4s]
  File "<string>", line 14, in <module>
NameError: name 'os' is not defined
... 以下略 ...
```

###### 対処法
`setup.py` に `import os` とか `from glob import glob` を書いてないのが原因。

書け！

##### Node.__init__() missing 1 required keyword-only argument: 'executable'

###### 原文
```
[ERROR] [launch]: Caught exception in launch (see debug for traceback): Caught exception when trying to load file of format [py]: Node.__init__() missing 1 required keyword-only argument: 'executable'
```

###### 対処法

わからん！

いろんなサイトを歩き回って以下のことをした。

1. `.launch.py` のインデントの数を見直した
1. 余計な改行をなくした
1. `colcon build` した
1. パソコン自体を再起動した
1. なぜか `ros2 launch hogepkg hogehoge.launch.py` できるようになった

おそらく、`colcon build` する前後で `source ~/.bashrc` する必要があるのかも？

### 独自のメッセージ型の作成

パブリッシュするデータの型を作成する。

以下を実行することで既存の型の一覧を表示することが可能。

```
ros2 interface list
```

#### 型を定義するためだけのパッケージを作成する
1. パッケージを作成する
    ```
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_cmake <PACKAGENAME_msgs>
    ```

    `PACKAGENAME_msgs` は方の名前にしておくとわかりやすくて良い。

    また、パッケージを作成したので、`package.xml` の `description`, `maintainer`, `license` をちゃんと書く。
    必要に応じて `test_depend` や `exec_depend`
    - `test_depend` : テストに必要なパッケージ
    - `exec_depend` : 実行に必要なパッケージ
1. `msg` ディレクトリを作成する
    ```
    cd <PACKAGENAME_msgs>
    mkdir msgs
    ```
1. `TYPENAME.msg` を作成し、型を定義する

    以下は `Person.msg` という、人の情報を格納することを目的とした型を作成して例に上げる。
    ```
    string name
    uint8 age
    uint8 height
    string like_food
    ```
    このようにすることで、各言語で使用可能なクラスが定義される。
1. `CMakeList.txt` の編集
    ```
    find_package(ament_cmake REQUIRED)
    # 以下を追加
    find_package(rosidl_default_generators REQUIRED)
    rosidl_generate_interfaces(${PROJECT_NAME})
        "msg/Person.msg"
    )
    ```
1. `package.xml` の編集
    ```
    <build_depend>rosidl_default_generators</build_depend>
    <exec_depend>rosidl_default_runtime</exec_depend>
    <member_of_group>rosidl_interface_packages</member_of_group>
    ```
1. ビルドして環境に反映
    ```
    cd ~/ros2_ws
    colcon build
    source ~/.bashrc
    ```
1. 型が利用できるようになっているか確認
    ```
    ros2 interface show person_msgs/msg/<TYPENAME>
    ```
    ##### エラー
    ```
    Unknown package `person_msgs`
    ```
    `person_msgs` パッケージがないと言っている。

    自分は `colcon build`, `source ~/.bashrc` したら治った。

    誤字とかしても同じエラーが出そう...

#### 作成したメッセージを利用する
他の型と使い方は対して変わらない
```
from <PACKAGENAME_msgs.msg> import <TYPENAME>
```
でインポートする。

```
pub = node.create_publisher(<TYPENAME>, "<TOPICNAME>", 10)
```
でパブリッシュする。

また、スクリプトを書き換えたら `colcon build source ~/.bashrc` を忘れないこと！

`talker` 側のお試し実行は以下でかんたんにできる。

```
ros2 topic echo <PACKAGENAME> <TALKER.py>
```

あと、どうせ `launch` ファイルを作成したのだから、`ros2 launch` でテスト実行するのも良し！


### サービスの実行
#### 概要
`topic` はいつ `publish` しても良いし、いつ `subscribe` しても良い。

ノード同士が干渉することはない。

ではノード同士で直接やり取りがしたくなった場合はどうすればいいのか...

そこで**サービス**というものが必要になってくる。

##### 結局

サービスとは、あるノードが別のノードに仕事を依頼する仕組みのこと。

例えば

人の名前を送ったら、年齢が返ってくるサービス

この例を参考にサービスを作成してみる。

#### 実装するサービス

- 人の名前を送ったら、年齢が返ってくる
    - `listener` が依頼される側
    - `talker` が依頼する側

#### サービスの作成
0. 前提
    - `<PACKAGENAME_msgs>` : person_msgs
    - `<TYPENAME.msg>` : Person.msg
1. `srv` ファイルの作成
    パッケージルートに `srv` ディレクトリを作成する。

    その中に `Query.srv` を作成する。

    ```
    string name
    ---
    uint8 age
    ```

    ここで `---` とは、サービスの渡す方と受け取る方を区別するための線として使用される。

    依頼者目線でそれぞれの関係性を記述すると以下の通り
    ```
    こっち側に書いてあることを要求する
    ---
    するとこっちがわに書いてあることが返ってくる
    ```

    仕事者目線では
    ```
    こっち側の内容が要求されるので
    ---
    こっち側の内容を渡す
    ```
1. `CMakeList.txt` の編集
    以下を追記
    ```
    rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/Person.msg"
        "srv/Query.srv" # ← 追加
    )
1. ビルドして確認   
    `colcon build source ~/.bashrc` した後に、以下を実行することで、正しく導入されているかどうかを確認することができる。

    ```
    ros2 interface show person_msgs/srv/Query
    ```

#### サービスのためのコールバック関数の実装（サーバの作成）
基本的には、`Query` 型のメッセージを送受信する処理だと考えれば簡単。

```
from person_msgs.srv import Query
```

```
srv = node.create_service(Query, "query", callback)
```

ただ `callback` 関数の書き方が、ひとひねりされている。

サービスは、「何かを受け取ってそれに対応する何かを返す」という性質上、受取データと送信データの２つが必要になる。

よって以下のような書き方になる。

```
def callback(request, response):
    # request がクライアントからの要求データ
    # それに対応するデータを入れた response を返す
    if request.name == "俺の彼女":
        response.age = 18
    else:
        response.age = 255
    return response
```

##### 動作確認
1. サービスノードを立ち上げる
    ```
    ros2 run mypkg talker.py
    ```
1. サービスが立ち上がっていることの確認
    ```
    ros2 service list
    ```
1. サービスにテキトーに要求してみる
    ```
    ros2 service call /query person_msgs/srv/Query "name: 俺の彼女"
    ros2 service call /query person_msgs/srv/Query "name: クソ老害ジジィ"
    ```

##### エラー
###### サービスが使用可能になるのを待っています
```
waiting for service to become available ...
```
サービスが立ち上がってない。

`ros2 run` で立ち上げろ。

#### ノードからサービスを呼び出す
面倒なのでサンプルコードそのまま載せる。

``` Python
import rclpy
from rclpy.node import Node
from person_msgs.srv import Query

def main():
    rclpy.init()
    node = Node("listener")
    client = node.create_client(Query, 'query') # サービスクライアントの作成
    while not client.wait_for_service(timeout_sec = 1.0):   # サービス待ち
        node.get_logger().info('waiting ... ')

    req = Query.Request()
    req.name = "俺の彼女"
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
```

##### 動作確認
```
ros2 run mypkg listener
```
```
ros2 run mypkg talker
```

#### パラメータ・アクション
トピック・サービスの他のデータや、処理の受け渡し方法。

- パラメータ：定数やたまに変更するデータ（センサの周波数など）

- アクション：サービスの長時間版（呼び出し側が途中経過の監視やキャンセルができる）
    - ナビゲーション（目的地を指定して終わるまで待つ）
    - マニピュレータの姿勢変更（最終的な姿勢を指定して終わるまで待つ）
    - トピックやサービスを組み合わせて実装される



