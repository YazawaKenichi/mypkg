#!/bin/bash -xv

_HOME=~
[ "$1" != "" ] && _HOME="$1"

cd $_HOME/ros2_ws
colcon build
source $_HOME/.bashrc
# talker は 0 ~ 値をパブリッシュする
# listener はそれをサブスクライブする
# talker がパブリッシュしたメッセージを listener がサブスクライブして標準出力されたものが /tmp/mypkg.log に格納される
timeout 10 ros2 launch mypkg talk_listen.launch.py > /tmp/mypkg.log

# 10 をサブスクライブしているか出力
cat /tmp/mypkg.log | grep 'Listen: 10'

