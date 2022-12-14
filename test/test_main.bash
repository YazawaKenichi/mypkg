#!/bin/bash
# coding: utf-8

ng () {
    echo NG at Line $1
    res=1
}

res=0

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build
source $dir/.bashrc
timeout 10 ros2 launch mypkg talk_listen.launch.py > /tmp/mypkg.log
# memo:↑ これでおよそ 17 くらいまで入る

cat /tmp/mypkg.log |
    grep 'Listen: 10'
[ $? == 0 ] || ng ${LINENO}

cat /tmp/mypkg.log |
    grep 'Listen: 60'
[ $? == 0 ] || ng ${LINENO}

