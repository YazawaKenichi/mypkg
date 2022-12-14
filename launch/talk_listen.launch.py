import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():

    talker = launch_ros.actions.Node(
            package='mypkg',
            executable='talker'
            )
    listener = launch_ros.actions.Node(
            package='mypkg',
            executable='listener',
            output='screen'
            )

    return launch.LaunchDescription([talker, listener])

# launch ファイルは .launch.py で終わることを setup.py に記述したので、
# そのルールを守ってファイル名を決定する
# ros2 launch mypkg talk_listen.launch.py を実行すると executable が同時に実行される？

