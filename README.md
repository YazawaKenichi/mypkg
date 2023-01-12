# mypkg
[![test](https://github.com/YazawaKenichi/mypkg/actions/workflows/test.yaml/badge.svg)](https://github.com/YazawaKenichi/mypkg/actions/workflows/test.yaml)
## 概要
このリポジトリは、千葉工業大学 未来ロボティクス学科 2022 年度 ロボットシステム学 の講義課題で作成した ROS2 パッケージです。

|内容物|説明
|:---:|:---
|talker.py|実行直後に数字をカウントアップし、`countup` トピックにパブリッシュします。
|listener.py|`countup` トピックにパブリッシュされている数字を取得 ( サブスクライブ ) し、標準出力します。
|talk_listen.launch.py|`talker.py` と `listener.py` を立ち上げる `launch` ファイルです。

## 動作環境
- OS : Ubuntu 22.04 jammy
- ROS2 : humble

## インストール
1. このリポジトリのクローン
    ```
    cd ~/ros2_ws/src
    git clone https://github.com/yazawakenichi/mypkg
    ```
1. パッケージのビルド
    ```
    cd ~/ros2_ws
    colcon build
    ```
1. パッケージの実行可能ファイルを使えるようにするスクリプトを `~/.bashrc` 末行に追加
    ```
    echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
    echo 'source ~/ros2_ws/install/local_setup.bash' >> ~/.bashrc
    ```
1. `~/.bashrc` の変更を適用
    ```
    source ~/.bashrc
    ```

## 実行方法
以下のコマンドを実行してで `listener.py` と `talker.py` ノードを同時に立ち上げる。
```
ros2 launch mypkg talk_listen.launch.py
```

## 実行結果
```
[listener-2] [INFO] [1673535223.854988507] [listener]: Listen: 0
[listener-2] [INFO] [1673535224.337194177] [listener]: Listen: 1
[listener-2] [INFO] [1673535224.837239859] [listener]: Listen: 2
[listener-2] [INFO] [1673535225.337852372] [listener]: Listen: 3
[listener-2] [INFO] [1673535225.838144686] [listener]: Listen: 4
[listener-2] [INFO] [1673535226.338456488] [listener]: Listen: 5
```

## ノードとトピック
![image](https://github.com/YazawaKenichi/mypkg/blob/main/.pictures/rosgraph.png)

## ライセンス
- このソフトウェアパッケージは、 MIT ライセンスのもと、再頒布および使用が許可されます。
- (C) 2022 KAWAHARA Shuji

