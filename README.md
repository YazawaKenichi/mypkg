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

## 使用方法
リポジトリをクローンし、ROS2 パッケージのセットアップ方法に従って、当パッケージをセットアップします。

その後、以下で `listener.py` と `talker.py` ノードを同時に立ち上げます。
```
ros2 launch mypkg talk_listens.launch.py
```

## ノードとトピック
![image](https://github.com/YazawaKenichi/mypkg/blob/main/.pictures/rosgraph.png)

## ライセンス
- このソフトウェアパッケージは、 MIT ライセンスのもと、再頒布および使用が許可されます。
- (C) 2022 KAWAHARA Shuji

