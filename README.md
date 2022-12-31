# mypkg

## 概要
このリポジトリは、千葉工業大学 未来ロボティクス学科 2022 年度 ロボットシステム学 の講義課題で作成した ROS2 パッケージです。

`talker.py` で数字をパブリッシュします `listener.py` でパブリッシュされた数字を取得します。

|内容物|説明
|:---:|:---
|talker.py|実行直後に数字をカウントアップし、`countup` トピックにパブリッシュします。
|listener.py|`countup` トピックにパブリッシュされている数字を取得 ( サブスクライブ ) し、標準出力します。
|talk_listen.launch.py|`talker.py` と `listener.py` を立ち上げる `launch` ファイルです。

## 動作環境
- OS : Ubuntu 22.04 jammy
- ROS2 : humble

## 使用方法
```
ros2 launch mypkg talk_listens.launch.py
```

## ノードとトピック

## ライセンス
- このソフトウェアパッケージは、 MIT ライセンスのもと、再頒布および使用が許可されます。
- (C) 2022 KAWAHARA Shuji

