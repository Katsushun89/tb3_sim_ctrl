# TB3_sim_ctrl

TurtleBot3 simulator on ROS2 Jazzy

## 概要

TB3_sim_ctrlは、ROS2 Jazzy環境でTurtlebot3のシミュレーションと制御を行うためのメタパッケージです。複数の機能別ROS2パッケージを統合管理し、Turtlebot3の開発環境を提供します。

## パッケージ構成

### [TB3_sim_env](./TB3_sim_env/README.md)

Turtlebot3シミュレーション環境のセットアップと管理を行うパッケージです。

**主な機能：**
- Gazebo環境の自動セットアップ
- Turtlebot3関連パッケージの一括インストール
- シミュレーション起動スクリプトの提供
- ROS2 Jazzyおよび新しいGazebo（旧Ignition）対応

**クイックスタート：**
```bash
# セットアップ
cd ~/ros2_ws/src/TB3_sim_ctrl/TB3_sim_env/script/
./setup_turtlebot3_simulation.sh

# シミュレーション起動
./launch_tb3_sim.sh world
```

詳細は[TB3_sim_env/README.md](./TB3_sim_env/README.md)を参照してください。

## 動作環境

- ROS2 Jazzy
- Ubuntu 24.04 (Noble)
- Docker環境推奨

## インストール

```bash
# ワークスペースに移動
cd ~/ros2_ws/src/

# リポジトリのクローン
git clone https://github.com/Katsushun89/TB3_sim_ctrl.git

# TB3_sim_envのセットアップを実行
cd TB3_sim_ctrl/TB3_sim_env/script/
./setup_turtlebot3_simulation.sh
```

## 今後の追加予定パッケージ

- **TB3_nav**: 自律ナビゲーション機能
- **TB3_slam**: SLAM（地図作成と自己位置推定）
- **TB3_control**: カスタム制御アルゴリズム
- **TB3_vision**: 画像処理・認識機能

## ライセンス

Apache License 2.0

## 貢献

Issue報告やPull Requestを歓迎します。

## 関連リンク

- [Turtlebot3公式](https://www.turtlebot.com/)
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/)
