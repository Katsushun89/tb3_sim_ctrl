# TB3_sim_env

Turtlebot3シミュレーション環境セットアップパッケージ

## 概要

TB3_sim_envは、ROS2 Jazzy環境でTurtlebot3のGazeboシミュレーションを簡単にセットアップするためのパッケージです。新しいGazebo（旧Ignition Gazebo）に対応し、必要な依存パッケージの自動インストールとビルドを行います。

## 動作環境

- ROS2 Jazzy
- Ubuntu 24.04 (Noble)
- Docker環境推奨

## 環境構築手順

### 1. セットアップスクリプトの実行

```bash
# Docker環境に入る（ホスト側で実行）
~/ros2_docker_ws/run.sh

# Docker環境内で実行
cd ~/ros2_ws/src/TB3_sim_ctrl/TB3_sim_env/script/
./setup_turtlebot3_simulation.sh
```

このスクリプトは以下を自動実行します：
- Gazebo関連パッケージのインストール
- Turtlebot3依存パッケージ（Navigation2、Cartographer等）のインストール
- Turtlebot3リポジトリのクローン（jazzyブランチ）
- シミュレーション用パッケージのビルド（実機用パッケージは除外）
- 環境変数の設定

### 2. シミュレーションの起動

#### 方法1: launch_tb3_sim.shスクリプトを使用（推奨）

```bash
cd ~/ros2_ws/src/TB3_sim_ctrl/TB3_sim_env/script/

# Turtlebot3ワールドで起動（デフォルト）
./launch_tb3_sim.sh world

# 空のワールドで起動
./launch_tb3_sim.sh empty

# ハウスワールドで起動
./launch_tb3_sim.sh house
```

#### 方法2: 手動で環境変数を設定して起動

```bash
# 環境変数の設定
export TURTLEBOT3_MODEL=burger

# ワークスペースのソース
source ~/ros2_ws/install/setup.bash

# シミュレーション起動
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## ロボットモデルの選択

環境変数`TURTLEBOT3_MODEL`で使用するロボットモデルを選択できます：

- `burger`（デフォルト）: 小型の2輪差動駆動ロボット
- `waffle`: 大型の2輪差動駆動ロボット
- `waffle_pi`: waffle + Raspberry Pi Cameraモデル

```bash
export TURTLEBOT3_MODEL=waffle
```

## キーボード操作

別ターミナルで以下を実行してキーボード操作が可能です：

```bash
# ワークスペースのソース
source ~/ros2_ws/install/setup.bash

# キーボード操作ノードの起動
ros2 run turtlebot3_teleop teleop_keyboard
```

操作方法：
- `w`: 前進
- `x`: 後退
- `a`: 左旋回
- `d`: 右旋回
- `s`: 停止
- `space`: 緊急停止

## 公開されるトピック

シミュレーション起動時に以下の主要トピックが公開されます：

- `/cmd_vel`: 速度指令（入力）
- `/odom`: オドメトリ情報
- `/scan`: LiDARスキャンデータ
- `/imu`: IMUセンサーデータ
- `/joint_states`: ジョイント状態
- `/tf`, `/tf_static`: 座標変換情報

## トラブルシューティング

### TURTLEBOT3_MODELエラーが出る場合

```bash
export TURTLEBOT3_MODEL=burger
```

### ビルドエラーが出る場合

DynamixelSDK関連のエラーは無視して問題ありません（実機用パッケージのため）。

### Gazeboが起動しない場合

```bash
# Gazeboの再インストール
sudo apt install --reinstall ros-jazzy-ros-gz-sim
```

## 関連リンク

- [Turtlebot3公式ドキュメント](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/)