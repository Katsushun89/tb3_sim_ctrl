# tb3_ctrl_bringup

Turtlebot3制御とナビゲーションのbringupパッケージ

## 概要

tb3_ctrl_bringupは、Turtlebot3のシミュレーション環境でNav2（Navigation2）スタックを統合して起動するためのlaunchファイルを提供するパッケージです。

## launchファイル

### tb3_simulation_launch.py

nav2_bringupパッケージの`tb3_simulation_launch.py`を直接呼び出すシンプルなlaunchファイルです。

**使用方法：**
```bash
ros2 launch tb3_ctrl_bringup tb3_simulation_launch.py
```

**主なパラメータ：**
- `slam`: SLAM実行の有無（デフォルト: False）
- `map`: 使用するマップファイルのパス
- `use_sim_time`: シミュレーション時間の使用（デフォルト: true）
- `params_file`: Nav2パラメータファイル
- `autostart`: Nav2スタックの自動起動（デフォルト: true）

### tb3_gazebo_nav2_launch.py

GazeboシミュレーションとNav2スタックを一緒に起動する統合launchファイルです。

**使用方法：**
```bash
ros2 launch tb3_ctrl_bringup tb3_gazebo_nav2_launch.py
```

**主なパラメータ：**
- `world`: Gazeboワールド（turtlebot3_world, turtlebot3_house, empty_world）
- `x_pose`, `y_pose`, `z_pose`: ロボットの初期位置
- `slam`: SLAM実行の有無
- `use_rviz`: RViz2の起動有無（デフォルト: true）
- その他Nav2関連パラメータ

## 使用例

### 基本的な起動（Gazebo + Nav2 + RViz2）
```bash
# ワークスペースのビルドとソース
cd ~/ros2_ws
colcon build --packages-select tb3_ctrl_bringup
source install/setup.bash

# Turtlebot3モデルの設定
export TURTLEBOT3_MODEL=burger

# 統合launch実行
ros2 launch tb3_ctrl_bringup tb3_gazebo_nav2_launch.py
```

### SLAMモードで起動
```bash
ros2 launch tb3_ctrl_bringup tb3_gazebo_nav2_launch.py slam:=True
```

### 既存マップを使用
```bash
ros2 launch tb3_ctrl_bringup tb3_gazebo_nav2_launch.py \
  slam:=False \
  map:=/path/to/your/map.yaml
```

### カスタム初期位置
```bash
ros2 launch tb3_ctrl_bringup tb3_gazebo_nav2_launch.py \
  x_pose:=0.0 \
  y_pose:=0.0 \
  z_pose:=0.01
```

## 依存関係

- nav2_bringup
- turtlebot3_gazebo
- turtlebot3_navigation2
- rviz2

## トラブルシューティング

### Nav2が起動しない場合
```bash
# Nav2パッケージの確認
ros2 pkg list | grep nav2_bringup
```

### プロセスのクリーンアップ
```bash
# TB3_sim_envのスクリプトを使用
~/ros2_ws/src/TB3_sim_ctrl/TB3_sim_env/script/reset_gazebo_processes.sh
```