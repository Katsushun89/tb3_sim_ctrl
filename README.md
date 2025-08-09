# TB3_sim_ctrl

TurtleBot3のシミュレーション制御用ROS2パッケージ群

## パッケージ構成

### 1. cyclic_goal_navigator
自律探索ノード。ローカルコストマップに基づいて連続的にゴールを設定し、無限探索を実行。

**主な機能:**
- 2段階ゴール選択（前方優先→左右探索）
- 障害物回避とレイキャスティング
- 走行経路のRViz永続表示
- 緊急脱出モード

**起動:**
```bash
ros2 launch cyclic_goal_navigator cyclic_goal_navigator.launch.py
```

### 2. tb3_ctrl_bringup
Nav2とTurtleBot3シミュレーション環境の統合起動パッケージ。

**主な機能:**
- Nav2の簡易起動
- カスタムRViz設定の適用
- 初期位置の自動設定（5秒後）
- シミュレーション環境の一括起動

**起動:**
```bash
ros2 launch tb3_ctrl_bringup tb3_nav2_simple.launch.py
```

**ファイル構成:**
- `launch/tb3_nav2_simple.launch.py`: Nav2統合起動
- `config/nav2_view.rviz`: カスタムRViz設定
- `scripts/set_initial_pose.py`: 初期位置自動設定スクリプト

## 環境構築

### 必要な依存関係
- ROS2 Jazzy
- Nav2
- TurtleBot3パッケージ
- Gazeboシミュレータ

### ビルド方法
```bash
cd ~/ros2_ws
colcon build --packages-select cyclic_goal_navigator tb3_ctrl_bringup
source install/setup.bash
```

## 使用例

### 完全自動探索の実行
```bash
# Terminal 1: Nav2とシミュレーション起動（初期位置自動設定付き）
ros2 launch tb3_ctrl_bringup tb3_nav2_simple.launch.py

# Terminal 2: 自律探索開始（少し待ってから）
ros2 launch cyclic_goal_navigator cyclic_goal_navigator.launch.py
```

### パラメータ調整例
```bash
# コストマップ閾値を変更して探索
ros2 launch cyclic_goal_navigator cyclic_goal_navigator.launch.py cost_threshold:=3
```

## トラブルシューティング

### 初期位置が設定されない場合
- Nav2が完全に起動するまで待つ（5秒以上）
- 手動で設定: RVizの"2D Pose Estimate"を使用

### ゴールが設定されない場合
- コストマップが正しく配信されているか確認
```bash
ros2 topic echo /local_costmap/costmap --once
```

### 経路が表示されない場合
- RVizで`/goal_markers`トピックが表示設定になっているか確認
- MarkerArrayの表示を有効化

## ライセンス
Apache 2.0

## 作者
s-katsu (katsushun89@gmail.com)
