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

**詳細なアルゴリズム説明:** [cyclic_goal_navigator/README.md](cyclic_goal_navigator/README.md)

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

### 3. tb3_aws_iot
AWS IoT連携ノード。TurtleBot3の位置姿勢とゴール座標をAWS IoT Device Shadowに送信。

**主な機能:**
- ロボットの位置姿勢をmap座標系で取得・変換
- ゴールマーカーからゴール座標を抽出
- AWS IoT Device Shadowへのリアルタイムデータ送信
- ゴールまでの距離計算
- 速度情報（線形・角速度）の取得

**起動:**
```bash
ros2 launch tb3_aws_iot aws_iot_publisher.launch.py aws_iot_endpoint:=YOUR-ENDPOINT.iot.REGION.amazonaws.com
```

**データフォーマット:**
- robot_pose: map座標系での位置姿勢
- goal_pose: ゴール位置姿勢
- distance_to_goal: ゴールまでの距離
- odometry: 線形・角速度情報

**AWS証明書設定:**
`~/.aws/tb3_aws_iot/certs/`に配置

## 環境構築

### 必要な依存関係
- ROS2 Jazzy
- Nav2
- TurtleBot3パッケージ
- Gazeboシミュレータ

### 必要なパッケージのインストール
[tb3_sim_env/README.md](tb3_sim_env/README.md)

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

# Terminal 2: 自律探索開始
ros2 launch cyclic_goal_navigator cyclic_goal_navigator.launch.py
```

ゴール座標と経路をrviz上に描画

下図は約15回ゴール設定したあとの様子

<img width="703" height="618" alt="image" src="https://github.com/user-attachments/assets/844185a9-01e4-4672-bab2-5270c7aef344" />

## 補足
実装は基本的にAI 系のツールを使用しています
- Claude code
- Cursor
各機能ごとに実装内容を指示して、問題あれば修正という流れを繰り返しています