# TurtleBot3 AWS IoT Integration

AWS IoTとの連携によりTurtleBot3のリアルタイムデータをクラウドに送信するROSパッケージ。

## 機能

### データ収集
- **ロボット位置姿勢**: マップ座標系での位置(x, y, z)と姿勢(quaternion + yaw角)
- **ゴール座標**: 現在のナビゲーション目標位置
- **オドメトリ情報**: 線形・角速度データ
- **目標距離**: ロボットとゴールの直線距離

### データ送信
- 設定可能な送信レート(デフォルト: 1Hz)
- JSON形式でのデータ構造化
- ファイル出力(実装例)またはAWS IoT直接送信

## 使用方法

### 基本起動
```bash
# AWS IoT Publisherの起動
ros2 launch tb3_aws_iot aws_iot_publisher.launch.py

# カスタムパラメータでの起動
ros2 launch tb3_aws_iot aws_iot_publisher.launch.py \
  thing_name:=my_turtlebot3 \
  publish_rate:=2.0 \
  output_file:=/tmp/my_robot_data.json
```

### 設定ファイル使用
```bash
ros2 run tb3_aws_iot aws_iot_publisher_node --ros-args \
  --params-file src/tb3_aws_iot/config/aws_iot_config.yaml
```

## パラメータ

| パラメータ | デフォルト値 | 説明 |
|------------|--------------|------|
| `base_frame` | `base_footprint` | ロボットのベースフレーム |
| `map_frame` | `map` | マップフレーム |
| `publish_rate` | `1.0` | データ送信レート[Hz] |
| `aws_iot_endpoint` | `""` | AWS IoTエンドポイントURL |
| `thing_name` | `turtlebot3` | AWS IoT Thing名 |
| `output_file` | `/tmp/aws_iot_data.json` | 出力ファイルパス |

## データ形式

### JSON出力例
```json
{
  "timestamp": 1692123456789,
  "thing_name": "turtlebot3",
  "robot_pose": {
    "position": {
      "x": 1.234567,
      "y": -0.567890,
      "z": 0.000000
    },
    "orientation": {
      "x": 0.000000,
      "y": 0.000000,
      "z": 0.123456,
      "w": 0.992366
    },
    "yaw": 0.247614
  },
  "goal_pose": {
    "position": {
      "x": 3.500000,
      "y": 2.000000,
      "z": 0.000000
    },
    "orientation": {
      "x": 0.000000,
      "y": 0.000000,
      "z": 0.707107,
      "w": 0.707107
    },
    "yaw": 1.570796
  },
  "distance_to_goal": 2.745906,
  "odometry": {
    "linear_velocity": {
      "x": 0.150000,
      "y": 0.000000
    },
    "angular_velocity": {
      "z": 0.200000
    }
  }
}
```

## トピック

### 購読トピック
- `/odom` (nav_msgs/Odometry): ロボットのオドメトリ情報
- `/goal_marker` (visualization_msgs/Marker): ゴールマーカー情報

## 依存関係

### ROSパッケージ
- rclcpp
- geometry_msgs
- nav_msgs 
- tf2, tf2_ros, tf2_geometry_msgs
- visualization_msgs

### システム依存
- nlohmann_json (オプション、なければ内蔵JSON生成を使用)

## 開発・拡張

### 実際のAWS IoT連携
現在の実装はファイル出力のデモ版です。実際のAWS IoT連携には：

1. AWS IoT Device SDK for C++の追加
2. 認証証明書の設定
3. MQTT接続の実装

### カスタマイズポイント
- データ送信頻度の調整
- 追加センサーデータの組み込み
- エラーハンドリングの強化
- セキュリティ設定の追加