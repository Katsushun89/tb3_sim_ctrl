# AWS IoT Setup Instructions

## 1. AWS IoT Thingの作成

### 1.1 AWS IoT Consoleにアクセス
1. AWS Management Consoleにログイン
2. 「IoT Core」サービスに移動

### 1.2 Thingの作成
1. 左メニューから「管理」→「モノ」を選択
2. 「モノを作成」ボタンをクリック
3. 「単一のモノを作成」を選択
4. モノの設定:
   - **モノの名前**: `turtlebot3`
   - **モノのタイプ**: `robot` (オプション)
   - **モノのグループ**: 空白のまま
5. 「次へ」をクリック

## 2. 証明書の作成とダウンロード

### 2.1 証明書の自動生成
1. 「証明書の作成」画面で「自動生成」を選択
2. 「次へ」をクリック

### 2.2 証明書ファイルのダウンロード
**重要**: すべてのファイルをダウンロードしてください
1. **デバイス証明書** (device-certificate.pem.crt) - ダウンロード
2. **公開キー** (public.pem.key) - ダウンロード
3. **プライベートキー** (private.pem.key) - ダウンロード
4. **Amazon Root CA 1** (AmazonRootCA1.pem) - ダウンロード

## 3. ポリシーの作成

### 3.1 ポリシー作成
1. 左メニューから「セキュリティ」→「ポリシー」を選択
2. 「ポリシーの作成」をクリック
3. ポリシー設定:
   - **ポリシー名**: `TurtleBot3Policy`
   - **ポリシードキュメント**:

```json
{
  "Version": "2012-10-17",
  "Statement": [
    {
      "Effect": "Allow",
      "Action": [
        "iot:Connect"
      ],
      "Resource": [
        "arn:aws:iot:*:*:client/turtlebot3"
      ]
    },
    {
      "Effect": "Allow", 
      "Action": [
        "iot:Publish"
      ],
      "Resource": [
        "arn:aws:iot:*:*:topic/$aws/things/turtlebot3/shadow/update",
        "arn:aws:iot:*:*:topic/$aws/things/turtlebot3/shadow/get"
      ]
    },
    {
      "Effect": "Allow",
      "Action": [
        "iot:Subscribe",
        "iot:Receive"
      ],
      "Resource": [
        "arn:aws:iot:*:*:topicfilter/$aws/things/turtlebot3/shadow/*"
      ]
    }
  ]
}
```

## 4. 証明書とポリシーのアタッチ

### 4.1 ポリシーのアタッチ
1. 「セキュリティ」→「証明書」から作成した証明書を選択
2. 「アクション」→「ポリシーのアタッチ」をクリック
3. `TurtleBot3Policy`を選択してアタッチ

### 4.2 証明書の有効化
1. 証明書を選択
2. 「アクション」→「有効化」をクリック

### 4.3 ThingにCertificateをアタッチ
1. 証明書を選択
2. 「アクション」→「モノのアタッチ」をクリック
3. `turtlebot3`を選択してアタッチ

## 5. エンドポイント情報の取得

1. 「設定」メニューを選択
2. **エンドポイント**をコピー（例: `xxxxx-ats.iot.us-east-1.amazonaws.com`）

## 6. 証明書ファイルの配置

ダウンロードした証明書ファイルを以下のディレクトリに配置:

```bash
mkdir -p ~/ros2_ws/src/tb3_sim_ctrl/tb3_aws_iot/certs/

# ダウンロードしたファイルをコピー
cp ~/Downloads/device-certificate.pem.crt ~/ros2_ws/src/tb3_sim_ctrl/tb3_aws_iot/certs/
cp ~/Downloads/private.pem.key ~/ros2_ws/src/tb3_sim_ctrl/tb3_aws_iot/certs/
cp ~/Downloads/AmazonRootCA1.pem ~/ros2_ws/src/tb3_sim_ctrl/tb3_aws_iot/certs/

# パーミッション設定
chmod 600 ~/ros2_ws/src/tb3_sim_ctrl/tb3_aws_iot/certs/private.pem.key
chmod 644 ~/ros2_ws/src/tb3_sim_ctrl/tb3_aws_iot/certs/*.crt
chmod 644 ~/ros2_ws/src/tb3_sim_ctrl/tb3_aws_iot/certs/*.pem
```

## 7. Device Shadowの確認

設定完了後、AWS IoT Consoleで確認:
1. 「管理」→「モノ」→「turtlebot3」を選択
2. 「Device Shadow」タブでShadowが更新されることを確認

## 8. 設定パラメータ

以下の情報をlaunchファイルまたは設定ファイルに記載:

- **AWS IoT Endpoint**: AWS Consoleの「設定」から取得
- **Thing Name**: `turtlebot3`
- **Certificate Path**: `~/ros2_ws/src/tb3_sim_ctrl/tb3_aws_iot/certs/`
- **Client ID**: `turtlebot3`

完了！これでTurtleBot3からAWS IoT Device Shadowにデータを送信できます。