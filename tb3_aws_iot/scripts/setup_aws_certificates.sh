#!/bin/bash

# AWS IoT証明書セットアップスクリプト
# Usage: ./setup_aws_certificates.sh [cert_directory]

set -e

# 証明書保存ディレクトリ
CERT_DIR="${1:-$HOME/.aws/tb3_aws_iot/certs}"

echo "=== AWS IoT Certificate Setup ==="
echo "Certificate directory: $CERT_DIR"

# ディレクトリ作成
mkdir -p "$CERT_DIR"

echo ""
echo "1. AWS IoT Consoleで以下を実行してください:"
echo "   - Thing作成: turtlebot3"
echo "   - 証明書作成（自動生成）"
echo "   - ポリシー作成・アタッチ"
echo "   - エンドポイント取得"
echo ""
echo "2. ダウンロードした証明書ファイルを以下に配置:"
echo "   $CERT_DIR/device-certificate.pem.crt"
echo "   $CERT_DIR/private.pem.key"
echo "   $CERT_DIR/AmazonRootCA1.pem"
echo ""

# Amazon Root CA 1をダウンロード（存在しない場合）
if [ ! -f "$CERT_DIR/AmazonRootCA1.pem" ]; then
    echo "Downloading Amazon Root CA 1..."
    curl -o "$CERT_DIR/AmazonRootCA1.pem" https://www.amazontrust.com/repository/AmazonRootCA1.pem
    echo "Amazon Root CA 1 downloaded"
fi

# AWS CLIがインストールされているかチェック
if command -v aws >/dev/null 2>&1; then
    echo ""
    echo "AWS CLI detected. You can also use AWS CLI for IoT operations:"
    echo ""
    echo "# Create thing"
    echo "aws iot create-thing --thing-name turtlebot3"
    echo ""
    echo "# Create and download certificate"
    echo "aws iot create-keys-and-certificate --set-as-active --output text > cert_info.txt"
    echo ""
    echo "# Create policy"
    echo "aws iot create-policy --policy-name TurtleBot3Policy --policy-document file://policy.json"
    echo ""
    echo "# Attach policy to certificate"
    echo "aws iot attach-policy --policy-name TurtleBot3Policy --target <certificate-arn>"
    echo ""
    echo "# Attach certificate to thing"
    echo "aws iot attach-thing-principal --thing-name turtlebot3 --principal <certificate-arn>"
    echo ""
fi

echo ""
echo "3. 証明書配置後、以下でパーミッション設定:"
echo "   chmod 600 $CERT_DIR/private.pem.key"
echo "   chmod 644 $CERT_DIR/*.crt $CERT_DIR/*.pem"
echo ""

# 証明書ファイルの確認
echo "4. 証明書ファイルの確認:"
for file in "device-certificate.pem.crt" "private.pem.key" "AmazonRootCA1.pem"; do
    if [ -f "$CERT_DIR/$file" ]; then
        echo "   ✓ $file exists"
    else
        echo "   ✗ $file missing"
    fi
done

echo ""
echo "5. launchファイルでエンドポイントを設定:"
echo "   aws_iot_endpoint:=YOUR-ENDPOINT.iot.REGION.amazonaws.com"
echo ""
echo "Setup complete! See AWS_SETUP.md for detailed instructions."