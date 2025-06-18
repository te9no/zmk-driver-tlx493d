# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## プロジェクト概要

これはZMK (Zephyr Mechanical Keyboard) ファームウェア用のInfineon TLX493D 3次元磁気センサードライバーモジュールです。このドライバーはTLX493D磁気センサーからの入力を読み取り、マウスのような相対的な動き（X、Y軸とスクロール）を生成します。

## アーキテクチャ

### 主要コンポーネント

1. **メインドライバー**: `drivers/input/input_tlx493d.c`
   - I2C通信を使用してTLX493D センサーからデータを読み取り
   - センサーの初期化とキャリブレーション処理
   - エラー回復とバスリカバリー機能
   - 自動キャリブレーション機能（30秒無動作時）
   - ハイステリシス機能による動き検出

2. **ヘッダーファイル**: `drivers/input/input_tlx493d.h`
   - センサーレジスタアドレスと設定値の定義
   - デフォルト構成値と閾値の定義

### デバイスツリー設定

- **バインディング**: `dts/bindings/input/infineon,tlx493d.yaml`
- **設定項目**:
  - `polling-interval-ms`: センサー読み取り間隔（デフォルト: 10ms）

### Kconfig設定

主要な設定オプション：
- `INPUT_TLX493D`: ドライバーの有効化
- `INPUT_TLX493D_POLLING_INTERVAL_MS`: ポーリング間隔
- `INPUT_TLX493D_Z_THRESHOLD`: Z軸スクロール閾値
- `INPUT_TLX493D_ROTATION_SCALER`: 回転感度スケーラー

## 開発コマンド

### ビルド
```bash
# ZMKプロジェクトでモジュールとして使用
west build
```

### デバッグ
ドライバーはZephyrログシステムを使用：
- ログレベル: `CONFIG_INPUT_LOG_LEVEL`
- ログモジュール: `tlx493d`

## 技術的詳細
- /sample/Infineon-TLV493D-A1B6_3DMagnetic-UserManual-v01_03-EN.pdf:センサーの詳細な仕様が記載されています。
- /sample/XENSIV_3D_Magnetic_Sensor_TLx493D:センサーのArduino用のライブラリ・サンプルコードです。

### センサー初期化シーケンス

1. 電源サイクル実行
2. リカバリーフレーム送信（0xFF）
3. リセットコマンド送信（0x00）
4. 工場設定読み取り
5. MOD1/MOD2レジスタ設定
6. キャリブレーション実行

### エラー処理
- I2Cバスリカバリー機能
- 連続エラー時の自動再初期化
- 指数バックオフによるリトライ制御
- 最大5回のバスリカバリー試行

### 入力処理
- デッドゾーン機能（X/Y: 3, Z: 5）
- ハイステリシス閾値: 20
- 相対入力として報告（INPUT_REL_X, INPUT_REL_Y, INPUT_REL_WHEEL）

### 自動キャリブレーション
- 30秒間動きが検出されない場合に自動実行
- 100サンプルの平均値を基準点として設定

## ファイル構造

```
├── CMakeLists.txt          # ビルド設定
├── Kconfig                 # 設定オプションのエントリポイント
├── drivers/
│   ├── input/
│   │   ├── input_tlx493d.c    # メインドライバー実装
│   │   └── input_tlx493d.h    # ドライバーヘッダー
├── dts/bindings/input/
│   └── infineon,tlx493d.yaml  # デバイスツリーバインディング
└── zephyr/
    └── module.yml          # Zephyrモジュール定義
```

## 主要定数とマクロ

- `TLV493D_REG_BX_MSB`: 0x00 (X軸データレジスタ)
- `AUTO_RECALIBRATION_TIMEOUT_MS`: 3000ms（自動キャリブレーション）
- `HYSTERESIS_THRESHOLD`: 20（動き検出閾値）

# ZMK Project Coding Instructions for AI

## Critical Prerequisites

**MANDATORY: Before making ANY implementation changes, you MUST:**

1. **Consult the ZMK Repository Expert**: Query the Deepwiki MCP server (repoName: `zmkfirmware/zmk`) to understand the current implementation and identify potential issues.

2. **Consult the Zephyr Repository Expert**: Since ZMK is Zephyr-based, also query the Deepwiki MCP server (repoName: `zephyrproject-rtos/zephyr`) for Zephyr-specific implementation details when needed.

3. **Share Context**: Provide the MCP server with:
   - Current implementation details
   - Identified problems or requirements
   - Your proposed implementation approach

4. **Validate Implementation Strategy**: Confirm your implementation plan with the ZMK expert before proceeding.

5. **Update Documentation**: After making any changes, always update the README.md with relevant information about new features, usage instructions, or implementation status.

## Why This Process is Essential

- **Pre-trained Knowledge Limitations**: LLM training data may not reflect the latest ZMK architecture, APIs, or best practices
- **Build Failure Prevention**: ZMK has specific conventions and dependencies that may not be obvious from general knowledge
- **Implementation Accuracy**: The ZMK codebase has evolved significantly, and outdated approaches will likely fail

## Remember

**NO ASSUMPTIONS**: Always verify current ZMK practices before implementation. What worked in older versions may not work in current releases.

**EXPERT FIRST**: Consult `zmkfirmware/zmk` and `zephyrproject-rtos/zephyr` repository experts before coding, not after encountering build failures.

## Build Verification

**MANDATORY BUILD CHECK**: After any implementation or modification, verify that the build passes by running the following command:

```bash
ZMK_CONFIG_NAME=zmk-config-MKB just --justfile ../../zmk-config/zmk-config-MKB-1/Justfile  --working-directory ../.. build MKB_L_RZT -S zmk-usb-logging
```

If build fails, fix the issue and try again until it passes.

Important: Build may take several minutes to complete. Make sure to wait for the build to finish completely before determining if the build is successful.


## Branch Strategy

- For compatibility for user's west.yml, no breaking changes should be made without updating the branch name.
- Current stable branch is `v1`.
- When making breaking changes, bump the branch name to `v2`.
- For changes with no breaking changes, new branches should be created from `v1` branch with prefix `v1-`.