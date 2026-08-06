# 2026_8_6 — 水上翼船ファームウェア

通信側と制御側を完全に分離した、実機運用用の2ノード構成です。各フォルダは独立したPlatformIOプロジェクトで、片方から他方のソースを参照しません。

| フォルダ | 基板 | 担当 |
|---|---|---|
| `communication/` | XIAO ESP32S3 | GNSS、microSD認識、統合Web、固定Waypoint送信、制御側UART通信、Heartbeat |
| `control/` | XIAO ESP32S3 | BNO08X、ToF、INA226、VESC、PCA9685、安全管理、自動制御 |

## 実装済みの本番経路

- 姿勢はBNO08X内蔵`Rotation Vector`を直接使用。独自姿勢推定器はありません。
- 通信側XIAOのD6/D7でGNSSを受信し、10 Hzで制御側XIAOへ転送します。GNSS位置からローカル座標を作り、LOS誘導でウェイポイントを追従します。
- ToF高さ、BNO姿勢・角速度を使って左右前翼と後部ヨーを50 Hzで制御します。
- CH0は左前翼、CH1は右前翼、CH2は後部ヨーです。
- INA226とVESC UARTを常時監視し、低電圧、過電流、VESC ERPMによる拘束判定、VESC fault、通信途絶で安全停止します。AS5600は使用しません。
- 制御側XIAOのD10はVESCモータ安全リレーです。非ゼロDuty指令中のみHIGHとし、それ以外はLOWに固定します。
- DISARMED、E-STOP、FAULTではPCA9685をFull OFFにし、VESCへDuty 0を送ります。
- ManualではWeb画面から左右前翼、後部ヨー、推進を同時に指令します。
- GNSS・VESC・INA226が未接続でも、PCA9685とXIAO間通信が正常なら3本のサーボをManualで操作できます。
- VESC応答が無効な間は推進出力とD10安全リレーを有効にしません。
- 通信側XIAOのWeb画面では、ウェイポイント追従と姿勢制御を独立にON/OFFできます。固定目標はA〜Hから選択し、座標は`waypoint_drift_los_goal_time_sim`の値です。
- WP OFF／姿勢OFFは全体手動、WP OFF／姿勢ONは姿勢補助付き手動、WP ON／姿勢OFFは前翼中立のLOS航行、WP ON／姿勢ONはLOS＋姿勢・高さ制御です。
- Waypoint、Mode、手動値、ARM、STARTの順序制御はブラウザではなく通信側XIAOが担当し、制御側XIAOのACKと安全状態を確認してから次へ進みます。
- 選択した固定点へ1.5 m以内まで到達すると、XIAOが`FINAL_WAYPOINT`でDISARMEDへ遷移し、全出力を停止します。
- 外付けmicroSDはD8/D9/D10、CS GPIO21で起動時に認識確認します。SD自動記録と時刻同期は、この統合運転版にはまだ戻していません。

## ビルド

### Webからコンパイル済みファームウェアを書き込む

GitHub Actionsが通信側と制御側を自動コンパイルし、次のWeb Installerへ公開します。

**[水上翼船 Firmware Installer](https://temesotejam.github.io/2026_8_6/)**

- 制御側XIAO ESP32S3と通信側XIAO ESP32S3は別ボタンです。
- PC版ChromeまたはEdgeのWeb Serialを使用します。
- COM番号は接続環境で変わるため、書き込む側のXIAOだけを接続して確認してください。
- 書き込み前にモータ用バッテリーとサーボ外部電源を切ってください。
- 各ビルドの結合済みBIN、SHA-256、manifestはGitHub Actionsのartifactからも取得できます。

### PlatformIOを使う

```bash
pio run -d control
pio run -d communication
```

書き込み時は、各XIAOに割り当てられたCOMポートを指定してください。

```bash
pio run -d control -t upload --upload-port COM4
pio run -d communication -t upload --upload-port COM6
```

配線、制御則、安全条件、操作手順は[`docs/PRODUCTION_IMPLEMENTATION.md`](docs/PRODUCTION_IMPLEMENTATION.md)にまとめています。
