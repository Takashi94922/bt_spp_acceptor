# 作業ログ

## 2026-10-31 10:44 バグ修正 #1〜#5 開始

- ブランチ: `vert`
- 起点コミット: `8b9b0b8`（だ面の向きを修正）
- 開始時点の未コミット変更（今回の作業とは無関係、そのまま保持）:
  - `main/Motion_control.cpp/.h`: getPRY に一次LPF追加
  - `main/main.cpp`: Servo4 の中心値 `.0f` → `50.0f`
  - `sdkconfig` / `sdkconfig.old` / `dependencies.lock`: `set-target esp32s3` による変更

### 対象
| # | 内容 | ファイル |
|---|---|---|
| 1 | ジャイロ ODR が `9`（& 0x07 → 1 = 14.9Hz）になっていたのを 952Hz（`6`）に修正 | `main/ESP32_i2c_LSM9DS1.cpp` |
| 2 | コマンド14（PRY目標値一括）の float 読み出し位置が 1 バイトずつ重なっていたのを修正 | `main/main.cpp` |
| 3 | コマンド10,14〜17 に長さチェックを追加、float 読み出しを memcpy に統一 | `main/main.cpp` |
| 4 | テレメトリ送信の `ESP_ERROR_CHECK` を撤去（送信失敗で abort → 再起動していた） | `main/ble_gatt_comm.cpp` |
| 5 | テレメトリタスクの自己削除をグローバルハンドル経由から `vTaskDelete(NULL)` に変更、多重起動を防止 | `main/main.cpp` |

### 結果（2026-10-31）
- #1〜#5 の修正を実施（未コミット）
  - #1: `settings.gyro.sampleRate = 6`（952Hz）。gyro帯域設定 `bandwidth = 0` は ODR 952Hz では約33Hzカットオフになる点に注意
  - #2,#3: `read_float()`（memcpy読み出し）を追加し、コマンド14 を `msg[1], msg[5], msg[9]` から読むよう修正。10,14〜17 に長さチェック追加。`set_pid_gain` の引数を `const uint8_t*` に変更
  - #4: `sendTelemetry` の送信失敗時は警告ログを出して今回分を打ち切る
  - #5: 実行中フラグ `bl_telem_busy` で多重起動を防止し、タスクは `vTaskDelete(NULL)` で自己削除
- **ビルド未確認**: ESP-IDF 5.4.1 の Python 仮想環境（`idf5.4_py3.14_env`）が存在せず export に失敗。v6.0.2 のみインストール済み
- **実機未確認**: 特に #1 で IMU の実更新レートが 14.9Hz → 952Hz に変わるため、Madgwick のゲインや LPF の挙動が変わる可能性あり

#### 動作確認（2026-10-31）
- ユーザーがビルドして実機で動作を確認（ESP32ターゲット。sdkconfig / dependencies.lock は ESP32 用に戻っている）

## 2026-10-31 11:32 改善 #6, #10 開始

- ブランチ: `vert`（起点コミット `8b9b0b8`、#1〜#5 は未コミットのまま）

### 対象
| # | 内容 | ファイル |
|---|---|---|
| 6 | IMU読み出しのI2Cトランザクションを集約し、磁気センサ（ODR 80Hz）は間引いて読む | `main/ESP32_i2c_LSM9DS1.cpp/.h`, `main/Motion_control.cpp/.h` |
| 10 | `calcU` の `dspm::Mat` 一時行列（毎周期のヒープ確保）をやめ、事前確保済みの `u` に直接計算する | `main/Motion_control.cpp` |

### 結果（2026-10-31）
- #6, #10 の修正を実施（未コミット）
  - #6: `Motion_control::update()` から未使用の `readTemp()` を削除。`readMag()` は `mag_div = round(IMU周波数 / 80Hz)` 回に1回（IMU 500Hz なら 6回に1回 ≒ 83Hz）だけ読む
    - 1周期あたりの I2C トランザクション: 4回（約0.9ms）→ 通常2回（約0.45〜0.5ms）、6回に1回だけ3回
    - accel(0x28〜) と gyro(0x18〜) はレジスタが連続していないため、1回の連続読み出しにはしていない（間の割り込みソースレジスタを読むとフラグがクリアされるため）
    - IMU周期（`IMU_sampling_ms = 2`）は変更していない。1ms 化は #7（制御タスク化）と合わせて検討
  - #10: `calcU` の `-1 * KC * dspm::Mat(...) + 75` などを、`u` に直接書き込む `mat_vec_affine()` に置き換え。毎周期 3〜4 回のヒープ確保がなくなる。計算結果は従来と同じ
- **ビルド・実機は未確認**（ユーザー側で確認予定）

#### 動作確認（2026-10-31）
- #6, #10 はユーザー側でビルド・実機確認済み。4番サーボの挙動はハード側の問題で、ユーザーが修正済み

## 2026-10-31 12:46 改善 #7 開始

- ブランチ: `vert`（起点コミット `8b9b0b8`、#1〜#6・#10 は未コミットのまま）
- ターゲット: ESP32（デュアルコア、Bluedroid / BT コントローラは Core 0 固定、FREERTOS_HZ=1000）

### 対象
| # | 内容 | ファイル |
|---|---|---|
| 7 | IMU / KF / 制御の3つの xTimer コールバックを、gptimer 割り込みで起こす専用の高優先度タスク（Core 1 固定）に移す | `main/main.cpp` |

### 結果（2026-10-31）
- #7 の修正を実施（未コミット）
  - IMU / KF / 制御の3つの xTimer（IMU 2ms・KF 2ms・制御 4ms）を廃止し、専用タスク `control_task`（Core 1 固定、優先度 20）に統合
  - gptimer（1MHz、2000us で自動リロード）のアラーム割り込みから `vTaskNotifyGiveFromISR` でタスクを起こす
  - 1周期の処理順: `motion.update()` → `motion.filterUpdate()` → （2回に1回）`apply_control()`。周期は従来と同じ（IMU/KF 500Hz、制御 250Hz）
  - I2C と gptimer の割り込みを Core 1 に確保するため、`sensor_controller_init()` と gptimer の初期化はタスク内で実行
  - 周期設定は `CONTROL_BASE_PERIOD_US` / `CONTROL_U_DIV` の2つの定数に集約
  - `app_main` から5秒ごとに「1周期の最大処理時間」と「取りこぼした周期数（overrun）」をログ出力
  - テレメトリ（200ms）は従来どおり xTimer のまま
- **ビルド確認済み**（ESP-IDF 5.4.1 / ESP32、Python 3.11 環境）。新規の警告なし
- **実機は未確認**

#### 実機確認（2026-10-31）
- #7 適用後に brownout リセットが発生。原因は電源の容量不足（ユーザーが確認）。ソフト起因ではない
- 電源を変えるとシリアルモニタが使えないため、`control loop` の負荷ログ（max exec / overrun）はまだ未確認

## 2026-10-31 コミット
- #1〜#7, #10 と、作業開始前からあった未コミット変更（getPRY の一次LPF、Servo4 中心値の修正）をまとめてコミット
- `sdkconfig.old`（menuconfig のバックアップ）と `.clangd`（ローカルのエディタ設定）は対象外
