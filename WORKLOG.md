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

## 2026-10-31 13:25 改善 E2〜E5（ESP側）, M1〜M4・M7・M8（モニター側）開始

- ESP側: ブランチ `vert`、起点コミット `f71af46`
- モニター側: `D:\YsProject\drone\DroneMonitor`（別リポジトリ）、起点コミット `dd73d8a`
  - 開始時点の未コミット変更（ユーザー作業中、そのまま保持）: `HomeView.xaml(.cs)`（緊急停止ボタン追加、送信周期 16→50ms 等）、`AndroidGamepadHandler.cs`、`WindowsGamepadHandler.cs`

### 対象
| # | 内容 | 側 |
|---|---|---|
| E2 | テレメトリを Core 0 固定・低優先度の常駐タスク1本に（周期 200ms のまま、毎回のタスク生成をやめる） | ESP |
| E3 | 制御ループ負荷（max exec / overrun / 周期）を送るキャラクタリスティック 0xFF07 を追加、1秒ごとに送信 | ESP |
| E4 | `isClientConnecting()` が切断後も true を返すのを修正 | ESP |
| E5 | BLE 受信ごとの HEX ログ出力をやめる | ESP |
| M1 | `SendSliderValueAsync` で 2 バイト配列に float を書き込んで例外になるバグを修正 | モニター |
| M2 | BLE 書き込みを1本のキューに直列化（タイマーの再入防止も） | モニター |
| M3 | 緊急停止ボタンでスロットル0・制御モード0を即時送信 | モニター |
| M4 | 制御ループ負荷の表示欄を追加 | モニター |
| M7 | `ControlDataPage` を `ControlDataView` をホストする薄いラッパーにして重複を解消 | モニター |
| M8 | Shift-JIS のファイル（BleService.cs ほか）を UTF-8 (BOM付き) に統一 | モニター |

### 結果（2026-10-31）
#### ESP側（未コミット）
- E2: テレメトリを xTimer＋毎回のタスク生成から、常駐タスク `telemetry_task`（Core 0、優先度 2、`vTaskDelayUntil` で 200ms 周期）に変更。未使用になった `create_and_start_timer` は削除
- E3: キャラクタリスティック 0xFF07（Read/Notify、`loop_stats_t` = uint32 x3: max_exec_us, overrun累計, period_us）を追加。テレメトリタスクが1秒ごとに集計して送信。シリアルへのログは5秒ごと
- E4: `isClientConnecting()` を `conn_id != 0xFFFF` でも判定するよう修正
- E5: BLE 書き込み受信時と `command_cb` の HEX ログを DEBUG レベルに変更（既定の INFO では出ない）
- ビルド確認済み（ESP-IDF 5.4.1 / ESP32）、新規の警告なし

#### モニター側（未コミット、`D:\YsProject\drone\DroneMonitor`）
- M8: Shift-JIS だった `BleService.cs`, `Platforms/Android/JoystickView.xaml.cs`, `Views/DualPage.xaml.cs`, `Views/ControlDataPage.xaml.cs` を UTF-8 (BOM付き) に変換（中身はコメントの文字コードのみ）
- M2: `BleService.WriteAsync(key, data)` を追加し、書き込みと通知の開始・停止を `SemaphoreSlim` で直列化。`HomeView` の定期送信は前回の送信中ならスキップ。`ControlDataView` の書き込みもすべて経由させた
- M1: PID 目標値の送信で専用の 5 バイト配列を使うよう修正。定期送信の例外は捕捉してログに出す
- M3: 緊急停止ボタンで、定期送信の残りを打ち切ったうえでスロットル0（`00 00`）と制御モード0（`05 00`）を即時送信。`BleService.IsControlBySelf` の変更を `ControlStateChanged` イベントで通知し、制御画面のボタン表示も連動
- M4: 制御画面の最下段に負荷表示を追加（最大処理時間 / 周期 / 使用率 / 取りこぼし）。取りこぼしが増えたか使用率 80% 以上で赤表示。0xFF07 が無い古いファームウェアでは「未対応」と表示
- M7: `ControlDataPage` を `ControlDataView` をホストするだけのラッパーに変更（重複していた約 250 行を削除）
- Windows ターゲットでビルド確認済み（.NET SDK 9.0.302、新規の警告なし）。Android ターゲットは maui-android ワークロードが無くビルド未確認
- 実機（ESP・スマホ）での動作は未確認

#### 変更（2026-10-31）
- M3 の緊急停止時スロットルを 0% → 30% に変更（0% だと墜落するため）。`HomeView.xaml.cs` の定数 `EMERGENCY_THROTTLE` で調整可能。制御モード0の送信はそのまま
- Windows ターゲットでビルド確認済み
- 緊急停止で制御モード0（姿勢制御オフ）を送るのをやめた。推力を残して降下させるため、姿勢制御は継続する（ユーザー判断）。Windows ターゲットでビルド確認済み

#### 実機確認（2026-10-31）
- ESP・モニターとも動作を確認（ユーザー）
- 制御ループ負荷: 最大処理時間 1058µs / 周期 2000µs（約53%）、取りこぼし 0
  - 見積もり（約 0.5ms）より大きい。磁気センサを読む周期（6回に1回、I2C 3回）や I2C ドライバのオーバーヘッドが最大値に効いていると考えられる
  - このままでは IMU 周期 1ms（1kHz）化は不可。行うなら SPI 化や読み出しのさらなる削減が必要

## 2026-10-31 コミット
- ESP側（このリポジトリ）: E2〜E5 をコミット。`sdkconfig.old` と `.clangd` は対象外
- モニター側（`D:\YsProject\drone\DroneMonitor`）: M1〜M4・M7・M8、緊急停止スロットル30%（制御継続）と、作業開始前からあった未コミット変更（緊急停止ボタン追加、送信周期 16→50ms、ゲームパッド処理の変更）をまとめてコミット
