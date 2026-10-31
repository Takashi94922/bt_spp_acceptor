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

## 2026-10-31 14:11 改善 E1, E6+M5, M6, PID送信先チェック 開始

- ESP側: ブランチ `vert`、起点コミット `06181f5`
- モニター側: 起点コミット `feaf0ba`（未コミット変更なし）

### 対象
| # | 内容 | 側 |
|---|---|---|
| E1 | BLE 切断時にスロットルを緊急時の値まで下げる（姿勢制御は継続） | ESP |
| E6 | ハートビート監視。一定時間コマンドが届かなければ E1 と同じ緊急動作 | ESP |
| M5 | ハートビートの定期送信 | モニター |
| M6 | 「Velocity」と表示していたジャイロ角速度のラベルを修正 | モニター |
| (新) | PID ゲインの送信先が未選択のときは送信しない | モニター |

- 方針: 緊急時の制御動作は ESP 側で1つの関数に切り出し、E1・E6 の両方から呼ぶ

### 結果（2026-10-31）
#### ESP側（未コミット）
- 緊急時の制御動作を `emergency_action(reason)` に切り出し。スロットルを `EMERGENCY_THROTTLE`（30%）まで下げ、姿勢制御は継続。現在値が 30% 未満なら上げない（地上で回り出さないように）
- E1: `Ble_comm::setDisconnectCb()` を追加し、切断時に `on_ble_disconnected()` → `emergency_action("BLE disconnected")`
- E6: コマンド受信のたびに `on_command_received()` で受信時刻を更新。コマンド 18 をハートビート（no-op）として追加。テレメトリタスク（200ms周期）の `check_heartbeat()` で、`HEARTBEAT_TIMEOUT_MS`（1000ms）以上コマンドが届かなければ `emergency_action("heartbeat timeout")`
  - ハートビートを一度受信するまでは監視しない（ハートビート非対応のモニターで誤動作しないように）
  - 緊急動作は1回だけ。次にコマンドを受信したら解除（スロットルは自動では戻らない）
- ビルド確認済み、新規の警告なし

#### モニター側（未コミット）
- M5: 接続中は `BleService` が 250ms ごとにハートビート（`18, seq`）を送信。書き込みキュー経由なので他の送信とは重ならない。切断時に停止
- Trip ボタンも「現在値より上げない」に修正（低スロットル時に押すと 30% に上がってしまっていた）
- M6: 「Velocity」ラベルを「Gyro [rad/s]」に修正
- (新): PID ゲインの送信先が未選択なら警告を出して送信しない
- Windows ターゲットでビルド確認済み（起動中の DroneMonitor.exe がロックしていたため、出力先を別フォルダにして確認）
- 実機は未確認
- 変更: Trip ボタン（操縦者による緊急停止）はスロットル 0% を送るように変更（定数 `TRIP_THROTTLE`）。制御モードは変更しない。通信途絶時のドローン側の緊急動作（30%）とは別扱い（ユーザー判断）。Windows ターゲットでビルド確認済み

#### 実機確認（2026-10-31）
- モニターを bin に再ビルドした後、ユーザーが動作を確認（OK）

## 2026-10-31 コミット
- ESP側: 緊急動作の関数化（`emergency_action`）、E1（切断時）、E6（ハートビート監視）をコミット
- モニター側: M5（ハートビート送信）、M6（ラベル修正）、PID送信先チェック、Trip をスロットル0% に変更、をコミット

## 2026-10-31 14:37 モニターUI改修 A・B・E 開始（Windows優先、Android対応は後回し）

- モニター側: 起点コミット `51b5b26`
### 対象
| # | 内容 |
|---|---|
| A | 操縦画面の作り直し（サーボのラベルと数値表示、スロットルをスライダーでも操作可能に、TRIP を大きな独立ボタンに、上部に状態バー、機体タイプの初期値を V字に・未実装の PRY 選択を削除） |
| B | 制御モードを None / MIMO / PID の3ボタンにし、現在のモードを強調表示 |
| E | タブを「Flight（操縦＋テレメトリ）」と「Gains（ゲイン設定）」の2つに整理 |

### 結果（2026-10-31）モニターUI改修 A・B・E（未コミット）
- A 操縦画面（`HomeView`）
  - スロットル: 大きな数値表示＋スライダーで操作可能に（ゲームパッドでスロットル操作中はスライダーと Stepper を無効化）
  - TRIP: 140×80 の赤い独立ボタンに
  - サーボ S1〜S4: 名前とスライダーと数値を1行ずつ
  - 目標姿勢（PID）: 枠でまとめた
  - 機体タイプ: 初期値を V字 にし、ゲームパッドにも起動時に反映。未実装だった「PRY」を削除
  - ゲームパッド状態の表示を「姿勢: ON/OFF（Aで切替） スロットル: ON/OFF（Bで切替）」に
  - 状態バー（`StatusBarView`、新規）: 接続状態（未接続／スキャン中／接続中＋デバイス名）、制御モード、制御ループ負荷を常時表示
  - 接続ボタン: スキャン中は「スキャン中…」と表示して無効化
- B 制御モード（`ControlDataView`）: Picker＋トグルボタンをやめ、None / MIMO / PID の3ボタンに。現在のモードを強調表示、未接続時は無効
  - テレメトリ表示も枠と単位付きで整理（姿勢 [deg]、Accel [m/s²]、Gyro [rad/s]、u を Thrust/S1〜S4 の名前付きで）
- E タブ: 「Flight（状態バー＋操縦＋制御モード/テレメトリ）」と「Gains（PID・KC ゲイン設定、`GainView`/`GainPage` 新規）」の2つに整理。`HomePage`・`ControlDataPage` は削除
- `BleService`: 接続状態 `State`（Disconnected/Connecting/Connected）と `ConnectionStateChanged` イベントを追加
- Windows ターゲットで bin にビルド確認済み（新規の警告は既存コードと同じパターンのもののみ）。見た目・操作感は未確認
- Android 版は未対応（`AndroidGamepadHandler` はスロットルスライダーの有効/無効切替をしていない）

## 2026-10-31 モニターUI改修 C・D 開始（A・B・E は未コミットのまま続けて作業）
| # | 内容 |
|---|---|
| C | テレメトリ表示の整理（姿勢を大きな数字で、センサ値と制御入力を表形式で） |
| D | ゲイン編集の改善（PID は 3軸×P/I/D の表で一括送信、KC は 5×6 の表で入力） |

### 結果（2026-10-31）モニターUI改修 C・D（未コミット）
- C テレメトリ（`ControlDataView`）: 姿勢を Pitch/Roll/Yaw の大きな数字（28pt）で表示。センサ値は Accel/Gyro × X/Y/Z の表、制御入力 u は Thrust/S1〜S4 の表に
- D ゲイン（`GainView`）
  - PID: 3軸×P/I/D の入力表。空欄は送らず、前回送信から変わった項目だけをコマンド11〜13で順に送信。読めない値が1つでもあれば何も送らない
  - KC: 5×6 の入力表（行 Thrust/S1〜S4、列 x1〜x6）。30個そろったときだけ ContGain_Upd にまとめて送信
  - 入力欄の色: 黄＝未送信の変更、赤＝数値として読めない。薄字（Placeholder）はファームウェア既定値
  - 「既定値で埋める」ボタン（PID・KC）、「クリップボードから貼り付け」（KC、カンマ・空白・改行区切りの30個）
  - 既定値は `Motion_control.h` の値をモニター側にも持っている（ファームウェアを変えたら両方直す必要あり）
- Windows ターゲットで bin にビルド確認済み（新規の警告は既存と同じ DisplayAlert のパターンのみ）。見た目・操作感は未確認
- 追加: Gains タブを左右2列にし、右側に制御モード・姿勢・制御入力（`ControlDataView`、センサ枠は `ShowSensors="False"` で非表示）を表示。姿勢を見ながらゲイン調整できるように（ユーザー要望）
  - アプリ起動中のため bin は未更新。別フォルダへのビルドでコンパイルのみ確認
  - アプリ終了後に bin を再ビルド（DroneMonitor.dll 15:03、Gains タブ変更を含むことを確認）

## 2026-10-31 モニター画面構成の変更（共通パネル化）開始
- 方針（ユーザー合意）: 上部に状態バー（TRIP は中央）、左側だけ Flight / Gains で切り替え、右側に制御モード・姿勢・センサ・制御入力の共通パネルを固定
- Shell のタブをやめて1ページ（MainPage）内で左側を切り替える。DualPage・GainPage は MainPage に統合

### 結果（2026-10-31）画面構成の変更（未コミット）
- `MainPage`（新規）: 上に状態バー、左に Flight / Gains 切り替え（両方を置いて IsVisible で切り替えるので入力内容は保持）、右に共通パネル（`ControlDataView`: 制御モード・姿勢・センサ・制御入力）。左右の幅は 3:2
- 状態バー（`StatusBarView`）: 中央に TRIP（180×64、赤）を配置。右側にスロットル指令値を追加。TRIP は `TripRequested` イベント経由で `HomeView.TripAsync()` を呼ぶ（動作は従来と同じ、スロットル0%）
- `HomeView`: TRIP ボタンを削除し、処理を `TripAsync()` として公開。スロットル指令値の変化を `ThrottleChanged` で通知
- Shell: タブをやめて MainPage 1枚に。`DualPage`・`GainPage` は削除
- Windows ターゲットで bin にビルド確認済み（15:12）。見た目・操作感は未確認

## 2026-10-31 コミット
- モニター側: UI 改修 A〜E、Gains での表示、共通パネル型レイアウト（MainPage）をまとめてコミット
- ESP側: 作業ログのみ（コード変更なし）
