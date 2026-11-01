/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "time.h"
#include "sys/time.h"

#include "driver/mcpwm_timer.h"
#include "driver/mcpwm_types.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_gen.h"
#include "driver/gptimer.h"
#include "esp_timer.h"

#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"

#include "motor.h"
#include "Motion_control.h"
#include "flight_log.h"

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

// 制御ループの周期設定。IMUの周期（姿勢推定）を基本周期とし、制御計算はその整数倍で回す
#define CONTROL_BASE_PERIOD_US 2000   // IMU周期 [us]（2ms = 500Hz）
#define CONTROL_U_DIV          2      // 制御計算は基本周期N回に1回（2ms*2 = 4ms）
#define CONTROL_TASK_CORE      1      // Bluedroid/BTコントローラはCore 0固定なのでCore 1で回す
#define CONTROL_TASK_PRIO      20

// テレメトリ設定。監視用途なので周期は控えめにし、BTと同じCore 0で低優先度に回す
#define TELEMETRY_PERIOD_MS    200
#define LOOP_STATS_PERIOD_MS   1000   // 制御ループ負荷の集計・送信周期
#define TELEMETRY_TASK_CORE    0
#define TELEMETRY_TASK_PRIO    2

// 緊急時（BLE切断・ハートビート途絶）の設定
#define EMERGENCY_THROTTLE     0.0f   // 緊急時のスロットル上限[%]。推力を切る
#define HEARTBEAT_TIMEOUT_MS   1000   // この時間コマンドが届かなければ緊急動作
#define CMD_HEARTBEAT          18     // モニターからの生存通知コマンド（中身は使わない）
#define CMD_GYRO_CALIB         19     // ジャイロのバイアス校正。結果は同じ番号で Command の通知に返す
#define CMD_STATUS             20     // ドローン側の状態。TELEMETRY_PERIOD_MSごとに Command の通知で送る
#define CMD_GAIN_READ          21     // ゲインの読み戻し要求。PIDとKCの各行を同じ番号で Command の通知に返す
#define CMD_KC_ROW4            22     // KCの5行目（S4）の書き込み。1〜4行目は 6〜9 番
#define CMD_LOG_START          23     // 記録の開始 [23, 秒 u8]。記録が終わると Log_Telem で送る
#define CMD_LOG_RESEND         24     // 記録の送り直し [24, 先頭のサンプル番号 u16, 個数 u16]。個数0ならヘッダから全部

// 記録（flight_log.h）の設定
#define LOG_MAX_SECONDS        10     // 1回の記録の上限[s]（メモリが足りなければもっと短くなる）
#define LOG_PACKETS_PER_TICK   15     // テレメトリ周期1回で送る記録のパケット数の上限
#define LOG_PACKET_INTERVAL_MS 10     // 記録のパケットの送信間隔（BLEの送信キューをあふれさせない）
#define LOG_PACKET_MAX         (4 + 46 * 5) // 1パケットの上限[byte]。MTUが大きくてもサンプル5個まで

// CMD_STATUS の flags
#define STATUS_FLAG_FAILSAFE    (1 << 0) // 緊急動作中（BLE切断・ハートビート途絶）
#define STATUS_FLAG_CALIBRATING (1 << 1) // ジャイロ校正中
#define STATUS_FLAG_HEARTBEAT   (1 << 2) // ハートビートの監視が有効
#define STATUS_FLAG_MAG         (1 << 3) // 姿勢推定に地磁気を使っている（モーター停止中）
#define STATUS_FLAG_LOG_REC     (1 << 4) // 記録中
#define STATUS_FLAG_LOG_SEND    (1 << 5) // 記録を送信中

// CMD_GAIN_READ の応答の種類（msg[1]）。KCは1行ずつ GAIN_KIND_KC_ROW0 + 行番号 で送る
#define GAIN_KIND_PID           0
#define GAIN_KIND_KC_ROW0       1

// ジャイロ校正の結果（Command の通知で返す）
enum GyroCalibStatus : uint8_t {
    GYRO_CALIB_OK = 0,        // 校正した
    GYRO_CALIB_MOVING = 1,    // 静止していなかった（最後の測定結果を使った）
    GYRO_CALIB_REJECTED = 2,  // スロットルが0でないので実行しなかった
};

#define TAG "DRONE"

Motor *Thrust, *Servo1, *Servo2, *Servo3, *Servo4;
Motion_control motion;
FlightLog flight_log;

#include "ble_gatt_comm.h"
void (*Ble_comm::command_cb)(uint8_t* data, uint16_t len) = command_cb;
Ble_comm bl_comm(
    motion.sensor_telem,
    motion.PRY_value,
    motion.u.data,
    motion.KC.data,
    motion.q.data
);

// 制御入力を計算してサーボに反映する
static void apply_control()
{
    motion.calcU();
    if(motion.ControlMethod != 0){
        //Thrust->setPWM((motion.u(0, 0)));
        Servo1->setPWM(motion.u(1, 0));
        Servo2->setPWM(motion.u(2, 0));
        Servo3->setPWM(motion.u(3, 0));
        Servo4->setPWM(motion.u(4, 0));
    }
}

// ---- 緊急時の制御動作 ----
// 最後にコマンドを受信した時刻。ハートビートの途絶判定に使う
static volatile TickType_t last_command_tick = 0;
// モニターからハートビートを受信したら監視を有効にする（ハートビート非対応のモニターで誤動作しないように）
static volatile bool heartbeat_armed = false;
// 緊急動作済み。次にコマンドを受信するまで再実行しない
static volatile bool failsafe_active = false;

// 緊急時の制御動作。スロットルをEMERGENCY_THROTTLEまで下げ、姿勢制御は継続する。
// 地上で止まっている機体を回し始めないよう、現在値より上げることはしない
static void emergency_action(const char* reason)
{
    failsafe_active = true;
    float current = Thrust->getPercent();
    if (current > EMERGENCY_THROTTLE) {
        Thrust->setPWM(EMERGENCY_THROTTLE);
    }
    ESP_LOGW(TAG, "emergency (%s): throttle %.0f%% -> %.0f%%", reason, current, Thrust->getPercent());
}

// コマンド受信のたびに呼ぶ。ハートビートの時刻を更新し、緊急動作を解除する
static void on_command_received(uint8_t cmd)
{
    last_command_tick = xTaskGetTickCount();
    if (cmd == CMD_HEARTBEAT) heartbeat_armed = true;
    failsafe_active = false;
}

// BLE切断時（BLEスタックのタスクから呼ばれる）
static void on_ble_disconnected()
{
    heartbeat_armed = false; // 再接続後は再びハートビートを受けてから監視する
    emergency_action("BLE disconnected");
}

// テレメトリタスクから定期的に呼ぶ。ハートビートが途絶えていたら緊急動作
static void check_heartbeat()
{
    if (!heartbeat_armed || failsafe_active || !bl_comm.isClientConnecting()) return;
    if ((xTaskGetTickCount() - last_command_tick) > pdMS_TO_TICKS(HEARTBEAT_TIMEOUT_MS)) {
        emergency_action("heartbeat timeout");
    }
}

// ---- ジャイロ校正 ----
// 校正は制御タスクで行う（I2Cを読むのは制御タスクだけにする）。コマンドでは要求を立てるだけ
static volatile bool gyro_calib_requested = false;
// 校正中（約0.5〜1.5秒、制御は止まる）。この間はスロットルの指令を受け付けない
static volatile bool gyro_calibrating = false;

static void send_gyro_calib_result(uint8_t status)
{
    if (!bl_comm.isClientConnecting()) return;
    // [CMD_GYRO_CALIB, status, bias x/y/z (float, deg/s, IMU座標)]
    char msg[2 + sizeof(float) * 3];
    msg[0] = CMD_GYRO_CALIB;
    msg[1] = status;
    for (int i = 0; i < 3; i++) {
        float b = motion.g_bias[i] * Motion_control::rad2deg;
        memcpy(&msg[2 + sizeof(float) * i], &b, sizeof(float));
    }
    bl_comm.sendMsg(msg, sizeof(msg));
}

// ---- モニターへの状態の返送・ゲインの読み戻し ----
// 読み戻しはコマンドでは要求を立てるだけにし、テレメトリタスクから送る（BLEのコールバック内でまとめて送らない）
static volatile bool gain_read_requested = false;

// ---- 記録 ----
// 開始・送り直しもコマンドでは要求を立てるだけにし、バッファの確保と送信はテレメトリタスクで行う
static volatile uint8_t log_start_seconds = 0;  // 0以外なら開始の要求
static volatile bool log_resend_requested = false;
static volatile uint16_t log_resend_first = 0, log_resend_count = 0;

static int16_t to_fixed(float v, float scale)
{
    float x = v * scale;
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)lrintf(x);
}

// 制御周期ごとに1サンプル記録する（制御タスクから）
static void record_flight_log()
{
    if (!flight_log.isRecording()) return;
    const PID *pids[] = {&motion.pitch_pid, &motion.roll_pid, &motion.yaw_pid};
    FlightLogSample s;
    s.throttle = to_fixed(Thrust->getPercent(), FLIGHT_LOG_SCALE_PERCENT);
    for (int i = 0; i < 3; i++) {
        s.target[i] = to_fixed(pids[i]->target, FLIGHT_LOG_SCALE_ANGLE);
        s.angle[i] = to_fixed(motion.PRY_value[i], FLIGHT_LOG_SCALE_ANGLE);
        s.rate[i] = to_fixed(motion.PRY_rate[i], FLIGHT_LOG_SCALE_RATE);
        s.pid[i][0] = to_fixed(pids[i]->last_p, FLIGHT_LOG_SCALE_PERCENT);
        s.pid[i][1] = to_fixed(pids[i]->last_i, FLIGHT_LOG_SCALE_PERCENT);
        s.pid[i][2] = to_fixed(pids[i]->last_d, FLIGHT_LOG_SCALE_PERCENT);
    }
    for (int i = 0; i < 4; i++) {
        s.servo[i] = to_fixed(motion.u(i + 1, 0), FLIGHT_LOG_SCALE_PERCENT);
    }
    flight_log.record(s);
}

// 記録の開始・送り直しの要求を処理し、送信待ちのパケットを送る（テレメトリタスクから）
static void service_flight_log()
{
    if (log_start_seconds != 0) {
        uint8_t sec = log_start_seconds;
        log_start_seconds = 0;
        if (sec > LOG_MAX_SECONDS) sec = LOG_MAX_SECONDS;
        FlightLogInfo info = {};
        info.period_us = CONTROL_BASE_PERIOD_US * CONTROL_U_DIV;
        info.control_method = motion.ControlMethod;
        motion.getPIDGains(info.pid_gains);
        motion.getPIDTargets(info.targets);
        uint32_t samples = (uint32_t)sec * 1000000u / info.period_us;
        flight_log.start((uint16_t)samples, info);
    }
    if (log_resend_requested) {
        log_resend_requested = false;
        flight_log.requestResend(log_resend_first, log_resend_count);
    }

    if (!bl_comm.isClientConnecting()) return;
    uint8_t pkt[LOG_PACKET_MAX];
    size_t max_len = bl_comm.notifyPayloadMax();
    if (max_len > sizeof(pkt)) max_len = sizeof(pkt);
    for (int i = 0; i < LOG_PACKETS_PER_TICK && flight_log.hasPacket(); i++) {
        size_t len = flight_log.buildPacket(pkt, max_len);
        if (len == 0) {
            ESP_LOGW(TAG, "log packet does not fit in MTU %u", (unsigned)(max_len + 3));
            break;
        }
        if (!bl_comm.sendLog(pkt, len)) break; // 次のテレメトリ周期で送り直す
        flight_log.advance();
        vTaskDelay(pdMS_TO_TICKS(LOG_PACKET_INTERVAL_MS));
    }
}

// [CMD_STATUS, 制御モード, flags, 0, スロットル[%], 目標 pitch/roll/yaw[rad]]（float）
static void send_status()
{
    uint8_t flags = 0;
    if (failsafe_active) flags |= STATUS_FLAG_FAILSAFE;
    if (gyro_calibrating) flags |= STATUS_FLAG_CALIBRATING;
    if (heartbeat_armed) flags |= STATUS_FLAG_HEARTBEAT;
    if (motion.mag_in_use) flags |= STATUS_FLAG_MAG;
    if (flight_log.isRecording() || log_start_seconds != 0) flags |= STATUS_FLAG_LOG_REC;
    if (flight_log.hasPacket()) flags |= STATUS_FLAG_LOG_SEND;

    float values[4];
    values[0] = Thrust->getPercent();
    motion.getPIDTargets(&values[1]);

    char msg[4 + sizeof(values)];
    msg[0] = CMD_STATUS;
    msg[1] = motion.ControlMethod;
    msg[2] = flags;
    msg[3] = 0;
    memcpy(&msg[4], values, sizeof(values));
    bl_comm.sendMsg(msg, sizeof(msg));
}

// [CMD_GAIN_READ, GAIN_KIND_PID, pitch/roll/yaw の Kp Ki Kd]（float 9個）
// [CMD_GAIN_READ, GAIN_KIND_KC_ROW0 + 行, KCの1行]（float 6個）を5行分
static void send_gains()
{
    float pid[9];
    motion.getPIDGains(pid);
    char pid_msg[2 + sizeof(pid)];
    pid_msg[0] = CMD_GAIN_READ;
    pid_msg[1] = GAIN_KIND_PID;
    memcpy(&pid_msg[2], pid, sizeof(pid));
    bl_comm.sendMsg(pid_msg, sizeof(pid_msg));

    constexpr int KC_ROWS = 5, KC_COLS = 6;
    float kc[KC_ROWS * KC_COLS];
    motion.getKC(kc);
    for (int row = 0; row < KC_ROWS; row++) {
        char kc_msg[2 + sizeof(float) * KC_COLS];
        kc_msg[0] = CMD_GAIN_READ;
        kc_msg[1] = GAIN_KIND_KC_ROW0 + row;
        memcpy(&kc_msg[2], &kc[row * KC_COLS], sizeof(float) * KC_COLS);
        bl_comm.sendMsg(kc_msg, sizeof(kc_msg));
    }
}

// 制御ループの負荷計測（app_mainで定期的にログ出力する）
static volatile uint32_t ctrl_overrun_count = 0;  // 処理が間に合わず取りこぼした周期数
static volatile uint32_t ctrl_exec_max_us = 0;    // 1周期の最大処理時間

// gptimerのアラーム割り込み。制御タスクを起こすだけ
static bool IRAM_ATTR control_timer_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)user_ctx, &high_task_woken);
    return high_task_woken == pdTRUE;
}

static void start_control_timer(TaskHandle_t task)
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1us per tick
        .intr_priority = 0,
        .flags = {},
    };
    gptimer_handle_t timer = NULL;
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &timer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = control_timer_on_alarm,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, task));
    ESP_ERROR_CHECK(gptimer_enable(timer));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = CONTROL_BASE_PERIOD_US,
        .reload_count = 0,
        .flags = {.auto_reload_on_alarm = true},
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(timer));
}

static void sensor_controller_init(float IMU_sampleFreq, float Cont_freq);

// IMU読み出し・姿勢推定・制御計算を基本周期ごとに順番に実行するタスク
static void control_task(void* arg)
{
    // I2Cとgptimerの割り込みをこのタスクのコアに確保するため、初期化はタスク内で行う
    sensor_controller_init(1e6f / CONTROL_BASE_PERIOD_US, 1e6f / (CONTROL_BASE_PERIOD_US * CONTROL_U_DIV));
    start_control_timer(xTaskGetCurrentTaskHandle());

    uint32_t u_count = 0;
    for(;;){
        // 通知が2以上溜まっていたら前周期の処理が間に合っていない
        uint32_t pending = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (gyro_calib_requested) {
            gyro_calib_requested = false;
            gyro_calibrating = true;
            bool still = motion.calibrateGyro();
            gyro_calibrating = false;
            send_gyro_calib_result(still ? GYRO_CALIB_OK : GYRO_CALIB_MOVING);
            // 校正中に溜まった周期は取りこぼしとして数えない
            ulTaskNotifyTake(pdTRUE, 0);
            u_count = 0;
            continue;
        }

        if (pending > 1) ctrl_overrun_count = ctrl_overrun_count + (pending - 1);

        int64_t t0 = esp_timer_get_time();
        motion.setMotorRunning(Thrust->getPercent() > 0.0f);
        motion.update();
        motion.updateSensorTelem();
        if (++u_count >= CONTROL_U_DIV) {
            u_count = 0;
            apply_control();
            record_flight_log();
        }
        uint32_t exec_us = (uint32_t)(esp_timer_get_time() - t0);
        if (exec_us > ctrl_exec_max_us) ctrl_exec_max_us = exec_us;
    }
}


// 単一Servo設定関数
static void set_servo_pwm(uint8_t servo_id, float pwm) {
    Motor* servos[] = {Thrust, Servo1, Servo2, Servo3, Servo4};
    if (servo_id < 5) {
        ESP_ERROR_CHECK(servos[servo_id]->setPWM(pwm));
    }
}

// 受信バッファ上のfloatはアライメントされていないのでmemcpyで読む
static float read_float(const uint8_t* p) {
    float v;
    memcpy(&v, p, sizeof(float));
    return v;
}

static void command_cb(uint8_t *msg, uint16_t msglen){
    if (msglen < 2) return;
    on_command_received(msg[0]);
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, msg, msglen, ESP_LOG_DEBUG);
    
    switch (msg[0]) {
        case 0 ... 4:
            if (msg[0] == 0 && gyro_calibrating) break; // 校正中は制御が止まっているので推力を上げない
            set_servo_pwm(msg[0], msg[1]);
            break;
        case 5:
            motion.setControlMethod(msg[1]);
            break;
        // KCの1行（float 6個）。6〜9番が1〜4行目、CMD_KC_ROW4 が5行目
        case 6 ... 9:
        case CMD_KC_ROW4:
            if (msglen >= 1+sizeof(float)*6) {
                int row = (msg[0] == CMD_KC_ROW4) ? 4 : msg[0] - 6;
                motion.setKC(&msg[1], row * 6, 6);
            }
            break;
        case 10: {
            if (msglen < 1+4) break;
            uint8_t valServo[4];
            memcpy(valServo, &msg[1], sizeof(uint8_t)*4);
            for(int i = 0; i < 4; i++) {
                set_servo_pwm(i+1, (float)valServo[i]);
            }
            break;
        }
        // PIDゲイン（11:pitch 12:roll 13:yaw、msg[1]は 0:Kp 1:Ki 2:Kd）
        case 11 ... 13:
            if(msglen >= 6) motion.setPIDGain(msg[0] - 11, msg[1], read_float(&msg[2]));
            break;

        //PID目標値の一括セット
        case 14:
            if (msglen >= 1+sizeof(float)*3) {
                motion.setPRYtarget(read_float(&msg[1]), read_float(&msg[1+sizeof(float)]), read_float(&msg[1+sizeof(float)*2]));
            }
            break;
        //PID目標値の個別セット
        case 15 ... 17:
            if (msglen >= 1+sizeof(float)) motion.setPIDTarget(msg[0] - 15, read_float(&msg[1]));
            break;
        // ハートビート（受信時刻の更新はon_command_receivedで済んでいる）
        case CMD_HEARTBEAT:
            break;
        // 記録の開始・送り直し（処理はテレメトリタスクで行う）
        case CMD_LOG_START:
            if (msg[1] > 0) log_start_seconds = msg[1];
            break;
        case CMD_LOG_RESEND:
            if (msglen >= 5) {
                uint16_t first, count;
                memcpy(&first, &msg[1], 2);
                memcpy(&count, &msg[3], 2);
                log_resend_first = first;
                log_resend_count = count;
                log_resend_requested = true;
            }
            break;
        // ゲインの読み戻し（送信はテレメトリタスクで行う）
        case CMD_GAIN_READ:
            gain_read_requested = true;
            break;
        // ジャイロ校正。校正中は制御が止まるので、スロットル0のとき（地上で静止）だけ受け付ける
        case CMD_GYRO_CALIB:
            if (Thrust->getPercent() > 0.0f) {
                ESP_LOGW(TAG, "gyro calibration rejected: throttle is not 0");
                send_gyro_calib_result(GYRO_CALIB_REJECTED);
            } else {
                gyro_calib_requested = true;
            }
            break;
        default: {   
            ESP_LOGI(TAG, "Unknown command.");
            if (bl_comm.isClientConnecting()) {
                bl_comm.sendMsg("Unknown command.", sizeof("Unknown command.") - 1);
            }
        }
    }
}

static void sensor_controller_init(float IMU_sampleFreq, float Cont_freq)
{
    i2c_master_bus_config_t bus_conf = {
        .i2c_port = -1,
        .sda_io_num = GPIO_NUM_18,
        .scl_io_num = GPIO_NUM_19,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 4,
        .intr_priority = 3,
        .flags{
            .enable_internal_pullup = false,
        },
    };
    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_conf, &bus_handle));

    motion.begin(IMU_sampleFreq, Cont_freq, bus_handle);
}

// Servo生成・初期化関数
static void init_servo(Motor** servo_ptr, gpio_num_t gpio, mcpwm_oper_handle_t oper, 
                       int min_us, int max_us, float center_pulse) {
    *servo_ptr = new Motor(gpio, oper, min_us, max_us);
    (*servo_ptr)->setCenterPulse(center_pulse);
    (*servo_ptr)->begin();
    (*servo_ptr)->setPWM(50);
}

static void pwm_init(){
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .intr_priority = 2,
    };
    mcpwm_timer_handle_t timer = NULL;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_operator_config_t operator_config = {.group_id = 0};
    mcpwm_oper_handle_t oper = NULL, operServo = NULL, operServo2 = NULL;
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operServo));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operServo, timer));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operServo2));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operServo2, timer));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    Thrust = new Motor(GPIO_NUM_21, oper, 1000, 2000);
    Thrust->begin();
    Thrust->setPWM(0);

    init_servo(&Servo1, GPIO_NUM_4, operServo, 900, 2100, 50.0f);
    init_servo(&Servo2, GPIO_NUM_16, operServo, 900, 2100, 50.0f);
    init_servo(&Servo3, GPIO_NUM_22, operServo2, 900, 2100, 50.0f);
    init_servo(&Servo4, GPIO_NUM_17, operServo2, 900, 2100, 50.0f);

    if(Thrust == NULL || Servo1 == NULL || Servo2 == NULL || Servo3 == NULL || Servo4 == NULL) {
        ESP_LOGE(TAG, "Motor or Servo creation failed");
    }
}

// スマホへの定期送信タスク。制御(Core 1)に影響しないようCore 0・低優先度で常駐させる
static void telemetry_task(void* arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t stats_count = 0;
    uint32_t log_count = 0;
    for(;;){
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));

        check_heartbeat();

        service_flight_log();
        if (bl_comm.isClientConnecting()) {
            // 記録の送信中は帯域を空けるため、通常のテレメトリを止める（状態は送る）
            if (!flight_log.hasPacket()) bl_comm.sendTelemetry();
            send_status();
            if (gain_read_requested) {
                gain_read_requested = false;
                send_gains();
            }
        }

        // 制御ループの負荷はLOOP_STATS_PERIOD_MSごとに集計して送る
        if (++stats_count < LOOP_STATS_PERIOD_MS / TELEMETRY_PERIOD_MS) continue;
        stats_count = 0;

        loop_stats_t stats = {
            .max_exec_us = ctrl_exec_max_us,
            .overrun = ctrl_overrun_count,
            .period_us = CONTROL_BASE_PERIOD_US,
        };
        ctrl_exec_max_us = 0;
        bl_comm.sendLoopStats(stats);

        // シリアルにも5回に1回だけ出す
        if (++log_count >= 5) {
            log_count = 0;
            ESP_LOGI(TAG, "control loop: max exec %" PRIu32 " us / period %" PRIu32 " us, overrun %" PRIu32,
                     stats.max_exec_us, stats.period_us, stats.overrun);
        }
    }
}

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    pwm_init();
    
    bl_comm.setCommandCb(command_cb);
    bl_comm.setDisconnectCb(on_ble_disconnected);
    // KCの一括書き込み（ContGain_Upd）も制御計算と排他して反映する
    bl_comm.setGainUpdateCb([](const uint8_t* data, uint16_t len) {
        motion.setKC(data, 0, len / sizeof(float));
    });
    ESP_ERROR_CHECK(bl_comm.begin());

    // IMU・姿勢推定・制御は専用タスクで回す（センサ初期化もタスク内で行う）
    if (xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, CONTROL_TASK_PRIO, NULL, CONTROL_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task.");
    }
    if (xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, TELEMETRY_TASK_PRIO, NULL, TELEMETRY_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create telemetry task.");
    }
}

