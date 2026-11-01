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

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

// 制御ループの周期設定。IMU/KFの周期を基本周期とし、制御計算はその整数倍で回す
#define CONTROL_BASE_PERIOD_US 2000   // IMU/KF周期 [us]（2ms = 500Hz）
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

// CMD_STATUS の flags
#define STATUS_FLAG_FAILSAFE    (1 << 0) // 緊急動作中（BLE切断・ハートビート途絶）
#define STATUS_FLAG_CALIBRATING (1 << 1) // ジャイロ校正中
#define STATUS_FLAG_HEARTBEAT   (1 << 2) // ハートビートの監視が有効

// CMD_GAIN_READ の応答の種類（msg[1]）。KCは1行ずつ GAIN_KIND_KC_ROW0 + 行番号 で送る
#define GAIN_KIND_PID           0
#define GAIN_KIND_KC_ROW0       1

// ジャイロ校正の結果（Command の通知で返す）
enum GyroCalibStatus : uint8_t {
    GYRO_CALIB_OK = 0,        // 校正した
    GYRO_CALIB_MOVING = 1,    // 静止していなかった（最後の測定結果を使った）
    GYRO_CALIB_REJECTED = 2,  // スロットルが0でないので実行しなかった
};

#define TAG "ESP_SPP_DRONE"

//#define COMM_MODE_BT_SPP

Motor *Thrust, *Servo1, *Servo2, *Servo3, *Servo4;
Motion_control motion;

#ifdef COMM_MODE_BT_SPP
    #include "bl_comm.h"
    Bl_comm bl_comm;
    void (*Bl_comm::command_cb)(uint8_t* data, uint16_t len) = command_cb; // or assign it to a valid function
    uint32_t Bl_comm::clientHandle = 0;
#else
    #include "ble_gatt_comm.h"
    void (*Ble_comm::command_cb)(uint8_t* data, uint16_t len) = command_cb;
    Ble_comm bl_comm(
        motion.sensor_telem,
        motion.PRY_value,
        motion.u.data, 
        motion.KC.data,
        motion.q.data
    );
#endif

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

// [CMD_STATUS, 制御モード, flags, 0, スロットル[%], 目標 pitch/roll/yaw[rad]]（float）
static void send_status()
{
    uint8_t flags = 0;
    if (failsafe_active) flags |= STATUS_FLAG_FAILSAFE;
    if (gyro_calibrating) flags |= STATUS_FLAG_CALIBRATING;
    if (heartbeat_armed) flags |= STATUS_FLAG_HEARTBEAT;

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

// IMU読み出し・KF・制御計算を基本周期ごとに順番に実行するタスク
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
        motion.update();
        motion.filterUpdate();
        if (++u_count >= CONTROL_U_DIV) {
            u_count = 0;
            apply_control();
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
        case 6 ... 9:
            if (msglen >= 1+sizeof(float)*6) {
                motion.setKC(&msg[1], (msg[0] - 6) * 6, 6);
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
                bl_comm.sendMsg("Unknown command.", 15);
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
    //motion.correctInitValue(100);
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

        if (bl_comm.isClientConnecting()) {
            //Throttle PRY x v a u
            #ifdef COMM_MODE_BT_SPP
            float msg[] = {Thrust->getPercent(),
                motion.PRY_value[0], motion.PRY_value[1], motion.PRY_value[2],
                motion.x(0, 0), motion.x(1, 0), motion.x(2, 0),
                motion.sensor_telem[3], motion.sensor_telem[4], motion.sensor_telem[5],
                motion.sensor_telem[0], motion.sensor_telem[1], motion.sensor_telem[2],
                motion.u(0, 0), motion.u(1, 0), motion.u(2, 0), motion.u(3, 0), motion.u(4, 0)
            };
            bl_comm.sendMsg((char *)msg, sizeof(msg));
            #else
            bl_comm.sendTelemetry();
            send_status();
            if (gain_read_requested) {
                gain_read_requested = false;
                send_gains();
            }
            #endif
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
        #ifndef COMM_MODE_BT_SPP
        bl_comm.sendLoopStats(stats);
        #endif

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
    #ifndef COMM_MODE_BT_SPP
    bl_comm.setDisconnectCb(on_ble_disconnected);
    // KCの一括書き込み（ContGain_Upd）も制御計算と排他して反映する
    bl_comm.setGainUpdateCb([](const uint8_t* data, uint16_t len) {
        motion.setKC(data, 0, len / sizeof(float));
    });
    #endif
    ESP_ERROR_CHECK(bl_comm.begin());

    // IMU・KF・制御は専用タスクで回す（センサ初期化もタスク内で行う）
    if (xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, CONTROL_TASK_PRIO, NULL, CONTROL_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task.");
    }
    if (xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, TELEMETRY_TASK_PRIO, NULL, TELEMETRY_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create telemetry task.");
    }
}

