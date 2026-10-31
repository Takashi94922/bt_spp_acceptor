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
        motion.xhat.data,
        motion.PRY_value,
        motion.u.data, 
        motion.KC.data,
        motion.q.data
    );
#endif

// テレメトリタスク実行中フラグ。タスク生成前に立て、タスク終了時に下ろす
static volatile bool bl_telem_busy = false;
//blでIMUデータを送信するのをノンブロッキングでやるためのタスク
void telemetry_task(void* arg){
    //ESP_LOGI("Telem", "sending imu");

    //Throttle PRY x v a u  
    #ifdef COMM_MODE_BT_SPP
    float msg[] = {Thrust->getPercent(),
        motion.PRY_value[0], motion.PRY_value[1], motion.PRY_value[2],
        motion.x(0, 0), motion.x(1, 0), motion.x(2, 0),
        motion.xhat(3, 0), motion.xhat(4, 0), motion.xhat(5, 0),
        motion.xhat(0, 0), motion.xhat(1, 0), motion.xhat(2, 0),
        motion.u(0, 0), motion.u(1, 0), motion.u(2, 0), motion.u(3, 0), motion.u(4, 0)
    };
        
    //ESP_LOGI(TAG, "msg len %d", sizeof(msg));
    bl_comm.sendMsg((char *)msg, sizeof(msg));
    #else
    //ESP_LOGI(TAG, "send Telemetring");
    bl_comm.sendTelemetry();
    #endif

    // 共有ハンドル経由だと後発タスクを消す恐れがあるので、自分自身(NULL)を削除する
    bl_telem_busy = false;
    vTaskDelete(NULL);
}
// 新たな定期的にスマホに送信するタイマーコールバック関数を定義します。
void bl_telemetry_callback(TimerHandle_t xTimer)
{
    // 前回の送信が終わっていなければ今回はスキップ（多重起動防止）
    if(bl_comm.isClientConnecting() && !bl_telem_busy){
        // 新たなタスクを作成してメッセージを送信します。

        //ESP_LOGI("Timer", "telemetring");
        bl_telem_busy = true;
        if (xTaskCreate(telemetry_task, "TelemetryTask", 4096, NULL, 1, NULL) != pdPASS) {
            bl_telem_busy = false;
        }
    }
}
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

// PID設定関数
static void set_pid_gain(void* pid_ptr, uint8_t gain_type, const uint8_t* value) {
    switch(gain_type) {
        case 0: memcpy(&((PID*)pid_ptr)->Kp, value, sizeof(float)); break;
        case 1: memcpy(&((PID*)pid_ptr)->Ki, value, sizeof(float)); break;
        case 2: memcpy(&((PID*)pid_ptr)->Kd, value, sizeof(float)); break;
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
    ESP_LOG_BUFFER_HEX(TAG, msg, msglen);
    
    switch (msg[0]) {
        case 0 ... 4:
            set_servo_pwm(msg[0], msg[1]);
            break;
        case 5:
            motion.ControlMethod = msg[1];
            break;
        case 6 ... 9:
            if (msglen >= 1+sizeof(float)*6) {
                ESP_LOGI(TAG, "%1.2f,%1.2f,%1.2f", motion.KC(0, 3), motion.KC(1, 3), motion.KC(2, 3));
                memcpy(&motion.KC.data[(msg[0] -6)*6], &msg[1], sizeof(float) * 6);
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
        case 11:
            if(msglen >= 6) set_pid_gain(&motion.pitch_pid, msg[1], &msg[2]);
            break;
        case 12:
            if(msglen >= 6) set_pid_gain(&motion.roll_pid, msg[1], &msg[2]);
            break;
        case 13:
            if(msglen >= 6) set_pid_gain(&motion.yaw_pid, msg[1], &msg[2]);
            break;

        //PID目標値の一括セット
        case 14:
            if (msglen >= 1+sizeof(float)*3) {
                motion.setPRYtarget(read_float(&msg[1]), read_float(&msg[1+sizeof(float)]), read_float(&msg[1+sizeof(float)*2]));
            }
            break;
        //PID目標値の個別セット
        case 15:
            if (msglen >= 1+sizeof(float)) motion.pitch_pid.target = read_float(&msg[1]);
            break;
        case 16:
            if (msglen >= 1+sizeof(float)) motion.roll_pid.target = read_float(&msg[1]);
            break;
        case 17:
            if (msglen >= 1+sizeof(float)) motion.yaw_pid.target = read_float(&msg[1]);
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

// タイマー生成と開始を一括で行う関数を追加
TimerHandle_t create_and_start_timer(const char* timer_name, int period_ms, 
                                      TimerCallbackFunction_t callback_func, int timer_id = 0)
{
    TimerHandle_t timer = xTimerCreate(timer_name, pdMS_TO_TICKS(period_ms), pdTRUE, 
                                       (void*)(intptr_t)timer_id, callback_func);
    if (timer == NULL) {
        ESP_LOGE(TAG, "Failed to create %s.", timer_name);
        return NULL;
    }

    if (xTimerStart(timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start %s.", timer_name);
        return NULL;
    }

    ESP_LOGI(TAG, "%s started successfully.", timer_name);
    return timer;
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
    ESP_ERROR_CHECK(bl_comm.begin());

    // IMU・KF・制御は専用タスクで回す（センサ初期化もタスク内で行う）
    if (xTaskCreatePinnedToCore(control_task, "control", 4096, NULL, CONTROL_TASK_PRIO, NULL, CONTROL_TASK_CORE) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task.");
    }
    create_and_start_timer("BL Telemetry", 200, bl_telemetry_callback, 2);

    // 制御ループの負荷を定期的にログ出力する
    for(;;){
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG, "control loop: max exec %" PRIu32 " us / period %d us, overrun %" PRIu32,
                 ctrl_exec_max_us, CONTROL_BASE_PERIOD_US, ctrl_overrun_count);
        ctrl_exec_max_us = 0;
    }
}

