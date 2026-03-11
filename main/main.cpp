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

#include "ESP32_i2c_LSM9DS1.h"
#include "MadgwickAHRS.h"

#include "motor.h"
#include "Motion_control.h"

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

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
        motion.KC.data
    );
#endif

TaskHandle_t bl_telem_handle_t = NULL;
//blでIMUデータを送信するのをノンブロッキングでやるためのタスク
void telemetry_task(){
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

    vTaskDelete(bl_telem_handle_t); // タスクを削除します。
}
// 新たな定期的にスマホに送信するタイマーコールバック関数を定義します。
void bl_telemetry_callback(TimerHandle_t xTimer)
{
    if(bl_comm.isClientConnecting()){
        // 新たなタスクを作成してメッセージを送信します。

        //ESP_LOGI("Timer", "telemetring");
        xTaskCreate( (TaskFunction_t)telemetry_task, "TelemetryTask", 4096, NULL, 1, &bl_telem_handle_t);
    }
}
// IMU用タイマーコールバック関数を定義します。
void IRAM_ATTR timer_callback(TimerHandle_t xTimer)
{
    //ESP_LOGI("Timer", "reading.. imu");
    motion.update();
}

// 制御用タイマーコールバック関数を定義します。
void IRAM_ATTR timerU_callback(TimerHandle_t xTimer)
{
    //ESP_LOGI("Timer", "calculating.. u");
    motion.calcU();
    if(motion.ControlMethod != 0){
        //Thrust->setPWM((motion.u(0, 0)));
        Servo1->setPWM(motion.u(1, 0));
        Servo2->setPWM(motion.u(2, 0));
        Servo3->setPWM(motion.u(3, 0));
        Servo4->setPWM(motion.u(4, 0));
    }
}
// 制御用タイマーコールバック関数を定義します。
void IRAM_ATTR timerKF_callback(TimerHandle_t xTimer)
{
    //ESP_LOGI("Timer", "calculating.. u");
    motion.filterUpdate();
}


// 単一Servo設定関数
static void set_servo_pwm(uint8_t servo_id, float pwm) {
    Motor* servos[] = {Thrust, Servo1, Servo2, Servo3, Servo4};
    if (servo_id < 5) {
        ESP_ERROR_CHECK(servos[servo_id]->setPWM(pwm));
    }
}

// PID設定関数
static void set_pid_gain(void* pid_ptr, uint8_t gain_type, const float* value) {
    switch(gain_type) {
        case 0: memcpy(&((PID*)pid_ptr)->Kp, value, sizeof(float)); break;
        case 1: memcpy(&((PID*)pid_ptr)->Ki, value, sizeof(float)); break;
        case 2: memcpy(&((PID*)pid_ptr)->Kd, value, sizeof(float)); break;
    }
}

static void command_cb(uint8_t *msg, uint16_t msglen){
    if (msglen < 2) return;
    ESP_LOG_BUFFER_HEX(TAG, msg, msglen);
    
    switch (msg[0]) {
        case 0 ... 4:
            set_servo_pwm(msg[0], (float)msg[1]);
            break;
        case 5:
            motion.ControlMethod = msg[1];
            break;
        case 10: {
            uint8_t valServo[4];
            memcpy(valServo, &msg[1], sizeof(uint8_t)*4);
            for(int i = 0; i < 4; i++) {
                set_servo_pwm(i+1, (float)valServo[i]);
            }
            break;
        }
        case 11:
            if(msglen >= 6) set_pid_gain(&motion.pitch_pid, msg[1], (float*)&msg[2]);
            break;
        case 12:
            if(msglen >= 6) set_pid_gain(&motion.roll_pid, msg[1], (float*)&msg[2]);
            break;
        case 13:
            if(msglen >= 6) set_pid_gain(&motion.yaw_pid, msg[1], (float*)&msg[2]);
            break;
        default:
            if (msg[0] > 6 && msg[0] < 10 && msglen > 1+sizeof(float)*6) {
                ESP_LOGI(TAG, "%1.2f,%1.2f,%1.2f", motion.KC(0, 3), motion.KC(1, 3), motion.KC(2, 3));
                memcpy(&motion.KC.data[(msg[0] -6)*6], &msg[1], sizeof(float) * 6);
                ESP_LOGI(TAG, "%1.2f,%1.2f,%1.2f", motion.KC(0, 3), motion.KC(1, 3), motion.KC(2, 3));
            } else {
                ESP_LOGI(TAG, "Unknown command.");
                if (bl_comm.isClientConnecting()) {
                    bl_comm.sendMsg("Unknown command.", 15);
                }
            }
    }
}

static void i2c_master_init(float sampleFreq)
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

    motion.begin(sampleFreq, bus_handle);
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

    init_servo(&Servo1, GPIO_NUM_4, operServo, 900, 2100, 44.0f);
    init_servo(&Servo2, GPIO_NUM_16, operServo, 900, 2100, 56.0f);
    init_servo(&Servo3, GPIO_NUM_22, operServo2, 900, 2100, 56.0f);
    init_servo(&Servo4, GPIO_NUM_17, operServo2, 900, 2100, 44.0f);

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

    const int IMU_sampling_ms = 4;
    i2c_master_init(1000/IMU_sampling_ms);
    // タイマーを作成し、コールバック関数を設定します。
    TimerHandle_t timer = create_and_start_timer("IMU Timer", IMU_sampling_ms, timer_callback, 1);

    const int CalcU_sampling_ms = 100;
    TimerHandle_t timerU = create_and_start_timer("CalcU Timer", CalcU_sampling_ms, timerU_callback, 1);

    //kalman filter用たいまー
    const int CalcKF_sampling_ms = 10;
    TimerHandle_t timerKF = create_and_start_timer("CalcKF Timer", CalcKF_sampling_ms, timerKF_callback, 1);

    // 新たなタイマーを作成し、コールバック関数を設定します。
    TimerHandle_t bl_telemetry = create_and_start_timer("BL Telemetry", 200, bl_telemetry_callback, 2);

    // タスクをブロックします。
    vTaskDelay(portMAX_DELAY);
}

