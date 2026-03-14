/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_gatts_api.h"
#include "esp_gap_ble_api.h"

#define GATTS_TABLE_TAG "BLE GATT"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "ESP_DRONE"
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,
    IDX_CHAR_CFG_B,

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,
    IDX_CHAR_CFG_C,

    IDX_CHAR_D,
    IDX_CHAR_VAL_D,
    IDX_CHAR_CFG_D,
    
    IDX_CHAR_E,
    IDX_CHAR_VAL_E,
    IDX_CHAR_CFG_E,

    IDX_CHAR_F,
    IDX_CHAR_VAL_F,
    IDX_CHAR_CFG_F,
    IDX_NB,
};
typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
    uint16_t target_handle;
} prepare_type_env_t;

typedef struct {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
} gatts_profile_inst;

typedef struct {
    uint16_t cfg_handle;
    uint16_t val_handle;
    uint8_t *value_ptr;
    size_t value_len;
    const char *label;
} notify_target_t;

/* Service */
constexpr uint16_t GATTS_SERVICE_UUID      = 0x00FF;
constexpr uint16_t GATTS_CHAR_UUID_Xhat_Telem = 0xFF01;
constexpr uint16_t GATTS_CHAR_UUID_PRY_Telem = 0xFF02;
constexpr uint16_t GATTS_CHAR_UUID_contU_TelemWrite = 0xFF03;
constexpr uint16_t GATTS_CHAR_UUID_ContGain_Upd = 0xFF04;
constexpr uint16_t GATTS_CHAR_UUID_Command = 0xFF05;
constexpr uint16_t GATTS_CHAR_UUID_Quaternion_Telem = 0xFF06;

constexpr uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
constexpr uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
constexpr uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
constexpr uint8_t char_prop_read                = ESP_GATT_CHAR_PROP_BIT_READ;
constexpr uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
constexpr uint8_t char_prop_read_notify         = ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_READ;
constexpr uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

class Ble_comm {
public:
    Ble_comm(float *xhat_value_p,float* PRY_value_p, float* controlU_p, float * controlGain_p, float *q_p);
    esp_err_t begin();

    bool isClientConnecting(){
        return (profile_tab[PROFILE_APP_IDX].gatts_if != ESP_GATT_IF_NONE);
    }
    void setCommandCb(void (*command_cb_p)(uint8_t* data, uint16_t len)) {
        Ble_comm::command_cb = command_cb_p;
    }
    static bool isConnecting, isWriting;
    void sendMsg(char *msg, unsigned char len);
    void sendTelemetry();
private:
    // BLE advertising/scan data
    static constexpr uint8_t RAW_ADV_DATA[21] = {
        0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,
        0x02, ESP_BLE_AD_TYPE_TX_PWR, 0xEB,
        0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xFF, 0x00,
        0x0A, ESP_BLE_AD_TYPE_NAME_CMPL, 'E', 'S', 'P', '_', 'D', 'R', 'O', 'N', 'E'
    };
    static constexpr uint8_t RAW_SCAN_RSP_DATA[10] = {
        0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,
        0x02, ESP_BLE_AD_TYPE_TX_PWR, 0xEB,
        0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xFF, 0x00
    };
    static constexpr char DEVICE_NAME[10] = {'E', 'S', 'P', '_', 'D', 'R', 'O', 'N', 'E', '\0'};

    static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
    static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

    static void prepare_write_event_env(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param, prepare_type_env_t *prepare_write_env);
    static void exec_write_event_env(esp_ble_gatts_cb_param_t *param, prepare_type_env_t *env_list[], size_t env_count);

    // BLE parameters and buffers
    static constexpr uint8_t cfg_val_on[2] = {0x01, 0x00};
    static constexpr esp_ble_adv_params_t adv_params = {
        .adv_int_min        = 0x20,
        .adv_int_max        = 0x40,
        .adv_type           = ADV_TYPE_IND,
        .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
        .channel_map        = ADV_CHNL_ALL,
        .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };
    static uint8_t adv_config_done;
    static gatts_profile_inst profile_tab[PROFILE_NUM];
    
    static esp_gatts_attr_db_t gatt_db[IDX_NB];
    static notify_target_t notify_targets[6];
    static notify_target_t* getNotifyTargetByCfgHandle(uint16_t cfg_handle);
    static notify_target_t* getNotifyTargetByValHandle(uint16_t val_handle);
    // BLE values and buffers
    static float *xhat_value;
    static float *PRY_value;
    static float *controlU;
    static float *controlGain;
    static float *q;
    static void (*command_cb)(uint8_t* data, uint16_t len);

    static prepare_type_env_t *env_list[5];
};
