#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "ble_gatt_comm.h"

esp_gatts_attr_db_t Ble_comm::gatt_db[IDX_NB] = {
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
    sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID), (uint8_t *)&GATTS_SERVICE_UUID}},

    /* Xhat Notify Characteristic Declaration */
    [IDX_CHAR_A]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /*  Xhat Notify Value */
    [IDX_CHAR_VAL_A] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_Xhat_Telem, ESP_GATT_PERM_READ,
    GATTS_DEMO_CHAR_VAL_LEN_MAX, 0, (uint8_t *)nullptr}},

    /* Xhat Notify  Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_A]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)Ble_comm::cfg_val_on}},

    /*PRY Characteristic Declaration */
    [IDX_CHAR_B]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /*PRY Characteristic Value */
    [IDX_CHAR_VAL_B]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_PRY_Telem, ESP_GATT_PERM_READ,
    GATTS_DEMO_CHAR_VAL_LEN_MAX, 0, (uint8_t *)nullptr}},
    
    /*PRY Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_B]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)Ble_comm::cfg_val_on}},

    /*Control U Characteristic Declaration */
    [IDX_CHAR_C]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /*Control U  Characteristic Value */
    [IDX_CHAR_VAL_C]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_contU_TelemWrite, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    GATTS_DEMO_CHAR_VAL_LEN_MAX, 0, (uint8_t *)nullptr}},

    /*Control U  Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_C]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)Ble_comm::cfg_val_on}},

    /*Control Gain Characteristic Declaration */
    [IDX_CHAR_D]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    /*Control U  Characteristic Value */
    [IDX_CHAR_VAL_D]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_ContGain_Upd, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    GATTS_DEMO_CHAR_VAL_LEN_MAX, 0, (uint8_t *)nullptr}},

    /*Control U  Client Characteristic Configuration Descriptor */
    [IDX_CHAR_CFG_D]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    sizeof(uint16_t), sizeof(uint16_t), (uint8_t *)Ble_comm::cfg_val_on}},

    /*Control command Characteristic Declaration */
    [IDX_CHAR_E]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
    CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

    [IDX_CHAR_VAL_E]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_Command, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    GATTS_DEMO_CHAR_VAL_LEN_MAX, 0, (uint8_t *)nullptr}},  
};
notify_target_t Ble_comm::notify_targets[5] = {
    {0, 0, (uint8_t *)nullptr, sizeof(float)*6, "A"},
    {0, 0, (uint8_t *)nullptr, sizeof(float)*3, "B"},
    {0, 0, (uint8_t *)nullptr, (sizeof(float)*5), "C"},
    {0, 0, (uint8_t *)nullptr, sizeof(float)*5*6, "D"},
    {0, 0, (uint8_t *)nullptr, sizeof(uint8_t)*2, "E"}
};

Ble_comm::Ble_comm(float *xhat_value_p,float *PRY_value_p, float *controlU_p, float *controlGain_p) {
    Ble_comm::xhat_value = xhat_value_p;
    Ble_comm::controlU = controlU_p;
    Ble_comm::controlGain = controlGain_p;
    Ble_comm::PRY_value = PRY_value_p;

    Ble_comm::gatt_db[IDX_CHAR_VAL_A].att_desc.value = (uint8_t*)xhat_value;
    Ble_comm::gatt_db[IDX_CHAR_VAL_B].att_desc.value = (uint8_t*)PRY_value;
    Ble_comm::gatt_db[IDX_CHAR_VAL_C].att_desc.value = (uint8_t*)controlU;
    Ble_comm::gatt_db[IDX_CHAR_VAL_D].att_desc.value = (uint8_t*)controlGain;

    Ble_comm::notify_targets[0].value_ptr = (uint8_t *)xhat_value;
    Ble_comm::notify_targets[1].value_ptr = (uint8_t *)PRY_value;
    Ble_comm::notify_targets[2].value_ptr = (uint8_t *)controlU;
    Ble_comm::notify_targets[3].value_ptr = (uint8_t *)controlGain;
}

uint8_t Ble_comm::adv_config_done = 0;
float *Ble_comm::xhat_value, *Ble_comm::PRY_value, *Ble_comm::controlU, *Ble_comm::controlGain;

prepare_type_env_t *Ble_comm::env_list[5];

gatts_profile_inst Ble_comm::profile_tab[PROFILE_NUM] = {
    {
        .gatts_cb = Ble_comm::gatts_profile_event_handler,
        .gatts_if =ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
        .conn_id = 0xFFFF,                /* Not get the conn_id, so initial is 0xFFFF */
    },
};
void Ble_comm::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(const_cast<esp_ble_adv_params_t*>(&Ble_comm::adv_params));
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(const_cast<esp_ble_adv_params_t*>(&Ble_comm::adv_params));
            }
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, conn_int = %d, latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void Ble_comm::prepare_write_event_env(esp_gatt_if_t gatts_if,
                             esp_ble_gatts_cb_param_t *param,
                             prepare_type_env_t *env) {
    ESP_LOGI(GATTS_TABLE_TAG, "Prepare Write: handle=0x%04X, len=%d", param->write.handle, param->write.len);

    esp_gatt_status_t status = ESP_GATT_OK;

    if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
        status = ESP_GATT_INVALID_OFFSET;
    } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
        status = ESP_GATT_INVALID_ATTR_LEN;
    }

    if (status == ESP_GATT_OK && env->prepare_buf == NULL) {
        env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE);
        env->prepare_len = 0;
        env->target_handle = param->write.handle;

        if (env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s: malloc failed", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }

    if (param->write.need_rsp) {
        esp_gatt_rsp_t rsp = {0};
        rsp.attr_value.len = param->write.len;
        rsp.attr_value.handle = param->write.handle;
        rsp.attr_value.offset = param->write.offset;
        rsp.attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
        memcpy(rsp.attr_value.value, param->write.value, param->write.len);

        esp_err_t err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                                    param->write.trans_id, status, &rsp);
        if (err != ESP_OK) {
            ESP_LOGE(GATTS_TABLE_TAG, "Send response failed: %d", err);
        }
    }

    if (status != ESP_GATT_OK) return;

    memcpy(env->prepare_buf + param->write.offset, param->write.value, param->write.len);
    env->prepare_len += param->write.len;
}

void Ble_comm::exec_write_event_env(esp_ble_gatts_cb_param_t *param, prepare_type_env_t *env_list[], size_t env_count) {
    prepare_type_env_t *target_env = NULL;

    // handle に一致するバッファを探す
    for (size_t i = 0; i < env_count; ++i) {
        if (env_list[i]->prepare_buf && env_list[i]->target_handle == param->write.handle) {
            target_env = env_list[i];
            break;
        }
    }

    // fallback: handle が 0 の場合でも prepare_buf があるものを選ぶ
    if (!target_env) {
        for (size_t i = 0; i < env_count; ++i) {
            if (env_list[i]->prepare_buf && env_list[i]->target_handle != 0) {
                target_env = env_list[i];
                break;
            }
        }
    }

    if (target_env) {
        if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC) {
            ESP_LOGI(GATTS_TABLE_TAG, "Execute Write for handle 0x%04X", target_env->target_handle);
            ESP_LOG_BUFFER_HEX(GATTS_TABLE_TAG, target_env->prepare_buf, target_env->prepare_len);
        } else {
            ESP_LOGI(GATTS_TABLE_TAG, "Prepare Write canceled for handle 0x%04X", target_env->target_handle);
        }

        free(target_env->prepare_buf);
        target_env->prepare_buf = NULL;
        target_env->prepare_len = 0;
        target_env->target_handle = 0;
    } else {
        ESP_LOGW(GATTS_TABLE_TAG, "No matching prepare buffer found");
    }
}

void Ble_comm::gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param){
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(const_cast<uint8_t*>(Ble_comm::RAW_ADV_DATA), sizeof(Ble_comm::RAW_ADV_DATA));
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(const_cast<uint8_t*>(Ble_comm::RAW_SCAN_RSP_DATA), sizeof(Ble_comm::RAW_SCAN_RSP_DATA));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
       	    break;
        }
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
       	    break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                // `param->write.is_prep`が`false`の場合は「通常の書き込み」

                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d", param->write.handle, param->write.len);
                ESP_LOG_BUFFER_HEX(GATTS_TABLE_TAG, param->write.value, param->write.len);

                //CCCD(Notify/Indicate設定)への書き込みかどうかで判定
                bool is_cccd_write = false;
                for (int i = 0; i < sizeof(notify_targets)/sizeof(notify_targets[0]); ++i) {
                    if (param->write.handle == notify_targets[i].cfg_handle) {
                        is_cccd_write = true;
                        break;
                    }
                }

                if (is_cccd_write) {
                    // CCCD(Notify/Indicate設定)への書き込み
                    uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                    for (int i = 0; i < sizeof(notify_targets)/sizeof(notify_targets[0]); ++i) {
                        const notify_target_t *t = &notify_targets[i];
                        if (param->write.handle == t->cfg_handle) {
                            if (descr_value == 0x0001) {
                                ESP_LOGI(GATTS_TABLE_TAG, "notify %s enable", t->label);
                                esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, t->val_handle, t->value_len, t->value_ptr, false);
                            } else if (descr_value == 0x0002) {
                                ESP_LOGI(GATTS_TABLE_TAG, "indicate %s enable", t->label);
                                esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, t->val_handle, t->value_len, t->value_ptr, true);
                            } else if (descr_value == 0x0000) {
                                ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate %s disable", t->label);
                            } else {
                                ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value for %s", t->label);
                                ESP_LOG_BUFFER_HEX(GATTS_TABLE_TAG, param->write.value, param->write.len);
                            }
                            break;
                        }
                    }
                }
                else {
                    // それ以外のCharacteristic Valueへの書き込み処理
                    // 書き込まれる変数は Control U, Control Gain, Remote Command のいずれか
                    if (param->write.handle == notify_targets[2].val_handle) { // Control U
                        if (param->write.len == sizeof(float) + 1) { // +1は制御信号の変更バイトフラグ
                            uint8_t index = param->write.value[4];
                            if (index < 5) { // controlUが5要素の場合
                                float value;
                                memcpy(&value, param->write.value, sizeof(float)); // 先頭4バイトをfloatに変換
                                Ble_comm::controlU[index] = value;
                                ESP_LOGI(GATTS_TABLE_TAG, "Control U[%d] updated: %f", index, value);
                            } else {
                                ESP_LOGE(GATTS_TABLE_TAG, "Invalid index for Control U write");
                            }
                        } else {
                            ESP_LOGE(GATTS_TABLE_TAG, "Invalid length for Control U write");
                        }
                    } else if (param->write.handle == notify_targets[3].val_handle) { // Control Gain
                        constexpr size_t expected_bytes = sizeof(float) * 5 * 6; // 5要素の制御ゲイン、各要素は6つの値を持つ
                        if (param->write.len == expected_bytes) {
                            // raw uint8_t* → float array is safe via memcpy
                            memcpy(Ble_comm::controlGain, param->write.value, expected_bytes);
                        } else {
                            ESP_LOGE(GATTS_TABLE_TAG, "Invalid length for Control Gain write: %d", param->write.len);
                        }
                    } else if (param->write.handle == notify_targets[4].val_handle) { // Remote Command
                        if (param->write.len > sizeof(uint8_t)) { // 2バイト以上のコマンド
                            command_cb(param->write.value, param->write.len);
                            ESP_LOGI(GATTS_TABLE_TAG, "Remote Command updated: %d", param->write.value[0]);
                        } else {
                            ESP_LOGE(GATTS_TABLE_TAG, "Invalid length for Remote Command write");
                        }
                    } else {
                        ESP_LOGW(GATTS_TABLE_TAG, "Unknown write handle: 0x%04X", param->write.handle);
                    }
                }

                if (param->write.need_rsp) {
                    //- 書き込み要求に対してレスポンスを返す（`need_rsp`がtrueの場合）
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }else {
                // `param->write.is_prep`が`true`の場合は「プリペアド書き込み」
                for (int i = 0; i < sizeof(Ble_comm::env_list)/sizeof(Ble_comm::env_list[0]); ++i) {
                    if (param->write.handle == Ble_comm::env_list[i]->target_handle || Ble_comm::env_list[i]->prepare_buf == NULL) {
                        prepare_write_event_env(gatts_if, param, Ble_comm::env_list[i]);
                        break;
                    }
                }
            }
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            exec_write_event_env(param, env_list, sizeof(env_list)/sizeof(env_list[0]));
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            //ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT: {
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            ESP_LOG_BUFFER_HEX(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            Ble_comm::profile_tab[PROFILE_APP_IDX].conn_id = param->connect.conn_id;
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT:{
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            Ble_comm::profile_tab[PROFILE_APP_IDX].conn_id = 0xFFFF; // または -1
            esp_ble_gap_start_advertising(const_cast<esp_ble_adv_params_t*>(&Ble_comm::adv_params));
            break;
        }
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != IDX_NB){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to IDX_NB(%d)", param->add_attr_tab.num_handle, IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d",param->add_attr_tab.num_handle);               
                notify_targets[0].cfg_handle = param->add_attr_tab.handles[IDX_CHAR_CFG_A];
                notify_targets[0].val_handle = param->add_attr_tab.handles[IDX_CHAR_VAL_A];
                notify_targets[1].cfg_handle = param->add_attr_tab.handles[IDX_CHAR_CFG_B];
                notify_targets[1].val_handle = param->add_attr_tab.handles[IDX_CHAR_VAL_B];
                notify_targets[2].cfg_handle = param->add_attr_tab.handles[IDX_CHAR_CFG_C];
                notify_targets[2].val_handle = param->add_attr_tab.handles[IDX_CHAR_VAL_C];
                notify_targets[3].cfg_handle = param->add_attr_tab.handles[IDX_CHAR_CFG_D];
                notify_targets[3].val_handle = param->add_attr_tab.handles[IDX_CHAR_VAL_D];
                notify_targets[4].cfg_handle = param->add_attr_tab.handles[IDX_CHAR_CFG_E];
                notify_targets[4].val_handle = param->add_attr_tab.handles[IDX_CHAR_VAL_E];
                esp_ble_gatts_start_service(param->add_attr_tab.handles[IDX_SVC]);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}

void Ble_comm::gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param){
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            Ble_comm::profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id, param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == profile_tab[idx].gatts_if) {
                if (profile_tab[idx].gatts_cb) {
                    profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

esp_err_t Ble_comm::begin(void){
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t ret;
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return ret;
    }
    if ((ret = esp_ble_gatts_register_callback(Ble_comm::gatts_event_handler)) != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return ret;
    }
    if ((ret = esp_ble_gap_register_callback(Ble_comm::gap_event_handler)) != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return ret;
    }
    if ((ret = esp_ble_gatts_app_register(ESP_APP_ID)) != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return ret;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret != ESP_OK) {
        ESP_LOGE(GATTS_TABLE_TAG, "set local MTU failed, error code = %x", local_mtu_ret);
    }
    return ESP_OK;
}
void Ble_comm::sendMsg(char *msg, unsigned char len) {
    // 接続中かつconn_idが有効な場合のみ送信
    if (Ble_comm::profile_tab[PROFILE_APP_IDX].gatts_if != ESP_GATT_IF_NONE &&
        Ble_comm::profile_tab[PROFILE_APP_IDX].conn_id != 0xFFFF) {
        esp_ble_gatts_send_indicate(
            profile_tab[PROFILE_APP_IDX].gatts_if,
            profile_tab[PROFILE_APP_IDX].conn_id,
            notify_targets[4].val_handle,
            len,
            (uint8_t*)msg,
            false
        );
    }
}
void Ble_comm::sendTelemetry(){
    if (Ble_comm::profile_tab[PROFILE_APP_IDX].gatts_if != ESP_GATT_IF_NONE &&
        Ble_comm::profile_tab[PROFILE_APP_IDX].conn_id != 0xFFFF) {
        // Notify Xhat telemetry
        ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(profile_tab[PROFILE_APP_IDX].gatts_if, profile_tab[PROFILE_APP_IDX].conn_id,
                                    notify_targets[0].val_handle, notify_targets[0].value_len, notify_targets[0].value_ptr, false));
        // Notify PRY telemetry
        ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(profile_tab[PROFILE_APP_IDX].gatts_if, profile_tab[PROFILE_APP_IDX].conn_id,
                                    notify_targets[1].val_handle, notify_targets[1].value_len, notify_targets[1].value_ptr, false));
        // Notify Control U telemetry
        ESP_ERROR_CHECK(esp_ble_gatts_send_indicate(profile_tab[PROFILE_APP_IDX].gatts_if, profile_tab[PROFILE_APP_IDX].conn_id,
                                    notify_targets[2].val_handle, notify_targets[2].value_len, notify_targets[2].value_ptr, false));
    }
}