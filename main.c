// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "driver/i2s.h"
#include "oled.h"
//new
#include "esp_spp_api.h"	
#include "time.h"	
#include "sys/time.h"
#define SPP_TAG "SPP_ACCEPTOR_DEMO"	
#define SPP_SERVER_NAME "SPP_SERVER"	
#define EXAMPLE_DEVICE_NAME "ESP_SPP_ACCEPTOR"	
#define SPP_SHOW_DATA 0	
#define SPP_SHOW_SPEED 1	
#define SPP_SHOW_MODE SPP_SHOW_SPEED    /*Choose show mode: show data or speed*/	
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;	
static struct timeval time_new, time_old;	
static long data_num = 0;	
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;	
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;	
static void print_speed(void)	
{	
    float time_old_s = time_old.tv_sec + time_old.tv_usec / 1000000.0;	
    float time_new_s = time_new.tv_sec + time_new.tv_usec / 1000000.0;	
    float time_interval = time_new_s - time_old_s;	
    float speed = data_num * 8 / time_interval / 1000.0;	
    ESP_LOGI(SPP_TAG, "speed(%fs ~ %fs): %f kbit/s" , time_old_s, time_new_s, speed);	
    data_num = 0;	
    time_old.tv_sec = time_new.tv_sec;	
    time_old.tv_usec = time_new.tv_usec;	
}	
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)	
{	
    switch (event) {	
    case ESP_SPP_INIT_EVT:	
        ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");	
        esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);	
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);	
        esp_spp_start_srv(sec_mask,role_slave, 0, SPP_SERVER_NAME);	
        break;	
    case ESP_SPP_DISCOVERY_COMP_EVT:	
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");	
        break;	
    case ESP_SPP_OPEN_EVT:	
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");	
        break;	
    case ESP_SPP_CLOSE_EVT:	
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");	
        break;	
    case ESP_SPP_START_EVT:	
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");	
        break;	
    case ESP_SPP_CL_INIT_EVT:	
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");	
        break;	
    case ESP_SPP_DATA_IND_EVT:	
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)	
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len=%d handle=%d",	
                 param->data_ind.len, param->data_ind.handle);	
        esp_log_buffer_hex("",param->data_ind.data,param->data_ind.len);	
#else	
        gettimeofday(&time_new, NULL);	
        data_num += param->data_ind.len;	
        if (time_new.tv_sec - time_old.tv_sec >= 3) {	
            print_speed();	
        }	
#endif	
        break;	
    case ESP_SPP_CONG_EVT:	
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");	
        break;	
    case ESP_SPP_WRITE_EVT:	
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");	
        break;	
    case ESP_SPP_SRV_OPEN_EVT:	
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");	
        gettimeofday(&time_old, NULL);	
        break;	
    case ESP_SPP_SRV_STOP_EVT:	
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");	
        break;	
    case ESP_SPP_UNINIT_EVT:	
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");	
        break;	
    default:	
        break;	
    }	
}
//new

/* event for handler "bt_av_hdl_stack_up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

/* handler for bluetooth stack enabled events */
static void bt_av_hdl_stack_evt(uint16_t event, void *p_param);

void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(BT_AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(BT_AV_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }

    case ESP_BT_GAP_PIN_REQ_EVT:{	
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);	
        if (param->pin_req.min_16_digit) {	
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");	
            esp_bt_pin_code_t pin_code = {0};	
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);	
        } else {	
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");	
            esp_bt_pin_code_t pin_code;	
            pin_code[0] = '1';	
            pin_code[1] = '2';	
            pin_code[2] = '3';	
            pin_code[3] = '4';	
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);	
        }	
        break;	
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    default: {
        ESP_LOGI(BT_AV_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

void app_main(void)
{
    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_LOGI(TAG, "Initialize I2C Master"); 
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "Initialize successful");
    ESP_LOGI(TAG, "Initialize OLED");
    ssd1306_init();
    
    i2s_config_t i2s_config = {
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
#else
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,                                  // Only TX
#endif
        .sample_rate = 44100,
        .bits_per_sample = 16,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           //2-channels
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = 6,
        .dma_buf_len = 60,
        .intr_alloc_flags = 0,                                                  //Default interrupt priority
        .tx_desc_auto_clear = true                                              //Auto clear tx descriptor on underflow
    };


    i2s_driver_install(0, &i2s_config, 0, NULL);
#ifdef CONFIG_EXAMPLE_A2DP_SINK_OUTPUT_INTERNAL_DAC
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
    i2s_set_pin(0, NULL);
#else
    i2s_pin_config_t pin_config = {
        .bck_io_num = CONFIG_EXAMPLE_I2S_BCK_PIN,
        .ws_io_num = CONFIG_EXAMPLE_I2S_LRCK_PIN,
        .data_out_num = CONFIG_EXAMPLE_I2S_DATA_PIN,
        .data_in_num = -1                                                       //Not used
    };

    i2s_set_pin(0, &pin_config);
#endif


    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }

    if ((err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }

    if ((err = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }

    if ((err = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(err));
        return;
    }

    if ((err = esp_bt_gap_register_callback(bt_app_gap_cb)) != ESP_OK) {	
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(err));	
        return;	
    }	
    if ((err = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {	
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(err));	
        return;	
    }	
    if ((err = esp_spp_init(esp_spp_mode)) != ESP_OK) {	
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(err));	
        return;	
    }	
    /* create application task */
    bt_app_task_start_up();

    /* Bluetooth device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use fixed pin code
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code;
    pin_code[0] = '1';
    pin_code[1] = '2';
    pin_code[2] = '3';
    pin_code[3] = '4';
    esp_bt_gap_set_pin(pin_type, 4, pin_code);

}


static void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s evt %d", __func__, event);
    switch (event) {
    case BT_APP_EVT_STACK_UP: {
        /* set up device name */
        char *dev_name = "ESP_SPEAKER";
        esp_bt_dev_set_device_name(dev_name);

        esp_bt_gap_register_callback(bt_app_gap_cb);

        /* initialize AVRCP controller */
        esp_avrc_ct_init();
        esp_avrc_ct_register_callback(bt_app_rc_ct_cb);
        /* initialize AVRCP target */
        assert (esp_avrc_tg_init() == ESP_OK);
        esp_avrc_tg_register_callback(bt_app_rc_tg_cb);

        esp_avrc_rn_evt_cap_mask_t evt_set = {0};
        esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
        assert(esp_avrc_tg_set_rn_evt_cap(&evt_set) == ESP_OK);

        /* initialize A2DP sink */
        esp_a2d_register_callback(&bt_app_a2d_cb);
        esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
        esp_a2d_sink_init();

        /* set discoverable and connectable mode, wait to be connected */
        esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled evt %d", __func__, event);
        break;
    }
}
