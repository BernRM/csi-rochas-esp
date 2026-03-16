/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* Get Start Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "nvs_flash.h"

#include "esp_mac.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_csi_gain_ctrl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"

#define CONFIG_LESS_INTERFERENCE_CHANNEL   11
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61 || (CONFIG_IDF_TARGET_ESP32C6 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0))
#define CONFIG_WIFI_BAND_MODE               WIFI_BAND_MODE_2G_ONLY
#define CONFIG_WIFI_2G_BANDWIDTHS           WIFI_BW_HT40
#define CONFIG_WIFI_5G_BANDWIDTHS           WIFI_BW_HT40
#define CONFIG_WIFI_2G_PROTOCOL             WIFI_PROTOCOL_11N
#define CONFIG_WIFI_5G_PROTOCOL             WIFI_PROTOCOL_11N
#else
#define CONFIG_WIFI_BANDWIDTH           WIFI_BW_HT20
#endif

#define CONFIG_ESP_NOW_PHYMODE           WIFI_PHY_MODE_HT20
#define CONFIG_ESP_NOW_RATE             WIFI_PHY_RATE_MCS0_LGI
#define CONFIG_FORCE_GAIN                   0

// -------- CSI -> SPI framing (custom) --------
#define CSI_SPI_ENABLE                     1
#define CSI_PRINT_CSV                      0

#ifndef CSI_SPI_HOST
#define CSI_SPI_HOST                       SPI2_HOST
#endif

// Ajuste os pinos conforme o seu hardware
#ifndef CSI_SPI_MOSI
#define CSI_SPI_MOSI                       23
#endif
#ifndef CSI_SPI_MISO
#define CSI_SPI_MISO                       19
#endif
#ifndef CSI_SPI_SCLK
#define CSI_SPI_SCLK                       18
#endif
#ifndef CSI_SPI_CS
#define CSI_SPI_CS                         5
#endif

#define CSI_SPI_QUEUE_LEN                  32
#define CSI_SPI_MAX_PAYLOAD                4096
#define CSI_SPI_FRAME_OVERHEAD             4
#define CSI_SPI_MAX_TRANSFER               (CSI_SPI_MAX_PAYLOAD + CSI_SPI_FRAME_OVERHEAD)

#define CSI_SPI_SENTINEL_0                 0xA5
#define CSI_SPI_SENTINEL_1                 0x5A
#define CSI_FAST_MODE_LLTF_ONLY            1

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
#define CSI_FORCE_LLTF                      0
#endif

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
#define CONFIG_GAIN_CONTROL                 1
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
#define ESP_IF_WIFI_STA ESP_MAC_WIFI_STA
#endif

static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t CONFIG_CSI_RECV_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x01};
static const char *TAG = "csi_recv";

typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  type;
    uint8_t  id;
    uint8_t  mac[6];
    int8_t   rssi;
    uint8_t  rate;
    uint8_t  sig_mode;
    uint8_t  mcs;
    uint8_t  bandwidth;
    uint8_t  smoothing;
    uint8_t  not_sounding;
    uint8_t  aggregation;
    uint8_t  stbc;
    uint8_t  fec_coding;
    uint8_t  sgi;
    int8_t   noise_floor;
    uint16_t ampdu_cnt;
    uint16_t channel;
    uint8_t  secondary_channel;
    uint32_t local_timestamp;
    uint8_t  ant;
    uint16_t sig_len;
    uint8_t  rx_state;
    uint16_t len;
    uint16_t first_word;
    uint16_t csi_len;
} csi_wire_header_t;

typedef struct {
    uint8_t *frame;
    size_t len;
} csi_spi_frame_t;

static QueueHandle_t s_spi_queue = NULL;
static uint32_t s_csi_cb_count = 0;
static uint32_t s_spi_enq_count = 0;
static uint32_t s_spi_drop_count = 0;

static void csi_spi_task(void *arg)
{
    (void)arg;
    while (true) {
        csi_spi_frame_t item = {0};
        if (xQueueReceive(s_spi_queue, &item, portMAX_DELAY) == pdTRUE) {
            spi_slave_transaction_t t = {0};
            t.length = item.len * 8;
            t.tx_buffer = item.frame;

            esp_err_t err = spi_slave_transmit(CSI_SPI_HOST, &t, portMAX_DELAY);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "spi_slave_transmit failed: %s", esp_err_to_name(err));
            }

            free(item.frame);
        }
    }
}

static void csi_spi_init(void)
{
    if (!CSI_SPI_ENABLE) {
        return;
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = CSI_SPI_MOSI,
        .miso_io_num = CSI_SPI_MISO,
        .sclk_io_num = CSI_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = CSI_SPI_MAX_TRANSFER,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = CSI_SPI_CS,
        .queue_size = 8,
        .flags = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = NULL,
    };

    ESP_ERROR_CHECK(spi_slave_initialize(CSI_SPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO));

    s_spi_queue = xQueueCreate(CSI_SPI_QUEUE_LEN, sizeof(csi_spi_frame_t));
    configASSERT(s_spi_queue);
    xTaskCreatePinnedToCore(csi_spi_task, "csi_spi_task", 4096, NULL, 5, NULL, 0);
}

static void csi_spi_queue_frame(const wifi_csi_info_t *info, const wifi_pkt_rx_ctrl_t *rx_ctrl,
                                uint32_t rx_id, float compensate_gain)
{
    if (!CSI_SPI_ENABLE || s_spi_queue == NULL) {
        return;
    }

#if (CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61) && CSI_FORCE_LLTF
    uint16_t csi_len = (info->len >= 2) ? (uint16_t)((info->len - 2) / 2) : 0;
#else
    uint16_t csi_len = (uint16_t)info->len;
#endif

    size_t payload_len = sizeof(csi_wire_header_t) + (size_t)csi_len * sizeof(int16_t);
    if (payload_len == 0 || payload_len > CSI_SPI_MAX_PAYLOAD) {
        return;
    }

    size_t frame_len = payload_len + CSI_SPI_FRAME_OVERHEAD;
    uint8_t *frame = (uint8_t *)malloc(frame_len);
    if (!frame) {
        return;
    }

    frame[0] = CSI_SPI_SENTINEL_0;
    frame[1] = CSI_SPI_SENTINEL_1;
    frame[2] = (uint8_t)(payload_len & 0xFF);
    frame[3] = (uint8_t)((payload_len >> 8) & 0xFF);

    csi_wire_header_t hdr = {0};
    hdr.version = 1;
    hdr.type = 1;
    hdr.id = (uint8_t)(rx_id & 0xFF);
    memcpy(hdr.mac, info->mac, sizeof(hdr.mac));
    hdr.rssi = rx_ctrl->rssi;
    hdr.rate = rx_ctrl->rate;
    hdr.sig_mode = rx_ctrl->sig_mode;
    hdr.mcs = rx_ctrl->mcs;
    hdr.bandwidth = rx_ctrl->cwb;
    hdr.smoothing = rx_ctrl->smoothing;
    hdr.not_sounding = rx_ctrl->not_sounding;
    hdr.aggregation = rx_ctrl->aggregation;
    hdr.stbc = rx_ctrl->stbc;
    hdr.fec_coding = rx_ctrl->fec_coding;
    hdr.sgi = rx_ctrl->sgi;
    hdr.noise_floor = rx_ctrl->noise_floor;
    hdr.ampdu_cnt = rx_ctrl->ampdu_cnt;
    hdr.channel = rx_ctrl->channel;
    hdr.secondary_channel = rx_ctrl->secondary_channel;
    hdr.local_timestamp = rx_ctrl->timestamp;
    hdr.ant = rx_ctrl->ant;
    hdr.sig_len = rx_ctrl->sig_len;
    hdr.rx_state = rx_ctrl->rx_state;
    hdr.len = info->len;
    hdr.first_word = info->first_word_invalid;
    hdr.csi_len = csi_len;

    uint8_t *payload = frame + CSI_SPI_FRAME_OVERHEAD;
    memcpy(payload, &hdr, sizeof(hdr));

    uint8_t *dst = payload + sizeof(hdr);
#if (CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61) && CSI_FORCE_LLTF
    if (csi_len > 0) {
        for (int i = 0, j = 0; i < (info->len - 2) && j < csi_len; i += 2, j++) {
            int16_t csi = ((int16_t)(((((uint16_t)info->buf[i + 1]) << 8) | info->buf[i]) << 4) >> 4);
            int16_t v = (int16_t)(compensate_gain * csi);
            memcpy(dst + (size_t)j * sizeof(int16_t), &v, sizeof(int16_t));
        }
    }
#else
    for (int i = 0; i < info->len; i++) {
        int16_t v = (int16_t)(compensate_gain * info->buf[i]);
        memcpy(dst + (size_t)i * sizeof(int16_t), &v, sizeof(int16_t));
    }
#endif

    csi_spi_frame_t item = {
        .frame = frame,
        .len = frame_len,
    };

    if (xQueueSend(s_spi_queue, &item, 0) != pdTRUE) {
        free(frame);
        s_spi_drop_count++;
        return;
    }

    s_spi_enq_count++;
}

static void wifi_init()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

#if CONFIG_IDF_TARGET_ESP32C5
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_band_mode(CONFIG_WIFI_BAND_MODE);
    wifi_protocols_t protocols = {
        .ghz_2g = CONFIG_WIFI_2G_PROTOCOL,
        .ghz_5g = CONFIG_WIFI_5G_PROTOCOL
    };
    ESP_ERROR_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &protocols));
    wifi_bandwidths_t bandwidth = {
        .ghz_2g = CONFIG_WIFI_2G_BANDWIDTHS,
        .ghz_5g = CONFIG_WIFI_5G_BANDWIDTHS
    };
    ESP_ERROR_CHECK(esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &bandwidth));
#elif (CONFIG_IDF_TARGET_ESP32C6 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)) || CONFIG_IDF_TARGET_ESP32C61
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_band_mode(CONFIG_WIFI_BAND_MODE);
    wifi_protocols_t protocols = {
        .ghz_2g = CONFIG_WIFI_2G_PROTOCOL,
    };
    ESP_ERROR_CHECK(esp_wifi_set_protocols(ESP_IF_WIFI_STA, &protocols));
    wifi_bandwidths_t bandwidth = {
        .ghz_2g = CONFIG_WIFI_2G_BANDWIDTHS,
    };
    ESP_ERROR_CHECK(esp_wifi_set_bandwidths(ESP_IF_WIFI_STA, &bandwidth));
#else
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, CONFIG_WIFI_BANDWIDTH));
    ESP_ERROR_CHECK(esp_wifi_start());
#endif

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
#if CONFIG_IDF_TARGET_ESP32C5
    if ((CONFIG_WIFI_BAND_MODE == WIFI_BAND_MODE_2G_ONLY && CONFIG_WIFI_2G_BANDWIDTHS == WIFI_BW_HT20)
            || (CONFIG_WIFI_BAND_MODE == WIFI_BAND_MODE_5G_ONLY && CONFIG_WIFI_5G_BANDWIDTHS == WIFI_BW_HT20)) {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    }
#elif (CONFIG_IDF_TARGET_ESP32C6 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)) || CONFIG_IDF_TARGET_ESP32C61
    if (CONFIG_WIFI_BAND_MODE == WIFI_BAND_MODE_2G_ONLY && CONFIG_WIFI_2G_BANDWIDTHS == WIFI_BW_HT20) {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    }
#else
    if (CONFIG_WIFI_BANDWIDTH == WIFI_BW_HT20) {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
    } else {
        ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
    }
#endif

    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, CONFIG_CSI_RECV_MAC));
}

static void wifi_esp_now_init(esp_now_peer_info_t peer)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    esp_now_rate_config_t rate_config = {
        .phymode = CONFIG_ESP_NOW_PHYMODE,
        .rate = CONFIG_ESP_NOW_RATE,//  WIFI_PHY_RATE_MCS0_LGI,
        .ersu = false,
        .dcm = false
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr, &rate_config));

}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) {
        ESP_LOGW(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    if (memcmp(info->mac, CONFIG_CSI_SEND_MAC, 6)) {
        return;
    }

    s_csi_cb_count++;

    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
    static int s_count = 0;
    float compensate_gain = 1.0f;
    static uint8_t agc_gain = 0;
    static int8_t fft_gain = 0;
#if CONFIG_GAIN_CONTROL
    static uint8_t agc_gain_baseline = 0;
    static int8_t fft_gain_baseline = 0;
    esp_csi_gain_ctrl_get_rx_gain(rx_ctrl, &agc_gain, &fft_gain);
    if (s_count < 100) {
        esp_csi_gain_ctrl_record_rx_gain(agc_gain, fft_gain);
    } else if (s_count == 100) {
        esp_csi_gain_ctrl_get_rx_gain_baseline(&agc_gain_baseline, &fft_gain_baseline);
#if CONFIG_FORCE_GAIN
        esp_csi_gain_ctrl_set_rx_force_gain(agc_gain_baseline, fft_gain_baseline);
        ESP_LOGD(TAG, "fft_force %d, agc_force %d", fft_gain_baseline, agc_gain_baseline);
#endif
    }
    esp_csi_gain_ctrl_get_gain_compensation(&compensate_gain, agc_gain, fft_gain);
    ESP_LOGI(TAG, "compensate_gain %f, agc_gain %d, fft_gain %d", compensate_gain, agc_gain, fft_gain);
#endif

    uint32_t rx_id = *(uint32_t *)(info->payload + 15);
#if CSI_SPI_ENABLE
    csi_spi_queue_frame(info, rx_ctrl, rx_id, compensate_gain);
    if ((s_csi_cb_count % 1000U) == 0U) {
        ESP_LOGI(TAG, "rx_stats: csi_cb=%lu spi_enq=%lu spi_drop=%lu",
                 (unsigned long)s_csi_cb_count,
                 (unsigned long)s_spi_enq_count,
                 (unsigned long)s_spi_drop_count);
    }
#endif
#if CSI_PRINT_CSV
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        ets_printf("type,seq,mac,rssi,rate,noise_floor,fft_gain,agc_gain,channel,local_timestamp,sig_len,rx_state,len,first_word,data\n");
    }

    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
               rx_id, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate,
               rx_ctrl->noise_floor, fft_gain, agc_gain,  rx_ctrl->channel,
               rx_ctrl->timestamp, rx_ctrl->sig_len, rx_ctrl->rx_state);
#else
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        ets_printf("type,id,mac,rssi,rate,sig_mode,mcs,bandwidth,smoothing,not_sounding,aggregation,stbc,fec_coding,sgi,noise_floor,ampdu_cnt,channel,secondary_channel,local_timestamp,ant,sig_len,rx_state,len,first_word,data\n");
    }

    ets_printf("CSI_DATA,%d," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
               rx_id, MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate, rx_ctrl->sig_mode,
               rx_ctrl->mcs, rx_ctrl->cwb, rx_ctrl->smoothing, rx_ctrl->not_sounding,
               rx_ctrl->aggregation, rx_ctrl->stbc, rx_ctrl->fec_coding, rx_ctrl->sgi,
               rx_ctrl->noise_floor, rx_ctrl->ampdu_cnt, rx_ctrl->channel, rx_ctrl->secondary_channel,
               rx_ctrl->timestamp, rx_ctrl->ant, rx_ctrl->sig_len, rx_ctrl->rx_state);

#endif
#if (CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61) && CSI_FORCE_LLTF
    int16_t csi = ((int16_t)(((((uint16_t)info->buf[1]) << 8) | info->buf[0]) << 4) >> 4);
    ets_printf(",%d,%d,\"[%d", (info->len - 2) / 2, info->first_word_invalid, (int16_t)(compensate_gain * csi));
    for (int i = 2; i < (info->len - 2); i += 2) {
        csi = ((int16_t)(((((uint16_t)info->buf[i + 1]) << 8) | info->buf[i]) << 4) >> 4);
        ets_printf(",%d", (int16_t)(compensate_gain * csi));
    }
#else
    ets_printf(",%d,%d,\"[%d", info->len, info->first_word_invalid, (int16_t)(compensate_gain * info->buf[0]));
    for (int i = 1; i < info->len; i++) {
        ets_printf(",%d", (int16_t)(compensate_gain * info->buf[i]));
    }
#endif
    ets_printf("]\"\n");
#endif
    s_count++;
}

static void wifi_csi_init()
{
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    /**< default config */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
    wifi_csi_config_t csi_config = {
        .enable                   = true,
        .acquire_csi_legacy       = false,
        .acquire_csi_force_lltf   = CSI_FORCE_LLTF,
        .acquire_csi_ht20         = true,
        .acquire_csi_ht40         = true,
        .acquire_csi_vht          = false,
        .acquire_csi_su           = false,
        .acquire_csi_mu           = false,
        .acquire_csi_dcm          = false,
        .acquire_csi_beamformed   = false,
        .acquire_csi_he_stbc_mode = 2,
        .val_scale_cfg            = 0,
        .dump_ack_en              = false,
        .reserved                 = false
    };
#elif CONFIG_IDF_TARGET_ESP32C6
    wifi_csi_config_t csi_config = {
        .enable                 = true,
        .acquire_csi_legacy     = false,
        .acquire_csi_ht20       = true,
        .acquire_csi_ht40       = true,
        .acquire_csi_su         = true,
        .acquire_csi_mu         = true,
        .acquire_csi_dcm        = true,
        .acquire_csi_beamformed = true,
        .acquire_csi_he_stbc    = 2,
        .val_scale_cfg          = false,
        .dump_ack_en            = false,
        .reserved               = false
    };
#else
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = !CSI_FAST_MODE_LLTF_ONLY,
        .stbc_htltf2_en    = !CSI_FAST_MODE_LLTF_ONLY,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = false,
        .shift             = false,
    };
#endif
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

void app_main()
{
    /**
     * @brief Initialize NVS
     */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /**
     * @brief Initialize Wi-Fi
     */
    wifi_init();

    /**
     * @brief Initialize ESP-NOW
     *        ESP-NOW protocol see: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html
     */

    esp_now_peer_info_t peer = {
        .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
        .ifidx     = WIFI_IF_STA,
        .encrypt   = false,
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    };

    wifi_esp_now_init(peer);

    ESP_LOGI(TAG, "================ CSI RECV ================");
    ESP_LOGI(TAG, "wifi_channel: %d, filter_mac: " MACSTR ", recv_mac: " MACSTR,
             CONFIG_LESS_INTERFERENCE_CHANNEL, MAC2STR(CONFIG_CSI_SEND_MAC), MAC2STR(CONFIG_CSI_RECV_MAC));
    ESP_LOGI(TAG, "wifi_bandwidth: %s, espnow_phymode: %s",
             (CONFIG_WIFI_BANDWIDTH == WIFI_BW_HT20) ? "HT20" : "HT40",
             (CONFIG_ESP_NOW_PHYMODE == WIFI_PHY_MODE_HT20) ? "HT20" : "HT40");
    ESP_LOGI(TAG, "csi_fast_mode_lltf_only: %d", CSI_FAST_MODE_LLTF_ONLY);

    csi_spi_init();
    wifi_csi_init();
}
