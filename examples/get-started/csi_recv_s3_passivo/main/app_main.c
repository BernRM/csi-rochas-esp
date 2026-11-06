/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 * SPDX-FileContributor: 2026 CSI Rochas (adaptação S3 passiva com ganho)
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * csi_recv_s3_passivo — fase 2 do CSI Rochas (NÃO é o RX da piloto).
 *
 * Derivado de examples/get-started/csi_recv (esp-csi upstream, ced500f). Mudanças:
 *   1. Só recepção: WIFI_MODE_NULL + modo promíscuo, canal 11 HT20. Sem ESP-NOW, sem scan,
 *      sem connect, sem esp_wifi_80211_tx. Não existe interface STA/AP que possa transmitir.
 *   2. Saída CRUA: o buffer de CSI vai sem compensação, junto com agc_gain, fft_gain, o estado
 *      do baseline e o fator de compensação calculado pelo componente esp_csi_gain_ctrl.
 *      A compensação é aplicada no PC.
 *   3. O callback do Wi-Fi só copia para uma fila; uma task escreve na UART0 a 2 Mbaud.
 *      Cada linha tem seq próprio e contadores (rx_total, fila_cheia) para medir perda.
 *   4. Comandos de 1 byte pela UART0: 'F' trava o ganho no baseline, 'A' volta ao automático,
 *      'R' reinicia o baseline.
 *
 * Linha de dados (uma por quadro; termina em *XX = XOR de todos os bytes antes do '*'):
 *   S3G,seq,t_us,mac,rssi,noise_floor,rate,sig_mode,mcs,cwb,agc,fft,gain_status,comp_x1000,
 *       forcado,first_word_invalid,len,rx_total,fila_cheia,csi_base64*XX
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "driver/uart.h"
#include "mbedtls/base64.h"
#include "esp_csi_gain_ctrl.h"

#if !CONFIG_IDF_TARGET_ESP32S3
#error "csi_recv_s3_passivo foi feito só para ESP32-S3"
#endif

#define S3G_VERSAO            "s3g-0.1"
#define S3G_CANAL             11
#define S3G_UART              UART_NUM_0
#define S3G_FILA_ITENS        256
#define S3G_CSI_MAX           384      /* LLTF no S3 = 128 B; margem para outros formatos */
#define S3G_BASELINE_QUADROS  100      /* igual ao exemplo upstream */
#define S3G_LINHA_MAX         (256 + ((S3G_CSI_MAX + 2) / 3) * 4 + 8)

/* Fonte usada para o baseline do ganho: o TX da piloto (csi_send, MAC fixo). */
static const uint8_t S3G_MAC_BASELINE[6] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};

typedef struct {
    uint32_t seq;
    uint32_t t_us;
    uint8_t  mac[6];
    int8_t   rssi;
    int8_t   noise_floor;
    uint8_t  rate;
    uint8_t  sig_mode;
    uint8_t  mcs;
    uint8_t  cwb;
    uint8_t  agc;
    int8_t   fft;
    uint8_t  gain_status;
    uint8_t  forcado;
    uint8_t  fwi;
    int32_t  comp_x1000;
    uint16_t len;
    uint32_t rx_total;
    uint32_t fila_cheia;
    uint8_t  buf[S3G_CSI_MAX];
} s3g_item_t;

static QueueHandle_t s_fila;
static volatile uint32_t s_rx_total;
static volatile uint32_t s_fila_cheia;
static volatile uint32_t s_seq;
static volatile uint32_t s_linhas;
static volatile uint32_t s_n_baseline;
static volatile bool     s_baseline_ok;
static volatile bool     s_forcado;
static uint8_t s_agc_base;
static int8_t  s_fft_base;

/* Escreve texto na UART0 terminando com *XX\n (XOR). */
static void s3g_escreve(char *linha, size_t n)
{
    uint8_t x = 0;
    for (size_t i = 0; i < n; i++) {
        x ^= (uint8_t)linha[i];
    }
    n += snprintf(linha + n, 6, "*%02X\n", x);
    uart_write_bytes(S3G_UART, linha, n);
}

static void s3g_evento(const char *fmt, ...)
{
    char linha[200];
    int n = snprintf(linha, sizeof(linha), "S3G_EVT,%lu,", (unsigned long)(esp_timer_get_time() & 0xffffffff));
    va_list ap;
    va_start(ap, fmt);
    n += vsnprintf(linha + n, sizeof(linha) - n - 6, fmt, ap);
    va_end(ap);
    if (n > (int)sizeof(linha) - 6) {
        n = sizeof(linha) - 6;
    }
    s3g_escreve(linha, n);
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) {
        return;
    }
    s_rx_total++;

    const wifi_pkt_rx_ctrl_t *rx = &info->rx_ctrl;
    uint8_t agc = 0;
    int8_t fft = 0;
    esp_csi_gain_ctrl_get_rx_gain(rx, &agc, &fft);

    /* Baseline só com quadros do TX da piloto, enquanto o ganho está automático. */
    if (!s_baseline_ok && !s_forcado && memcmp(info->mac, S3G_MAC_BASELINE, 6) == 0) {
        if (s_n_baseline < S3G_BASELINE_QUADROS) {
            esp_csi_gain_ctrl_record_rx_gain(agc, fft);
            s_n_baseline++;
        }
        if (s_n_baseline >= S3G_BASELINE_QUADROS &&
                esp_csi_gain_ctrl_get_rx_gain_baseline(&s_agc_base, &s_fft_base) == ESP_OK) {
            s_baseline_ok = true;
        }
    }

    float comp = 1.0f;
    int32_t comp_x1000 = -1;   /* -1 = sem baseline ainda */
    if (s_baseline_ok && esp_csi_gain_ctrl_get_gain_compensation(&comp, agc, fft) == ESP_OK) {
        comp_x1000 = (int32_t)(comp * 1000.0f + 0.5f);
    }

    s3g_item_t it;
    it.seq = s_seq++;
    it.t_us = rx->timestamp;
    memcpy(it.mac, info->mac, 6);
    it.rssi = rx->rssi;
    it.noise_floor = rx->noise_floor;
    it.rate = rx->rate;
    it.sig_mode = rx->sig_mode;
    it.mcs = rx->mcs;
    it.cwb = rx->cwb;
    it.agc = agc;
    it.fft = fft;
    it.gain_status = (uint8_t)esp_csi_gain_ctrl_get_gain_status();
    it.forcado = s_forcado ? 1 : 0;
    it.fwi = info->first_word_invalid ? 1 : 0;
    it.comp_x1000 = comp_x1000;
    it.len = info->len > S3G_CSI_MAX ? S3G_CSI_MAX : info->len;
    it.rx_total = s_rx_total;
    it.fila_cheia = s_fila_cheia;
    memcpy(it.buf, info->buf, it.len);

    if (xQueueSend(s_fila, &it, 0) != pdTRUE) {
        s_fila_cheia++;
    }
}

static void s3g_task_saida(void *arg)
{
    static s3g_item_t it;
    static char linha[S3G_LINHA_MAX];
    for (;;) {
        if (xQueueReceive(s_fila, &it, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        int n = snprintf(linha, sizeof(linha),
                         "S3G,%lu,%lu," MACSTR ",%d,%d,%u,%u,%u,%u,%u,%d,%u,%ld,%u,%u,%u,%lu,%lu,",
                         (unsigned long)it.seq, (unsigned long)it.t_us, MAC2STR(it.mac),
                         it.rssi, it.noise_floor, it.rate, it.sig_mode, it.mcs, it.cwb,
                         it.agc, it.fft, it.gain_status, (long)it.comp_x1000, it.forcado,
                         it.fwi, it.len, (unsigned long)it.rx_total, (unsigned long)it.fila_cheia);
        size_t olen = 0;
        mbedtls_base64_encode((unsigned char *)linha + n, sizeof(linha) - n - 6, &olen, it.buf, it.len);
        s3g_escreve(linha, n + olen);
        s_linhas++;
    }
}

static void s3g_task_comandos(void *arg)
{
    uint8_t c;
    for (;;) {
        if (uart_read_bytes(S3G_UART, &c, 1, portMAX_DELAY) != 1) {
            continue;
        }
        if (c == 'F') {
            if (!s_baseline_ok) {
                s3g_evento("ERRO_F_SEM_BASELINE,%lu", (unsigned long)s_n_baseline);
            } else {
                esp_err_t e = esp_csi_gain_ctrl_set_rx_force_gain(s_agc_base, s_fft_base);
                s_forcado = (e == ESP_OK);
                s3g_evento("FORCADO,%s,%u,%d", esp_err_to_name(e), s_agc_base, s_fft_base);
            }
        } else if (c == 'A') {
            /* force_gain(0,0) devolve ao automático (esp_csi_gain_ctrl >= 0.1.2). */
            esp_err_t e = esp_csi_gain_ctrl_set_rx_force_gain(0, 0);
            s_forcado = false;
            s3g_evento("AUTO,%s", esp_err_to_name(e));
        } else if (c == 'R') {
            esp_csi_gain_ctrl_set_rx_force_gain(0, 0);
            s_forcado = false;
            esp_csi_gain_ctrl_reset_rx_gain_baseline();
            s_baseline_ok = false;
            s_n_baseline = 0;
            s3g_evento("RESET_BASELINE");
        }
    }
}

static void s3g_task_estado(void *arg)
{
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        char linha[200];
        int n = snprintf(linha, sizeof(linha), "S3G_STAT,%lu,%lu,%lu,%lu,%u,%d,%d,%u,%lu,%u",
                         (unsigned long)(esp_timer_get_time() & 0xffffffff),
                         (unsigned long)s_rx_total, (unsigned long)s_linhas,
                         (unsigned long)s_fila_cheia, (unsigned)uxQueueMessagesWaiting(s_fila),
                         s_baseline_ok ? s_agc_base : 0, s_baseline_ok ? s_fft_base : 0,
                         s_forcado ? 1 : 0, (unsigned long)s_n_baseline,
                         (unsigned)esp_get_free_heap_size());
        s3g_escreve(linha, n);
    }
}

static void s3g_wifi_so_recepcao(void)
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_promiscuous_filter_t filtro = {
        .filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA,
    };
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filtro));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(S3G_CANAL, WIFI_SECOND_CHAN_NONE));

    /* Mesma configuração do RX de produção em modo LLTF-only. */
    wifi_csi_config_t csi = {
        .lltf_en           = true,
        .htltf_en          = false,
        .stbc_htltf2_en    = false,
        .ltf_merge_en      = true,
        .channel_filter_en = true,
        .manu_scale        = false,
        .shift             = 0,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(uart_driver_install(S3G_UART, 1024, 32 * 1024, 0, NULL, 0));
    s_fila = xQueueCreate(S3G_FILA_ITENS, sizeof(s3g_item_t));
    configASSERT(s_fila);

    const esp_app_desc_t *d = esp_app_get_description();
    char linha[200];
    int n = snprintf(linha, sizeof(linha),
                     "S3G_HDR,%s,%s,%s,%s %s,canal=%d,HT20,LLTF,baud=%d,mode=NULL_promiscuo,baseline_mac=" MACSTR,
                     S3G_VERSAO, d->project_name, d->version, d->date, d->time, S3G_CANAL,
                     CONFIG_ESP_CONSOLE_UART_BAUDRATE, MAC2STR(S3G_MAC_BASELINE));
    s3g_escreve(linha, n);
    n = snprintf(linha, sizeof(linha),
                 "S3G_COLS,seq,t_us,mac,rssi,noise_floor,rate,sig_mode,mcs,cwb,agc,fft,gain_status,"
                 "comp_x1000,forcado,first_word_invalid,len,rx_total,fila_cheia,csi_base64");
    s3g_escreve(linha, n);

    xTaskCreatePinnedToCore(s3g_task_saida, "s3g_saida", 6144, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(s3g_task_comandos, "s3g_cmd", 3072, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(s3g_task_estado, "s3g_stat", 3072, NULL, 3, NULL, 1);

    s3g_wifi_so_recepcao();
    s3g_evento("WIFI_OK");
}
