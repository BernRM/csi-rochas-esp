# csi_recv_s3_passivo

**Fase 2 do CSI Rochas — não é o RX da piloto.** Firmware de teste para ESP32-S3: só recebe,
no canal 11, e manda pela UART o CSI cru junto com o ganho de recepção (AGC/FFT) de cada quadro.
O objetivo é responder:

1. o ganho de recepção do S3 varia de quadro para quadro?
2. compensar esse ganho reduz a variância da amplitude do CSI com o canal parado?

É derivado de `examples/get-started/csi_recv` (esp-csi, Apache-2.0) e usa o componente
`espressif/esp_csi_gain_ctrl` 0.1.5.

## Diferenças em relação ao `csi_recv`

| | `csi_recv` (upstream) | `csi_recv_s3_passivo` |
|---|---|---|
| Wi-Fi | STA + ESP-NOW (peer broadcast) | `WIFI_MODE_NULL` + promíscuo; sem ESP-NOW, sem scan, sem connect, sem `esp_wifi_80211_tx` |
| Canal | 11, HT40 | 11, HT20, LLTF apenas (igual ao RX de produção em modo LLTF) |
| Filtro | só o MAC `1a:00:00:00:00:00` | todos os quadros MGMT e DATA; o baseline do ganho usa só o MAC `1a:00:…` |
| CSI impresso | já multiplicado pela compensação | **cru**, com `agc`, `fft`, `gain_status` e `comp_x1000` na mesma linha |
| Saída | `ets_printf` no callback, 921600 baud | fila do FreeRTOS + task dedicada, 2.000.000 baud, linha com XOR |
| Ganho travado | constante de compilação | comandos pela serial: `F` trava no baseline, `A` volta ao automático, `R` reinicia o baseline |

## Formato da linha

Cada linha termina em `*XX`, o XOR de todos os bytes antes do `*`. São três tipos:

- `S3G_HDR` e `S3G_COLS`: mandados no boot.
- `S3G_EVT,<t_us>,...`: confirmação de comandos.
- `S3G_STAT,<t_us>,rx_total,linhas,fila_cheia,fila_ocupada,agc_base,fft_base,forcado,n_baseline,heap`:
  mandado a cada 1 s.

A linha de dados tem um quadro:

```
S3G,seq,t_us,mac,rssi,noise_floor,rate,sig_mode,mcs,cwb,agc,fft,gain_status,comp_x1000,forcado,first_word_invalid,len,rx_total,fila_cheia,csi_base64*XX
```

- `gain_status`: 0 = coletando o baseline, 1 = pronto, 2 = travado.
- `comp_x1000`: o fator de compensação × 1000, ou -1 enquanto não há baseline. A amplitude
  compensada é `cru × comp_x1000 / 1000`.
- `csi_base64`: os bytes int8 em pares (imag, real). Com `first_word_invalid=1`, as duas
  primeiras subportadoras são inválidas.
- `seq` é contínuo no firmware. Um buraco no `seq` visto pelo PC indica perda na serial; o
  `fila_cheia` indica perda dentro do firmware.

## Build (ESP-IDF 5.5.0)

```
idf.py set-target esp32s3
idf.py build
```

O `sdkconfig.defaults` é para o ESP32-S3-WROOM-1-N16R8: flash de 16 MB DIO, app única, console
na UART0 a 2 Mbaud e USB-Serial-JTAG desligado.

## Uso

```
python tools/s3g_captura.py --porta /dev/ttyACM0 --duracao 1200 --forcar-apos 600 --saida captura_AAAAMMDD
python tools/s3g_analise.py captura_AAAAMMDD.s3g.txt --min-quadros 2000 --saida-md resultado.md
pytest tools/test_s3g.py
```

- A captura precisa só do `pyserial`; a análise precisa do `numpy`.
- Os dados capturados **não** vão para este repositório.
