#include <Arduino.h>
#include "driver/i2s.h"

// ===== PINOS =====
#define I2S_WS   25   // LRCLK / WS
#define I2S_SCK  26   // BCLK
#define I2S_SD   33   // DATA IN (SD compartilhado pelos 2 mics)

// LEDs
#define LED_LEFT   12
#define LED_RIGHT  13

#define FS              16000   // taxa de amostragem
#define FRAMES_PER_READ 256     // nº de frames por leitura (cada frame = L+R)
#define SILENCE_THRESH  0.005f  // limiar RMS (~-46 dBFS)
#define HYST_DB         2.0f    // histerese em dB

// buffers
int32_t ibuf[2 * FRAMES_PER_READ];  // 2 words (R e L) por frame
float   Lbuf[FRAMES_PER_READ];
float   Rbuf[FRAMES_PER_READ];

static float rms(const float* x, int n) {
  double acc = 0.0;
  for (int i = 0; i < n; ++i) acc += (double)x[i] * x[i];
  return sqrt(acc / (double)n);
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_RIGHT, LOW);

  i2s_config_t cfg = {};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = FS;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;      // mic entrega 24b em 32b
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;      // ordem no buffer: R, L
  cfg.communication_format = I2S_COMM_FORMAT_I2S;
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = 8;
  cfg.dma_buf_len = 256;
  cfg.use_apll = true;
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = 0;

  i2s_pin_config_t pins = {};
  pins.bck_io_num   = I2S_SCK;
  pins.ws_io_num    = I2S_WS;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num  = I2S_SD;

  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_set_clk(I2S_NUM_0, FS, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_STEREO);
}

void loop() {
  size_t br = 0;
  i2s_read(I2S_NUM_0, ibuf, sizeof(ibuf), &br, portMAX_DELAY);
  int frames = br / (sizeof(int32_t) * 2);
  if (frames <= 0) return;

  // Deintercala (ordem: R, L)
  for (int i = 0; i < frames; ++i) {
    int32_t r24 = ibuf[2*i + 0] >> 8; // Right
    int32_t l24 = ibuf[2*i + 1] >> 8; // Left
    Rbuf[i] = (float)r24 / 8388608.0f;
    Lbuf[i] = (float)l24 / 8388608.0f;
  }

  // RMS
  float rRMS = rms(Rbuf, frames);
  float lRMS = rms(Lbuf, frames);

  // ILD (diferença em dB)
  float ild_dB = 20.0f * log10f((lRMS + 1e-9f) / (rRMS + 1e-9f));

  // Evita decisões no silêncio
  if (rRMS < SILENCE_THRESH && lRMS < SILENCE_THRESH) {
    Serial.printf("Silêncio | L_RMS=%.5f | R_RMS=%.5f\n", lRMS, rRMS);
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_RIGHT, LOW);
    return;
  }

  // Impressão e LEDs
  Serial.printf("L_RMS=%.5f | R_RMS=%.5f | ILD=%.2f dB | ", lRMS, rRMS, ild_dB);

  if (ild_dB > HYST_DB) {
    Serial.println("Som vindo da ESQUERDA");
    digitalWrite(LED_LEFT, HIGH);
    digitalWrite(LED_RIGHT, LOW);
  } else if (ild_dB < -HYST_DB) {
    Serial.println("Som vindo da DIREITA");
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_RIGHT, HIGH);
  } else {
    Serial.println("Equilibrado");
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_RIGHT, LOW);
  }
}
