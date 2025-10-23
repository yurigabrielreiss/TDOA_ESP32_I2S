/*
 * TÍTULO: Localização de fontes sonoras
 * AUTOR: Yuri Gabriel dos Reis Souza e Prof. Dr. Johnny Werner
 * DESCRIÇÃO: Este código utiliza dois microfones I2S (INMP441) conectados a um ESP32
 * para determinar a direção de uma fonte sonora. Ele implementa o método de
 * Correlação Cruzada Generalizada com Transformada de Fase (GCC-PHAT) para calcular
 * a Diferença de Tempo de Chegada (TDOA) do som nos microfones.
 */

// === BIBLIOTECAS ===
#include <Arduino.h>
#include <math.h>      
#include "driver/i2s.h"
#include "arduinoFFT.h"

// ===== CONSTANTES  =====
#define DISTANCIA_MICS_M 0.15f
#define VELOCIDADE_SOM_MPS 343.0f

// ===== PINOS =====
#define I2S_WS   25  // Word Select (WS)
#define I2S_SCK  26  // Serial Clock (SCK)
#define I2S_SD   33  // Serial Data (SD)

#define LED_LEFT  12 // LED esquerda
#define LED_RIGHT 13 // LED direita

// ===== CONFIGURAÇÕES I2S E FFT =====
#define FS              16000.0f // Taxa de Amostragem
#define SAMPLES_PER_READ 1024

// Ambiente ruidoso -> aumentar treshold (ex: 1.0e-5f)
#define ENERGY_THRESH 5.0e-6f

// Média Móvel
#define ANGLE_FILTER_SIZE 10      // Número de leituras de ângulo a serem usadas na média.
float angle_history[ANGLE_FILTER_SIZE];
int angle_history_index = 0;
float angle_sum = 0;

// Filtro passa-faixa
#define USE_BANDPASS_FILTER true 
#define MIN_FREQ 150.0f
#define MAX_FREQ 6000.0f

// Treshold de qualidade da correlação calculada no GCC
#define CORRELATION_QUALITY_THRESH 2.0f // O pico deve ser pelo menos 2.0x maior que a média para ser validada

// === BUFFERS GLOBAIS ===
int32_t i2s_buf[SAMPLES_PER_READ * 2];
float   L_buf[SAMPLES_PER_READ];
float   R_buf[SAMPLES_PER_READ];
float vRealL[SAMPLES_PER_READ];
float vImagL[SAMPLES_PER_READ];
float vRealR[SAMPLES_PER_READ];
float vImagR[SAMPLES_PER_READ];

ArduinoFFT FFT_L = ArduinoFFT(vRealL, vImagL, SAMPLES_PER_READ, FS);
ArduinoFFT FFT_R = ArduinoFFT(vRealR, vImagR, SAMPLES_PER_READ, FS);

// Função para calcular a energia de um sinal (soma dos quadrados das amostras).
float calculate_energy(const float* buffer, int len) {
  double sum_sq = 0.0;
  for (int i = 0; i < len; ++i) {
    sum_sq += (double)buffer[i] * buffer[i];
  }
  return (float)(sum_sq / len);
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_RIGHT, LOW);

  for (int i = 0; i < ANGLE_FILTER_SIZE; i++) {
    angle_history[i] = 90.0f;
  }
  angle_sum = 90.0f * ANGLE_FILTER_SIZE;

  // --- Configuração do Hardware I2S ---
  i2s_config_t i2s_config = {};
  i2s_config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  i2s_config.sample_rate = (uint32_t)FS;
  i2s_config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  i2s_config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  i2s_config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  i2s_config.intr_alloc_flags = 0;
  i2s_config.dma_buf_count = 8;
  i2s_config.dma_buf_len = SAMPLES_PER_READ;
  i2s_config.use_apll = false;

  i2s_pin_config_t pin_config = {};
  pin_config.bck_io_num = I2S_SCK;
  pin_config.ws_io_num = I2S_WS;
  pin_config.data_out_num = I2S_PIN_NO_CHANGE;
  pin_config.data_in_num = I2S_SD;

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);

  Serial.println("Driver I2S pronto.");
}

void loop() {
  // --- Leitura Contínua do Áudio ---
  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, i2s_buf, sizeof(i2s_buf), &bytes_read, portMAX_DELAY);

  int samples_read = bytes_read / (sizeof(int32_t) * 2);
  if (samples_read != SAMPLES_PER_READ) return;

  // --- PASSO 1: PRÉ-PROCESSAMENTO ---
  for (int i = 0; i < SAMPLES_PER_READ; ++i) {
    int32_t r32 = i2s_buf[2*i + 0];
    int32_t l32 = i2s_buf[2*i + 1];
    R_buf[i] = (float)(r32 >> 8) / 8388608.0f;
    L_buf[i] = (float)(l32 >> 8) / 8388608.0f;
  }

  if (calculate_energy(L_buf, SAMPLES_PER_READ) < ENERGY_THRESH && calculate_energy(R_buf, SAMPLES_PER_READ) < ENERGY_THRESH) {
    digitalWrite(LED_LEFT, LOW);
    digitalWrite(LED_RIGHT, LOW);
    return;
  }

  // --- PASSO 2: PREPARAR PARA A FFT ---
  for (int i = 0; i < SAMPLES_PER_READ; i++) {
    vRealL[i] = L_buf[i]; vImagL[i] = 0;
    vRealR[i] = R_buf[i]; vImagR[i] = 0;
  }

  // --- PASSO 3: FFT ---
  FFT_L.windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
  FFT_L.compute(FFT_FORWARD);
  FFT_R.windowing(FFT_WIN_TYP_HANN, FFT_FORWARD);
  FFT_R.compute(FFT_FORWARD);

  // --- PASSO 3.5: FILTRO PASSA-BANDA (OPCIONAL) ---
  if (USE_BANDPASS_FILTER) {
    float freq_resolution = FS / SAMPLES_PER_READ;
    int min_bin = (int)(MIN_FREQ / freq_resolution);
    int max_bin = (int)(MAX_FREQ / freq_resolution);
    for (int i = 1; i < (SAMPLES_PER_READ / 2); i++) {
      if (i < min_bin || i > max_bin) {
        vRealL[i] = 0; vImagL[i] = 0; vRealR[i] = 0; vImagR[i] = 0;
        vRealL[SAMPLES_PER_READ-i] = 0; vImagL[SAMPLES_PER_READ-i] = 0;
        vRealR[SAMPLES_PER_READ-i] = 0; vImagR[SAMPLES_PER_READ-i] = 0;
      }
    }
  }

  // --- PASSO 4: CÁLCULO DA CORRELAÇÃO CRUZADA (GCC-PHAT) ---
  for (int i = 0; i < SAMPLES_PER_READ; i++) {
    float real_part = vRealL[i] * vRealR[i] + vImagL[i] * vImagR[i];
    float imag_part = vImagL[i] * vRealR[i] - vRealL[i] * vImagR[i];
    float mag = sqrt(real_part * real_part + imag_part * imag_part);
    if (mag > 1.0e-9) {
        vRealL[i] = real_part / mag;
        vImagL[i] = imag_part / mag;
    } else {
        vRealL[i] = 0;
        vImagL[i] = 0;
    }
  }

  // --- PASSO 5: IFFT ---
  FFT_L.compute(FFT_REVERSE);

  // --- PASSO 6: ENCONTRAR O PICO E AVALIAR A QUALIDADE DO SINAL---
  float max_corr = -1000;
  int delay_index = 0;
  double corr_sum = 0;
  for (int i = 0; i < SAMPLES_PER_READ; i++) {
    corr_sum += vRealL[i];
    if (vRealL[i] > max_corr) {
      max_corr = vRealL[i];
      delay_index = i;
    }
  }
  
  // VERIFICAÇÃO DE QUALIDADE
  float avg_corr = corr_sum / SAMPLES_PER_READ;
  if (max_corr < avg_corr * CORRELATION_QUALITY_THRESH) {
    // provavelmente é ruído. Ignora a leitura.
    return;
  }

  if (delay_index >= SAMPLES_PER_READ / 2) {
    delay_index -= SAMPLES_PER_READ;
  }

  // --- PASSO 7: VALIDAR E CALCULAR O ÂNGULO ---
  int max_theoretical_delay = (int)ceil((DISTANCIA_MICS_M / VELOCIDADE_SOM_MPS) * FS);
  if (abs(delay_index) > max_theoretical_delay + 2) {
      // Atraso fisicamente impossível, ignora a leitura
  } else {
    float tdoa_s = (float)delay_index / FS;
    float path_diff_m = tdoa_s * VELOCIDADE_SOM_MPS;
    float cos_theta = path_diff_m / DISTANCIA_MICS_M;
    if (cos_theta > 1.0) cos_theta = 1.0;
    if (cos_theta < -1.0) cos_theta = -1.0;
    float angle_deg = acos(cos_theta) * 180.0 / M_PI;

    angle_sum -= angle_history[angle_history_index];
    angle_history[angle_history_index] = angle_deg;
    angle_sum += angle_deg;
    angle_history_index = (angle_history_index + 1) % ANGLE_FILTER_SIZE;
    float filtered_angle = angle_sum / ANGLE_FILTER_SIZE;

    Serial.printf("Atraso: %d | Ângulo: %.1f° | ", delay_index, angle_deg, filtered_angle);
    if (filtered_angle < 80.0) {
      Serial.println("--> DIREITA");
      digitalWrite(LED_LEFT, LOW);
      digitalWrite(LED_RIGHT, HIGH);
    } else if (filtered_angle > 100.0) {
      Serial.println("<-- ESQUERDA");
      digitalWrite(LED_LEFT, HIGH);
      digitalWrite(LED_RIGHT, LOW);
    } else {
      Serial.println("--- CENTRO ---");
      digitalWrite(LED_LEFT, LOW);
      digitalWrite(LED_RIGHT, LOW);
    }
  }
}
