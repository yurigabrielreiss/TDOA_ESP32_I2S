# Localização Espacial de Fontes Sonoras (TDOA + GCC-PHAT)

**Autores:** Yuri Gabriel dos Reis Souza, Prof. Dr. Johnny Werner  
**Resumo:** Implementação de um protótipo de baixo custo para localização de fontes sonoras usando a técnica TDOA e o algoritmo GCC-PHAT em um ESP32 com microfones I2S (INMP441). O sistema estima o atraso de chegada entre dois microfones, converte esse atraso em ângulo de chegada e indica a direção por LEDs e via Serial.

---

## Estrutura do repositório
```
/
├─ README.md                      # (este arquivo)
├─ code/
│  ├─ TDOA_GCC_PHAT.ino           # sketch principal (ESP32)
│  ├─ test_i2s_single.ino         # sketch de teste I2S (2 mics)
├─ docs/
│  ├─ Relatório_LOCALIZAÇÃO ESPACIAL DE FONTES SONORAS_final.pdf # relatório final
│  ├─ electronics-11-00890.pdf    # artigo principal de embasamento do projeto

```

---

## O que tem aqui
- Código Arduino (ESP32) que implementa captura I2S, GCC-PHAT, IFFT e estimação do Δt e do ângulo (DoA).  
- Testes de captura I2S e scripts de validação (RMS/energia).  
- Documentação e relatório final (em `docs/`).  
- Datasheets e referências bibliográficas.

---

## Requisitos de hardware
- 1x ESP32 (WROOM-32D ou similar)  
- 2x microfones I2S INMP441  
- 2x LEDs + resistores (indicadores esquerda/direita)  
- Fios e protoboard / PCB 

---

## Pinagem
| Componente | ESP32 | Observações |
|-------------|-------|-------------|
| BCLK (SCK) | GPIO 26 | Clock I2S |
| LRCLK (WS) | GPIO 25 | Seleciona canal L/R |
| SD (DATA) | GPIO 33 | Entrada de dados I2S |
| LED Esquerda | GPIO 12 | Indica som vindo da esquerda |
| LED Direita | GPIO 13 | Indica som vindo da direita |

---

## Como usar
1. Abra `code/TDOA_GCC_PHAT.ino` no Arduino IDE.  
2. Instale a biblioteca `arduinoFFT`.  
3. Carregue o código no ESP32.  
4. Abra o Serial Monitor (115200 bps) e observe:  
   - `ESQUERDA`, `DIREITA` ou `CENTRO`  
   - LEDs indicando a direção.

---

## Resultados
- Estimação em tempo real de Δt e direção da fonte.  
- LEDs indicam lado da origem do som.  
- Resposta robusta a ruído e reverberação moderada.  
- Testado em ambiente fechado com boa consistência.

---

## Datasheets dos componentes
- ESP32-WROOM-32D: https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32d_esp32-wroom-32u_datasheet_en.pdf
- Microfone Digital I2S INMP441: https://invensense.tdk.com/wp-content/uploads/2015/02/INMP441.pdf

---

## Contato
yurigabrielreiss@hotmail.com
