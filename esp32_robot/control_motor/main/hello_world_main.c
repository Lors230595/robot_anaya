#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"

// =====================================================================
//  ANGULO DEL ESLABON 1 CON POTENCIOMETRO (sin motores)
//
//  Conexion:
//     Terminal 1 (un extremo)  -> 3.3 V   (NUNCA a 5 V)
//     Terminal 2 (el central)  -> pin ADC1 (senal)
//     Terminal 3 (otro extremo)-> GND
//
//  Imprime SOLO cuando el angulo del brazo cambia mas de UMBRAL_GRADOS.
// =====================================================================

// --- PIN DEL POTENCIOMETRO ---
// Cambia POT_CANAL si usas otro pin (solo pines de ADC1):
//   GPIO36 = ADC_CHANNEL_0 | GPIO39 = ADC_CHANNEL_3
//   GPIO33 = ADC_CHANNEL_5 | GPIO35 = ADC_CHANNEL_7
#define POT_UNIDAD           ADC_UNIT_1
#define POT_CANAL            ADC_CHANNEL_7      // GPIO35

// --- CALIBRACION DEL POTENCIOMETRO (lectura -> grados del potenciometro) ---
#define RAW_MIN              0
#define RAW_MAX              4095
#define ANGULO_MAX           270.0f     // recorrido total del potenciometro

// --- CALIBRACION DEL BRAZO (grados del potenciometro -> grados del brazo) ---
// Medido en tu brazo:
//   brazo en 0°     -> potenciometro en 270° (tope del potenciometro)
//   brazo en ~5°    -> el potenciometro empieza a bajar de 270°
//   brazo en 90°    -> potenciometro en 185°
//   brazo en maximo -> potenciometro en 162°
// El potenciometro baja 1° por cada 1° del brazo, y esta "pegado" a su tope
// los primeros ~5° del brazo (zona muerta). Por eso:
//
//     angulo_brazo = (ANGULO_POT_HOME + ZONA_MUERTA_GRADOS) - angulo_pot
//
//   Comprobacion: pot 185° -> (270 + 5) - 185 = 90°  (correcto)
//                 pot 162° -> (270 + 5) - 162 = 113° (angulo maximo del brazo)
#define ANGULO_POT_HOME      270.0f     // lo que marca el potenciometro con el brazo en 0°
#define ZONA_MUERTA_GRADOS   5.0f       // grados que mueve el brazo antes de que el pot empiece a bajar
#define TOLERANCIA_HOME      1.0f       // si el pot marca >= (270 - 1) se toma el brazo como 0°

// --- FILTRO Y SALIDA ---
#define MUESTRAS_PROMEDIO    64      // lecturas que se promedian en cada medicion (reduce ruido)
#define PERIODO_LECTURA_MS   50      // cada cuanto se mide
#define ALFA_FILTRO          0.2f    // filtro suave: 0.05 = muy estable/lento, 1.0 = sin filtro
#define UMBRAL_GRADOS        1.0f    // cambio minimo del brazo para imprimir

// Promedia varias lecturas del ADC para reducir el ruido
static int leer_promedio(adc_oneshot_unit_handle_t adc)
{
    int suma = 0;
    for (int i = 0; i < MUESTRAS_PROMEDIO; i++) {
        int raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc, POT_CANAL, &raw));
        suma += raw;
    }
    return suma / MUESTRAS_PROMEDIO;
}

// Lectura cruda -> grados del potenciometro (0..270)
static float raw_a_grados_pot(int raw)
{
    float g = (float)(raw - RAW_MIN) * ANGULO_MAX / (float)(RAW_MAX - RAW_MIN);
    if (g < 0.0f) g = 0.0f;
    if (g > ANGULO_MAX) g = ANGULO_MAX;
    return g;
}

// Grados del potenciometro -> grados del brazo (0 = posicion home)
static float grados_pot_a_brazo(float pot)
{
    // Zona muerta: el pot esta en su tope, el brazo esta en 0° (o a menos de ~5°)
    if (pot >= ANGULO_POT_HOME - TOLERANCIA_HOME) return 0.0f;

    float brazo = (ANGULO_POT_HOME + ZONA_MUERTA_GRADOS) - pot;
    if (brazo < 0.0f) brazo = 0.0f;
    return brazo;
}

void app_main(void)
{
    // Inicia el ADC1 en modo lectura unica
    adc_oneshot_unit_handle_t adc;
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = POT_UNIDAD,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &adc));

    // Canal del potenciometro: atenuacion de 12 dB para leer hasta ~3.1 V
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,      // 0..4095
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc, POT_CANAL, &chan_cfg));

    vTaskDelay(pdMS_TO_TICKS(300));
    printf("\n=== Angulo del eslabon 1 con potenciometro ===\n");

    // Lectura inicial: se imprime una sola vez
    int raw = leer_promedio(adc);
    float pot = raw_a_grados_pot(raw);
    float pot_filtrado = pot;                       // angulo del pot ya suavizado
    float brazo = grados_pot_a_brazo(pot_filtrado);
    float ultimo_impreso = brazo;
    printf("Eslabon 1: %.1f grados   (potenciometro=%.1f, raw=%d)\n", brazo, pot_filtrado, raw);

    while (1) {
        raw = leer_promedio(adc);
        pot = raw_a_grados_pot(raw);

        // Filtro suave: el angulo filtrado sigue al medido poco a poco
        pot_filtrado += ALFA_FILTRO * (pot - pot_filtrado);
        brazo = grados_pot_a_brazo(pot_filtrado);

        // Solo imprime cuando el angulo del brazo cambio lo suficiente
        if (fabsf(brazo - ultimo_impreso) >= UMBRAL_GRADOS) {
            ultimo_impreso = brazo;
            printf("Eslabon 1: %.1f grados   (potenciometro=%.1f, raw=%d)\n", brazo, pot_filtrado, raw);
        }

        vTaskDelay(pdMS_TO_TICKS(PERIODO_LECTURA_MS));
    }
}