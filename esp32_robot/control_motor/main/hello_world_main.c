#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_oneshot.h"

// --- DEFINICIÓN DE PINES ADC ---
#define POT1_ADC_CHANNEL ADC_CHANNEL_6 // GPIO34
#define POT2_ADC_CHANNEL ADC_CHANNEL_7 // GPIO35
#define POT_MAX_ANGLE    270.0f

// --- CONFIGURACIÓN DEL FILTRO DE RUIDO ---
#define ALPHA_FILTRO     0.15f 

// --- DEFINICIÓN DE PINES MOTORES ---
#define TB6600_1_STEP   GPIO_NUM_26
#define TB6600_1_DIR    GPIO_NUM_25
#define TB6600_1_ENABLE GPIO_NUM_27

#define TB6600_2_STEP   GPIO_NUM_14
#define TB6600_2_DIR    GPIO_NUM_12
#define TB6600_2_ENABLE GPIO_NUM_13

// --- PARÁMETROS DEL MECANISMO ---
#define PASOS_POR_VUELTA       400.0f
#define GRADOS_POR_PASO        (360.0f / PASOS_POR_VUELTA)

#define OBJETIVO_MOTOR1_GRADOS 30.0f
#define OBJETIVO_MOTOR2_GRADOS 120.0f
#define MARGEN_TOLERANCIA_DEG  1.5f

#define ANCHO_PULSO_US         10

// --- PARÁMETROS DE LA RAMPA EXPONENCIAL ---
#define RETARDO_MIN_US         400    // Tiempo a velocidad máxima (us)
#define RETARDO_MAX_US         2500   // Tiempo de arranque/parada suave (us)
#define PASOS_RAMPA            30     // Pasos dedicados a acelerar/desacelerar
#define PASOS_POR_RAFAGA       10     // Pasos por ciclo de FreeRTOS

// --- ESTRUCTURA DE ESTADO COMPARTIDO ---
typedef struct {
    float angulo_m1;
    float angulo_m2;
    float objetivo_m1;
    float objetivo_m2;
    SemaphoreHandle_t mutex;
} estado_adc_t;

static estado_adc_t g_estado_adc;

typedef struct {
    adc_channel_t adc_channel;
    gpio_num_t pin_step;
    gpio_num_t pin_dir;
    float objetivo_grados;
    const char *nombre;
    uint8_t motor_id;
} config_motor_t;

// Función de paso con retardo dinámico para controlar la velocidad
static void dar_paso_dinamico(gpio_num_t pin_step, gpio_num_t pin_dir, bool sentido, uint32_t retardo_us)
{
    gpio_set_level(pin_dir, sentido);
    gpio_set_level(pin_step, 1);
    esp_rom_delay_us(ANCHO_PULSO_US);
    gpio_set_level(pin_step, 0);
    
    if (retardo_us > ANCHO_PULSO_US) {
        esp_rom_delay_us(retardo_us - ANCHO_PULSO_US);
    }
}

// --- TAREA LECTURA ADC CON FILTRO ---
static void tarea_lectura_adc(void *arg)
{
    adc_oneshot_unit_handle_t adc_handle = (adc_oneshot_unit_handle_t)arg;
    int raw_val1 = 0, raw_val2 = 0;

    static float angulo_filtrado_m1 = -1.0f;
    static float angulo_filtrado_m2 = -1.0f;

    while (1) {
        if (adc_oneshot_read(adc_handle, POT1_ADC_CHANNEL, &raw_val1) == ESP_OK &&
            adc_oneshot_read(adc_handle, POT2_ADC_CHANNEL, &raw_val2) == ESP_OK) {
            
            float a1_raw = ((float)raw_val1 / 4095.0f) * POT_MAX_ANGLE;
            float a2_raw = ((float)raw_val2 / 4095.0f) * POT_MAX_ANGLE;

            if (angulo_filtrado_m1 < 0.0f) {
                angulo_filtrado_m1 = a1_raw;
                angulo_filtrado_m2 = a2_raw;
            } else {
                angulo_filtrado_m1 = (ALPHA_FILTRO * a1_raw) + ((1.0f - ALPHA_FILTRO) * angulo_filtrado_m1);
                angulo_filtrado_m2 = (ALPHA_FILTRO * a2_raw) + ((1.0f - ALPHA_FILTRO) * angulo_filtrado_m2);
            }

            if (xSemaphoreTake(g_estado_adc.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_estado_adc.angulo_m1 = angulo_filtrado_m1;
                g_estado_adc.angulo_m2 = angulo_filtrado_m2;
                xSemaphoreGive(g_estado_adc.mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --- TAREA CONTROL DE MOTOR ---
static void tarea_control_motor(void *arg)
{
    const config_motor_t *cfg = (const config_motor_t *)arg;
    int contador_log = 0;
    int pasos_recorridos = 0;
    float velocidad_pasos_sec = 0.0f;

    while (1) {
        float angulo_actual = 0.0f;
        float objetivo_grados = cfg->objetivo_grados;

        if (xSemaphoreTake(g_estado_adc.mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            angulo_actual = (cfg->motor_id == 1) ? g_estado_adc.angulo_m1 : g_estado_adc.angulo_m2;
            objetivo_grados = (cfg->motor_id == 1) ? g_estado_adc.objetivo_m1 : g_estado_adc.objetivo_m2;
            xSemaphoreGive(g_estado_adc.mutex);
        }

        float error = objetivo_grados - angulo_actual;
        bool en_tolerancia = fabsf(error) <= MARGEN_TOLERANCIA_DEG;
        int pasos_faltantes = (int)(fabsf(error) / GRADOS_POR_PASO);

        if (!en_tolerancia) {
            bool direccion = (error > 0.0f) ? 1 : 0;
            int pasos_a_dar = (pasos_faltantes < PASOS_POR_RAFAGA) ? pasos_faltantes : PASOS_POR_RAFAGA;

            for (int i = 0; i < pasos_a_dar; i++) {
                // Cálculo de factores de rampa exponencial
                float factor_acel = (float)pasos_recorridos / (float)PASOS_RAMPA;
                float factor_desacel = (float)pasos_faltantes / (float)PASOS_RAMPA;

                if (factor_acel > 1.0f) factor_acel = 1.0f;
                if (factor_desacel > 1.0f) factor_desacel = 1.0f;

                float factor_velocidad = (factor_acel < factor_desacel) ? factor_acel : factor_desacel;
                float exp_decay = expf(-3.0f * factor_velocidad); 
                uint32_t retardo_actual_us = (uint32_t)(RETARDO_MIN_US + (RETARDO_MAX_US - RETARDO_MIN_US) * exp_decay);

                // Calcular velocidad actual para la gráfica
                velocidad_pasos_sec = 1000000.0f / (float)retardo_actual_us;

                dar_paso_dinamico(cfg->pin_step, cfg->pin_dir, direccion, retardo_actual_us);

                pasos_recorridos++;
                pasos_faltantes--;
            }

            vTaskDelay(1); // Ceder tiempo sin bloquear
        } else {
            pasos_recorridos = 0;
            velocidad_pasos_sec = 0.0f;
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // SALIDA ÚNICA A TELEPLOT (Se imprime periódicamente fuera del loop rápido)
        if (++contador_log >= 5) {
            printf(">%s_Pos_Actual:%.1f\n", cfg->nombre, angulo_actual);
            printf(">%s_Pasos_Restantes:%d\n", cfg->nombre, en_tolerancia ? 0 : pasos_faltantes);
            printf(">%s_Objetivo:%.1f\n", cfg->nombre, objetivo_grados);
            printf(">%s_Velocidad:%.1f\n", cfg->nombre, velocidad_pasos_sec);
            fflush(stdout);
            contador_log = 0;
        }
    }
}

// Lee los nuevos setpoints desde la consola en el formato: M1,M2 y Enter.
static void tarea_entrada_setpoints(void *arg)
{
    (void)arg;
    char linea[64];

    while (1) {
        printf("Ingrese setpoints en grados (M1,M2): ");
        fflush(stdout);

        if (fgets(linea, sizeof(linea), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        float objetivo_m1, objetivo_m2;
        char extra;
        if (sscanf(linea, " %f , %f %c", &objetivo_m1, &objetivo_m2, &extra) == 2 &&
            isfinite(objetivo_m1) && isfinite(objetivo_m2)) {
            if (xSemaphoreTake(g_estado_adc.mutex, portMAX_DELAY) == pdTRUE) {
                g_estado_adc.objetivo_m1 = objetivo_m1;
                g_estado_adc.objetivo_m2 = objetivo_m2;
                xSemaphoreGive(g_estado_adc.mutex);
            }
            printf("Nuevos setpoints: M1=%.2f, M2=%.2f\n", objetivo_m1, objetivo_m2);
        } else {
            printf("Entrada no valida. Use: numero,numero\n");
        }
    }
}

void app_main(void)
{
    gpio_config_t config_motores = {
        .pin_bit_mask = (1ULL << TB6600_1_STEP) | (1ULL << TB6600_1_DIR) | (1ULL << TB6600_1_ENABLE) |
                        (1ULL << TB6600_2_STEP) | (1ULL << TB6600_2_DIR) | (1ULL << TB6600_2_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config_motores);

    gpio_set_level(TB6600_1_ENABLE, 0);
    gpio_set_level(TB6600_2_ENABLE, 0);

    adc_oneshot_unit_handle_t adc_handle;
    const adc_oneshot_unit_init_cfg_t unit_config = { .unit_id = ADC_UNIT_1 };
    const adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &adc_handle));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, POT1_ADC_CHANNEL, &channel_config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, POT2_ADC_CHANNEL, &channel_config));

    g_estado_adc.mutex = xSemaphoreCreateMutex();
    g_estado_adc.objetivo_m1 = OBJETIVO_MOTOR1_GRADOS;
    g_estado_adc.objetivo_m2 = OBJETIVO_MOTOR2_GRADOS;

    static const config_motor_t motor1_cfg = {
        .adc_channel = POT1_ADC_CHANNEL,
        .pin_step = TB6600_1_STEP,
        .pin_dir = TB6600_1_DIR,
        .objetivo_grados = OBJETIVO_MOTOR1_GRADOS,
        .nombre = "M1",
        .motor_id = 1
    };

    static const config_motor_t motor2_cfg = {
        .adc_channel = POT2_ADC_CHANNEL,
        .pin_step = TB6600_2_STEP,
        .pin_dir = TB6600_2_DIR,
        .objetivo_grados = OBJETIVO_MOTOR2_GRADOS,
        .nombre = "M2",
        .motor_id = 2
    };

    xTaskCreate(tarea_lectura_adc, "tarea_adc", 3072, (void *)adc_handle, 5, NULL);
    xTaskCreate(tarea_control_motor, "control_m1", 3072, (void *)&motor1_cfg, 5, NULL);
    xTaskCreate(tarea_control_motor, "control_m2", 3072, (void *)&motor2_cfg, 5, NULL);
    xTaskCreate(tarea_entrada_setpoints, "entrada_setpoints", 3072, NULL, 5, NULL);
    
}