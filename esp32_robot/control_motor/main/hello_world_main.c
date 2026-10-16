#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

// --- DEFINICIÓN DE PINES MOTORES ---
#define TB6600_1_STEP   GPIO_NUM_26
#define TB6600_1_DIR    GPIO_NUM_25
#define TB6600_1_ENABLE GPIO_NUM_27

#define TB6600_2_STEP   GPIO_NUM_14
#define TB6600_2_DIR    GPIO_NUM_12
#define TB6600_2_ENABLE GPIO_NUM_13

// --- PARÁMETROS DEL MECANISMO ---
#define PASOS_POR_VUELTA       400.0f
#define GRADOS_POR_PASO        (360.0f / PASOS_POR_VUELTA)   // 0.9 grados por paso
#define MAX_GRADOS_ENTRADA     3600.0f                       // límite de seguridad por comando

#define ANCHO_PULSO_US         10

// --- PARÁMETROS DE LA RAMPA EXPONENCIAL ---
#define RETARDO_MIN_US         400    // Tiempo a velocidad máxima (us)
#define RETARDO_MAX_US         2500   // Tiempo de arranque/parada suave (us)
#define PASOS_RAMPA            30     // Pasos dedicados a acelerar/desacelerar
#define PASOS_POR_RAFAGA       10     // Pasos por ciclo de FreeRTOS

// --- ESTADO COMPARTIDO ---
typedef struct {
    int32_t pasos_m1;      // pasos a dar (con signo: + sentido normal, - sentido contrario)
    int32_t pasos_m2;
    bool    moviendo_m1;   // true desde que se da la orden hasta que el motor termina
    bool    moviendo_m2;
    SemaphoreHandle_t mutex;
} estado_t;

static estado_t g_estado;

typedef struct {
    gpio_num_t pin_step;
    gpio_num_t pin_dir;
    const char *nombre;
    uint8_t motor_id;
} config_motor_t;

static TaskHandle_t g_tarea_motor[2];

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

// SALIDA A TELEPLOT
static void imprimir_teleplot(const char *nombre, int32_t pos_pasos, int pasos_restantes,
                              float objetivo_grados, float velocidad_pasos_sec)
{
    printf(">%s_Pos_Actual:%.1f\n", nombre, pos_pasos * GRADOS_POR_PASO);
    printf(">%s_Pasos_Restantes:%d\n", nombre, pasos_restantes);
    printf(">%s_Objetivo:%.1f\n", nombre, objetivo_grados);
    printf(">%s_Velocidad:%.1f\n", nombre, velocidad_pasos_sec);
    fflush(stdout);
}

// --- TAREA CONTROL DE MOTOR ---
// Espera una orden (notificación), da EXACTAMENTE los pasos pedidos y se detiene.
static void tarea_control_motor(void *arg)
{
    const config_motor_t *cfg = (const config_motor_t *)arg;
    int32_t posicion_pasos = 0;   // posición acumulada desde el arranque (en pasos, con signo)

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);   // dormido hasta recibir una orden

        int32_t pasos_orden = 0;
        if (xSemaphoreTake(g_estado.mutex, portMAX_DELAY) == pdTRUE) {
            pasos_orden = (cfg->motor_id == 1) ? g_estado.pasos_m1 : g_estado.pasos_m2;
            xSemaphoreGive(g_estado.mutex);
        }

        int pasos_faltantes = abs((int)pasos_orden);
        int pasos_recorridos = 0;
        bool direccion = (pasos_orden > 0);
        int32_t posicion_inicial = posicion_pasos;
        float objetivo_grados = (posicion_inicial + pasos_orden) * GRADOS_POR_PASO;
        float velocidad_pasos_sec = 0.0f;
        int contador_log = 0;

        // Fijar sentido y dar tiempo de establecimiento al TB6600
        gpio_set_level(cfg->pin_dir, direccion);
        esp_rom_delay_us(50);

        while (pasos_faltantes > 0) {
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

            posicion_pasos = posicion_inicial + (direccion ? pasos_recorridos : -pasos_recorridos);

            if (++contador_log >= 5) {
                imprimir_teleplot(cfg->nombre, posicion_pasos, pasos_faltantes, objetivo_grados, velocidad_pasos_sec);
                contador_log = 0;
            }

            vTaskDelay(1); // Ceder tiempo sin bloquear
        }

        // Muestra final con velocidad 0
        posicion_pasos = posicion_inicial + pasos_orden;
        imprimir_teleplot(cfg->nombre, posicion_pasos, 0, objetivo_grados, 0.0f);

        // Avisar que este motor terminó
        if (xSemaphoreTake(g_estado.mutex, portMAX_DELAY) == pdTRUE) {
            if (cfg->motor_id == 1) g_estado.moviendo_m1 = false;
            else                    g_estado.moviendo_m2 = false;
            xSemaphoreGive(g_estado.mutex);
        }
    }
}

// --- ENTRADA POR CONSOLA (lectura bloqueante con eco) ---
static void leer_linea(char *buf, size_t max)
{
    size_t len = 0;
    uint8_t c;

    while (1) {
        if (uart_read_bytes(UART_NUM_0, &c, 1, portMAX_DELAY) != 1) continue;

        if (c == '\r' || c == '\n') {
            if (len == 0) continue;           // ignora \n sobrante tras \r
            uart_write_bytes(UART_NUM_0, "\r\n", 2);
            break;
        } else if (c == 0x08 || c == 0x7F) {  // borrar
            if (len > 0) {
                len--;
                uart_write_bytes(UART_NUM_0, "\b \b", 3);
            }
        } else if (len < max - 1 && c >= 32 && c < 127) {
            buf[len++] = (char)c;
            uart_write_bytes(UART_NUM_0, (const char *)&c, 1);  // eco
        }
    }
    buf[len] = '\0';
}

// Interpreta un comando "M1,M2" (ej: 100,150). Decimales con punto: 45.5,90
static bool parsear_comando(const char *texto, float *v1, float *v2)
{
    char *fin = NULL;
    const char *p = texto;

    *v1 = strtof(p, &fin);
    if (fin == p) return false;
    p = fin;

    while (*p == ' ') p++;
    if (*p != ',') return false;
    p++;

    *v2 = strtof(p, &fin);
    if (fin == p) return false;
    p = fin;

    while (*p == ' ') p++;
    if (*p != '\0') return false;

    return isfinite(*v1) && isfinite(*v2);
}

static void pedir_comando(float *g1, float *g2)
{
    char buf[48];

    while (1) {
        printf("Comando M1,M2 (grados a mover, ej: 100,150): ");
        fflush(stdout);
        leer_linea(buf, sizeof(buf));

        if (!parsear_comando(buf, g1, g2)) {
            printf("  Comando invalido. Use: numero,numero (ej: 100,150)\n");
            continue;
        }
        if (fabsf(*g1) > MAX_GRADOS_ENTRADA || fabsf(*g2) > MAX_GRADOS_ENTRADA) {
            printf("  Maximo permitido: %.0f grados por motor.\n", MAX_GRADOS_ENTRADA);
            continue;
        }
        return;
    }
}

static bool hay_movimiento(void)
{
    bool m = false;
    if (xSemaphoreTake(g_estado.mutex, portMAX_DELAY) == pdTRUE) {
        m = g_estado.moviendo_m1 || g_estado.moviendo_m2;
        xSemaphoreGive(g_estado.mutex);
    }
    return m;
}

// --- TAREA DE ENTRADA: pide grados, los convierte a pasos y arranca ambos motores a la vez ---
static void tarea_entrada_comandos(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(300));
    printf("\n=== Control de 2 motores NEMA17 + TB6600 ===\n");
    printf("Resolucion: %.2f grados/paso (%.0f pasos/vuelta)\n", GRADOS_POR_PASO, PASOS_POR_VUELTA);
    printf("Formato: M1,M2  (negativo = sentido contrario, 0 = ese motor quieto)\n\n");

    while (1) {
        float d1, d2;
        pedir_comando(&d1, &d2);

        // grados -> pasos (redondeado al paso más cercano)
        int32_t pasos1 = (int32_t)lroundf(d1 / GRADOS_POR_PASO);
        int32_t pasos2 = (int32_t)lroundf(d2 / GRADOS_POR_PASO);

        if (pasos1 == 0 && pasos2 == 0) {
            printf("Nada que mover.\n\n");
            continue;
        }

        printf("M1: %ld pasos (%.1f grados) | M2: %ld pasos (%.1f grados)\n",
               (long)pasos1, pasos1 * GRADOS_POR_PASO,
               (long)pasos2, pasos2 * GRADOS_POR_PASO);

        if (xSemaphoreTake(g_estado.mutex, portMAX_DELAY) == pdTRUE) {
            g_estado.pasos_m1 = pasos1;
            g_estado.pasos_m2 = pasos2;
            g_estado.moviendo_m1 = (pasos1 != 0);
            g_estado.moviendo_m2 = (pasos2 != 0);
            xSemaphoreGive(g_estado.mutex);
        }

        // Despertar ambos motores casi en el mismo instante => se mueven simultáneamente
        if (pasos1 != 0) xTaskNotifyGive(g_tarea_motor[0]);
        if (pasos2 != 0) xTaskNotifyGive(g_tarea_motor[1]);

        // Esperar a que ambos terminen antes de volver a preguntar
        while (hay_movimiento()) {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        printf("Movimiento completado.\n\n");
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

    gpio_set_level(TB6600_1_STEP, 0);
    gpio_set_level(TB6600_2_STEP, 0);
    gpio_set_level(TB6600_1_ENABLE, 0);
    gpio_set_level(TB6600_2_ENABLE, 0);

    // UART0 con driver, para poder leer lo que escribes en el monitor serial
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

    g_estado.mutex = xSemaphoreCreateMutex();
    g_estado.moviendo_m1 = false;
    g_estado.moviendo_m2 = false;

    static const config_motor_t motor1_cfg = {
        .pin_step = TB6600_1_STEP,
        .pin_dir = TB6600_1_DIR,
        .nombre = "M1",
        .motor_id = 1
    };

    static const config_motor_t motor2_cfg = {
        .pin_step = TB6600_2_STEP,
        .pin_dir = TB6600_2_DIR,
        .nombre = "M2",
        .motor_id = 2
    };

    // Cada motor en su propio núcleo para que los retardos no se interfieran entre sí
    xTaskCreatePinnedToCore(tarea_control_motor, "control_m1", 3072, (void *)&motor1_cfg, 5, &g_tarea_motor[0], 0);
    xTaskCreatePinnedToCore(tarea_control_motor, "control_m2", 3072, (void *)&motor2_cfg, 5, &g_tarea_motor[1], 1);
    xTaskCreate(tarea_entrada_comandos, "entrada_comandos", 4096, NULL, 5, NULL);
}