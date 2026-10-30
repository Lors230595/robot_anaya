#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "esp_attr.h"

// =====================================================================
//  PINES
// =====================================================================
#define M1_STEP_GPIO       GPIO_NUM_26
#define M1_DIR_GPIO        GPIO_NUM_25
#define M1_ENABLE_GPIO     GPIO_NUM_27

#define M2_STEP_GPIO       GPIO_NUM_14
#define M2_DIR_GPIO        GPIO_NUM_12
#define M2_ENABLE_GPIO     GPIO_NUM_13

#define SERVO_GPIO         GPIO_NUM_18

// =====================================================================
//  SENTIDOS (cambia si un motor va al reves)
// =====================================================================
#define M1_DIR_POSITIVO    1
#define M2_DIR_POSITIVO    1

// =====================================================================
//  MECANISMO
// =====================================================================
#define GRADOS_POR_PASO    (360.0 / 400.0)   // 0.9 grados por paso
#define RELACION_M2        1.09
#define MAX_GRADOS_ENTRADA 3600.0

// =====================================================================
//  MOTOR 1 (lazo abierto)
// =====================================================================
// =====================================================================
//  RAMPA
// =====================================================================
#define RETARDO_MIN_US     400
#define RETARDO_MAX_US     2500
#define PASOS_ACEL         100
#define PASOS_DESACEL      100
#define VELOCIDAD_FACTOR   0.5f     // menor = mas lento
#define TICK_US            20

#define RAMPA_EXPONENCIAL  0
#define RAMPA_LINEAL       1
#define RAMPA_S_CURVA      2
#define PERFIL_RAMPA       RAMPA_EXPONENCIAL

// =====================================================================
//  SERVO
// =====================================================================
#define SERVO_MIN_US       500
#define SERVO_MAX_US       2500
#define SERVO_PERIOD_US    20000

// =====================================================================
//  ESTRUCTURA DE MOTOR
// =====================================================================
typedef struct {
    gpio_num_t pin_step;
    gpio_num_t pin_dir;
    gpio_num_t pin_enable;
    int        dir_positivo;
    int        numero;
    gptimer_handle_t timer;
    TaskHandle_t tarea;

    // Compartido con la ISR y con su tarea de home
    volatile bool     activo;
    volatile bool     pulso_alto;
    volatile uint32_t pasos_total;
    volatile uint32_t pasos_hechos;
    volatile uint32_t periodo_ticks;
    volatile int32_t  espera_ticks;
    volatile int8_t   signo;

    // Solo tarea de motor
    int32_t pos_pasos;
} motor_t;

static motor_t M1 = {
    .pin_step = M1_STEP_GPIO, .pin_dir = M1_DIR_GPIO, .pin_enable = M1_ENABLE_GPIO,
    .dir_positivo = M1_DIR_POSITIVO, .numero = 1,
};

static motor_t M2 = {
    .pin_step = M2_STEP_GPIO, .pin_dir = M2_DIR_GPIO, .pin_enable = M2_ENABLE_GPIO,
    .dir_positivo = M2_DIR_POSITIVO, .numero = 2,
};

static TaskHandle_t g_tarea_main;
static volatile double g_inc1 = 0.0;
static volatile double g_inc2 = 0.0;
static double g_ideal2 = 0.0;
static float  g_servo_grados = 90.0f;

static uint32_t g_tabla_acel[PASOS_ACEL + 1];
static uint32_t g_tabla_desacel[PASOS_DESACEL + 1];

typedef enum { MOV_OK, MOV_RECHAZADO } resultado_mov_t;

// =====================================================================
//  RAMPA
// =====================================================================
static uint32_t calcular_periodo_ticks(float f)
{
    float retardo_us;

#if PERFIL_RAMPA == RAMPA_EXPONENCIAL
    retardo_us = RETARDO_MIN_US + (RETARDO_MAX_US - RETARDO_MIN_US) * expf(-3.0f * f);
#else
    float v0 = 1000000.0f / RETARDO_MAX_US;
    float v1 = 1000000.0f / RETARDO_MIN_US;
    float s = f;
#if PERFIL_RAMPA == RAMPA_S_CURVA
    s = 0.5f * (1.0f - cosf(3.14159265f * f));
#endif
    retardo_us = 1000000.0f / (v0 + (v1 - v0) * s);
#endif

    retardo_us /= VELOCIDAD_FACTOR;
    return (uint32_t)(retardo_us / TICK_US + 0.5f);
}

static void construir_tabla(uint32_t *tabla, int pasos)
{
    for (int i = 0; i <= pasos; i++) {
        float f = (pasos > 0) ? (float)i / (float)pasos : 1.0f;
        tabla[i] = calcular_periodo_ticks(f);
    }
}

static void construir_tablas_rampa(void)
{
    construir_tabla(g_tabla_acel, PASOS_ACEL);
    construir_tabla(g_tabla_desacel, PASOS_DESACEL);
}

// =====================================================================
//  ISR: genera pulsos STEP con rampa
// =====================================================================
static bool IRAM_ATTR timer_isr_cb(gptimer_handle_t timer,
                                   const gptimer_alarm_event_data_t *edata,
                                   void *user_ctx)
{
    motor_t *m = (motor_t *)user_ctx;
    if (!m->activo) return false;

    if (m->pulso_alto) {
        gpio_set_level(m->pin_step, 0);
        m->pulso_alto = false;

        uint32_t hechos = m->pasos_hechos + 1;
        m->pasos_hechos = hechos;

        if (hechos >= m->pasos_total) {
            m->activo = false;
            return false;
        }

        uint32_t faltan = m->pasos_total - hechos;
        uint32_t idx_a = (hechos < PASOS_ACEL) ? hechos : PASOS_ACEL;
        uint32_t idx_d = (faltan < PASOS_DESACEL) ? faltan : PASOS_DESACEL;
        uint32_t per_a = g_tabla_acel[idx_a];
        uint32_t per_d = g_tabla_desacel[idx_d];

        m->periodo_ticks = (per_a > per_d) ? per_a : per_d;
        m->espera_ticks = (int32_t)m->periodo_ticks - 1;
    } else if (--m->espera_ticks <= 0) {
        gpio_set_level(m->pin_step, 1);
        m->pulso_alto = true;
    }
    return false;
}

// =====================================================================
//  SERVO
// =====================================================================
static void servo_fijar_grados(float grados)
{
    if (grados < 0.0f) grados = 0.0f;
    if (grados > 180.0f) grados = 180.0f;

    float pulso_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * grados / 180.0f;
    uint32_t duty = (uint32_t)(pulso_us * 65535.0f / SERVO_PERIOD_US + 0.5f);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    g_servo_grados = grados;
}

// =====================================================================
//  MOVIMIENTO BASE DE UN MOTOR (con rampa)
// =====================================================================
static resultado_mov_t mover_pasos(motor_t *m, int32_t pasos)
{
    if (pasos == 0) return MOV_OK;

    int8_t signo = (pasos > 0) ? 1 : -1;

    gpio_set_level(m->pin_dir, (signo > 0) ? m->dir_positivo : !m->dir_positivo);

    m->signo = signo;
    m->pasos_total = (uint32_t)((pasos > 0) ? pasos : -pasos);
    m->pasos_hechos = 0;
    m->pulso_alto = false;
    m->periodo_ticks = g_tabla_acel[0];
    m->espera_ticks = (int32_t)g_tabla_acel[0];

    vTaskDelay(pdMS_TO_TICKS(10));   // establecimiento de DIR

    ESP_ERROR_CHECK(gptimer_set_raw_count(m->timer, 0));
    m->activo = true;
    ESP_ERROR_CHECK(gptimer_start(m->timer));

    while (m->activo) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_ERROR_CHECK(gptimer_stop(m->timer));
    gpio_set_level(m->pin_step, 0);
    m->pulso_alto = false;

    m->pos_pasos += signo * (int32_t)m->pasos_hechos;

    return MOV_OK;
}

// =====================================================================
//  FUNCION MOTOR 1: lazo abierto por grados incrementales
// =====================================================================
static void mover_motor1(double incremento)
{
    if (fabs(incremento) < 1e-9) return;

    double incremento_ajustado = incremento * 1.8;
    int32_t pasos = (int32_t)llround(incremento_ajustado / GRADOS_POR_PASO);
    printf("M1: %.2f x 1.8 = %.2f grados -> %ld pasos\n",
           incremento, incremento_ajustado, (long)pasos);
    mover_pasos(&M1, pasos);
    printf("M1 posicion estimada: %.2f grados\n", M1.pos_pasos * GRADOS_POR_PASO);
}

// =====================================================================
//  FUNCION MOTOR 2: lazo abierto por pasos
//  grados pedidos * RELACION_M2 -> pasos (cada paso = 0.9 grados)
// =====================================================================
static void mover_motor2(double pedido)
{
    if (fabs(pedido) < 1e-9) return;

    g_ideal2 += pedido;
    int32_t obj = (int32_t)llround(g_ideal2 * RELACION_M2 / GRADOS_POR_PASO);
    int32_t pasos = obj - M2.pos_pasos;

    printf("M2: %.2f x %.3f = %.2f grados -> %ld pasos (%.2f grados)\n",
           pedido, RELACION_M2, pedido * RELACION_M2, (long)pasos,
           pasos * GRADOS_POR_PASO);

    resultado_mov_t r = mover_pasos(&M2, pasos);

    printf("M2 posicion: %.2f grados\n", M2.pos_pasos * GRADOS_POR_PASO / RELACION_M2);
}

// =====================================================================
//  TAREAS DE MOVIMIENTO (una por motor, en paralelo)
// =====================================================================
static void tarea_motor1(void *arg)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        mover_motor1(g_inc1);
        xTaskNotifyGive(g_tarea_main);
    }
}

static void tarea_motor2(void *arg)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        mover_motor2(g_inc2);
        xTaskNotifyGive(g_tarea_main);
    }
}

// =====================================================================
//  ENTRADA POR CONSOLA
// =====================================================================
static void leer_linea(char *buf, size_t max)
{
    size_t len = 0;
    uint8_t c;

    while (1) {
        if (uart_read_bytes(UART_NUM_0, &c, 1, portMAX_DELAY) != 1) continue;

        if (c == '\r' || c == '\n') {
            if (len == 0) continue;
            uart_write_bytes(UART_NUM_0, "\r\n", 2);
            break;
        } else if (c == 0x08 || c == 0x7F) {
            if (len > 0) {
                len--;
                uart_write_bytes(UART_NUM_0, "\b \b", 3);
            }
        } else if (len < max - 1 && c >= 32 && c < 127) {
            buf[len++] = (char)c;
            uart_write_bytes(UART_NUM_0, (const char *)&c, 1);
        }
    }
    buf[len] = '\0';
}

static bool parsear_comando(const char *texto, double *v1, double *v2, double *v3)
{
    char *fin = NULL;
    const char *p = texto;

    *v1 = strtod(p, &fin);
    if (fin == p) return false;
    p = fin;
    while (*p == ' ') p++;
    if (*p != ',') return false;
    p++;

    *v2 = strtod(p, &fin);
    if (fin == p) return false;
    p = fin;
    while (*p == ' ') p++;
    if (*p != ',') return false;
    p++;

    *v3 = strtod(p, &fin);
    if (fin == p) return false;
    p = fin;
    while (*p == ' ') p++;
    if (*p != '\0') return false;

    return isfinite(*v1) && isfinite(*v2) && isfinite(*v3);
}

static void pedir_comando(double *g1, double *g2, double *g3)
{
    char buf[48];

    while (1) {
        printf("ingresa un angulo (30,0,0): ");
        fflush(stdout);
        leer_linea(buf, sizeof(buf));

        if (!parsear_comando(buf, g1, g2, g3)) {
            printf("  Formato invalido. Use: numero,numero,numero (ej: 30,0,0)\n");
            continue;
        }
        if (fabs(*g1) > MAX_GRADOS_ENTRADA || fabs(*g2) > MAX_GRADOS_ENTRADA ||
            fabs(*g3) > MAX_GRADOS_ENTRADA) {
            printf("  Maximo permitido: %.0f grados por valor.\n", MAX_GRADOS_ENTRADA);
            continue;
        }
        return;
    }
}

// =====================================================================
//  app_main
// =====================================================================
void app_main(void)
{
    // Salidas de motores
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << M1_STEP_GPIO) | (1ULL << M1_DIR_GPIO) | (1ULL << M1_ENABLE_GPIO) |
                        (1ULL << M2_STEP_GPIO) | (1ULL << M2_DIR_GPIO) | (1ULL << M2_ENABLE_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&out_cfg));

    gpio_set_level(M1_STEP_GPIO, 0);
    gpio_set_level(M2_STEP_GPIO, 0);
    gpio_set_level(M1_ENABLE_GPIO, 0);
    gpio_set_level(M2_ENABLE_GPIO, 0);

    // Servo (PWM)
    ledc_timer_config_t servo_timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_16_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&servo_timer_cfg));

    ledc_channel_config_t servo_channel_cfg = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&servo_channel_cfg));
    servo_fijar_grados(g_servo_grados);

    // UART consola
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

    construir_tablas_rampa();

    // Un timer por motor
    motor_t *motores[2] = { &M1, &M2 };
    for (int i = 0; i < 2; i++) {
        motor_t *m = motores[i];
        gptimer_config_t timer_cfg = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000,
        };
        ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &m->timer));

        gptimer_alarm_config_t alarm_cfg = {
            .alarm_count = TICK_US,
            .reload_count = 0,
            .flags.auto_reload_on_alarm = true,
        };
        ESP_ERROR_CHECK(gptimer_set_alarm_action(m->timer, &alarm_cfg));

        gptimer_event_callbacks_t cbs = { .on_alarm = timer_isr_cb };
        ESP_ERROR_CHECK(gptimer_register_event_callbacks(m->timer, &cbs, m));
        ESP_ERROR_CHECK(gptimer_enable(m->timer));
    }

    vTaskDelay(pdMS_TO_TICKS(300));

    g_tarea_main = xTaskGetCurrentTaskHandle();

    // Tareas de movimiento
    xTaskCreate(tarea_motor1, "motor1", 4096, NULL, 5, &M1.tarea);
    xTaskCreate(tarea_motor2, "motor2", 4096, NULL, 5, &M2.tarea);

    printf("\n=== Control brazo: M1 + M2 (relacion %.3f) + Servo ===\n", RELACION_M2);
    printf("Resolucion: %.2f grados/paso\n\n", GRADOS_POR_PASO);

    while (1) {
        double d[3];
        pedir_comando(&d[0], &d[1], &d[2]);

        if (fabs(d[2]) > 0.0) {
            servo_fijar_grados(g_servo_grados + (float)d[2]);
            printf("Servo: posicion %.1f grados\n", g_servo_grados);
        }

        if (fabs(d[0]) == 0.0 && fabs(d[1]) == 0.0) {
            printf("Nada que mover.\n\n");
            continue;
        }

        g_inc1 = d[0];
        g_inc2 = d[1];
        xTaskNotifyGive(M1.tarea);
        xTaskNotifyGive(M2.tarea);

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        printf("Movimiento completado.\n\n");
    }
}