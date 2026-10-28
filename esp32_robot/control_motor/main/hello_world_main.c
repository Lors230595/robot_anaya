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
#include "esp_log.h"
#include "esp_attr.h"

// --- DEFINICIÓN DE PINES MOTORES ---
#define TB6600_1_STEP   GPIO_NUM_26
#define TB6600_1_DIR    GPIO_NUM_25
#define TB6600_1_ENABLE GPIO_NUM_27

#define TB6600_2_STEP   GPIO_NUM_14
#define TB6600_2_DIR    GPIO_NUM_12
#define TB6600_2_ENABLE GPIO_NUM_13

// Servo azul (p. ej., SG90): señal PWM en GPIO18.
// Alimenta el servo desde una fuente externa de 5 V y une las tierras (GND).
#define SERVO_GPIO       GPIO_NUM_18
#define SERVO_MIN_US     500
#define SERVO_MAX_US     2500
#define SERVO_PERIOD_US  20000

// --- PARÁMETROS DEL MECANISMO ---
#define PASOS_POR_VUELTA       400.0
#define GRADOS_POR_PASO        (360.0 / PASOS_POR_VUELTA)    // 0.9 grados por paso
#define MAX_GRADOS_ENTRADA     3600.0                        // límite de seguridad por comando

// --- PARÁMETROS DE LA RAMPA EXPONENCIAL ---
#define RETARDO_MIN_US         400    // Periodo a velocidad máxima (us)
#define RETARDO_MAX_US         2500   // Periodo de arranque/parada suave (us)
#define PASOS_ACEL             100     // Pasos dedicados a ACELERAR al inicio
#define PASOS_DESACEL          100     // Pasos dedicados a DESACELERAR al final
#define VELOCIDAD_FACTOR       0.60f   // 60% de la velocidad actual

// Perfil de la rampa:
//   RAMPA_EXPONENCIAL : la curva que ya usabas (la más suave al arrancar)
//   RAMPA_LINEAL      : aceleración constante (velocidad sube en línea recta)
//   RAMPA_S_CURVA     : arranque y llegada muy suaves (curva de coseno)
#define RAMPA_EXPONENCIAL      0
#define RAMPA_LINEAL           1
#define RAMPA_S_CURVA          2
#define PERFIL_RAMPA           RAMPA_EXPONENCIAL

// --- TEMPORIZADOR DE GENERACIÓN DE PASOS ---
#define TICK_US                20     // Resolución del timer (us). El pulso STEP dura 1 tick (20 us)
#define PAUSA_ENTRE_MOV_MS     50     // Reposo tras cada movimiento (deja asentar el rotor)

#define NUM_MOTORES            2

typedef struct {
    gpio_num_t pin_step;
    gpio_num_t pin_dir;
    gpio_num_t pin_enable;
    const char *nombre;
    gptimer_handle_t timer;

    // Compartido con la ISR
    volatile bool     activo;
    volatile bool     pulso_alto;
    volatile uint32_t pasos_total;
    volatile uint32_t pasos_hechos;
    volatile int32_t  espera_ticks;
    volatile uint32_t periodo_ticks;

    // Solo tarea principal
    int32_t pos_pasos;        // posición real acumulada (pasos con signo desde el arranque)
    double  ideal_grados;     // posición ideal acumulada (suma exacta de lo que pediste)
    int32_t pasos_orden;      // pasos con signo del movimiento actual
    int32_t pos_inicio;       // pos_pasos al inicio del movimiento actual
} motor_t;

static motor_t g_motores[NUM_MOTORES] = {
    { .pin_step = TB6600_1_STEP, .pin_dir = TB6600_1_DIR, .pin_enable = TB6600_1_ENABLE, .nombre = "M1" },
    { .pin_step = TB6600_2_STEP, .pin_dir = TB6600_2_DIR, .pin_enable = TB6600_2_ENABLE, .nombre = "M2" },
};

static float g_servo_grados = 90.0f;

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

static uint32_t g_tabla_acel[PASOS_ACEL + 1];       // periodo (ticks) según pasos ya dados
static uint32_t g_tabla_desacel[PASOS_DESACEL + 1]; // periodo (ticks) según pasos que faltan

// Periodo entre pasos (en ticks) para un avance de rampa f = 0..1
// f = 0 -> arranque (RETARDO_MAX_US, lento) | f = 1 -> velocidad máxima (RETARDO_MIN_US)
static uint32_t calcular_periodo_ticks(float f)
{
    float retardo_us;

#if PERFIL_RAMPA == RAMPA_EXPONENCIAL
    retardo_us = RETARDO_MIN_US + (RETARDO_MAX_US - RETARDO_MIN_US) * expf(-3.0f * f);
#else
    // Se interpola la VELOCIDAD (pasos/s) y luego se convierte a periodo
    float v0 = 1000000.0f / RETARDO_MAX_US;
    float v1 = 1000000.0f / RETARDO_MIN_US;
    float s = f;
#if PERFIL_RAMPA == RAMPA_S_CURVA
    s = 0.5f * (1.0f - cosf(3.14159265f * f));
#endif
    retardo_us = 1000000.0f / (v0 + (v1 - v0) * s);
#endif

    // Reduce la velocidad general a un 80% de la velocidad actual.
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

// Tablas de aceleración y desaceleración (la ISR no puede usar float, por eso se precalculan)
static void construir_tablas_rampa(void)
{
    construir_tabla(g_tabla_acel, PASOS_ACEL);
    construir_tabla(g_tabla_desacel, PASOS_DESACEL);
}

// --- ISR: genera los pulsos STEP de un motor de forma independiente ---
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

        // Rampa propia de este motor: acelera según los pasos dados y desacelera según los que faltan.
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

// --- ENTRADA POR CONSOLA (lectura bloqueante con eco) ---
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

// Interpreta "M1,M2,servo"; el tercer valor son grados incrementales.
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
        printf("Comando M1,M2,Servo (grados incrementales, ej: 100,150,30): ");
        fflush(stdout);
        leer_linea(buf, sizeof(buf));

        if (!parsear_comando(buf, g1, g2, g3)) {
            printf("  Comando invalido. Use: numero,numero,numero (ej: 100,150,30)\n");
            continue;
        }
        if (fabs(*g1) > MAX_GRADOS_ENTRADA || fabs(*g2) > MAX_GRADOS_ENTRADA ||
            fabs(*g3) > MAX_GRADOS_ENTRADA) {
            printf("  Maximo permitido: %.0f grados por motor.\n", MAX_GRADOS_ENTRADA);
            continue;
        }
        return;
    }
}

// --- SALIDA A TELEPLOT ---
static void imprimir_teleplot(void)
{
    for (int i = 0; i < NUM_MOTORES; i++) {
        motor_t *m = &g_motores[i];
        uint32_t hechos = m->pasos_hechos;
        uint32_t total = m->pasos_total;
        bool activo = m->activo;

        int32_t pos = m->pos_inicio + ((m->pasos_orden >= 0) ? (int32_t)hechos : -(int32_t)hechos);
        float velocidad = activo ? 1000000.0f / ((float)m->periodo_ticks * TICK_US) : 0.0f;
        double objetivo = (m->pos_inicio + m->pasos_orden) * GRADOS_POR_PASO;

        printf(">%s_Pos_Actual:%.2f\n", m->nombre, pos * GRADOS_POR_PASO);
        printf(">%s_Pasos_Restantes:%lu\n", m->nombre, (unsigned long)(total - hechos));
        printf(">%s_Objetivo:%.2f\n", m->nombre, objetivo);
        printf(">%s_Velocidad:%.1f\n", m->nombre, velocidad);
    }
    fflush(stdout);
}

static bool hay_motor_activo(void)
{
    for (int i = 0; i < NUM_MOTORES; i++) {
        if (g_motores[i].activo) return true;
    }
    return false;
}

// --- EJECUTA UN MOVIMIENTO SIMULTÁNEO ---
static void mover_motores(void)
{
    for (int i = 0; i < NUM_MOTORES; i++) {
        motor_t *m = &g_motores[i];
        uint32_t pasos = (uint32_t)abs((int)m->pasos_orden);

        m->pos_inicio = m->pos_pasos;
        m->pasos_hechos = 0;
        m->pasos_total = pasos;
        m->pulso_alto = false;
        m->periodo_ticks = g_tabla_acel[0];
        m->espera_ticks = (int32_t)g_tabla_acel[0];

        if (pasos > 0) {
            gpio_set_level(m->pin_dir, m->pasos_orden > 0 ? 1 : 0);
        }
    }

    vTaskDelay(pdMS_TO_TICKS(10));   // tiempo de establecimiento de DIR

    // Cada motor arranca y genera sus pasos con su propio temporizador.
    for (int i = 0; i < NUM_MOTORES; i++) {
        motor_t *m = &g_motores[i];
        m->activo = (m->pasos_total > 0);
        if (m->activo) {
            ESP_ERROR_CHECK(gptimer_set_raw_count(m->timer, 0));
            ESP_ERROR_CHECK(gptimer_start(m->timer));
        }
    }

    while (hay_motor_activo()) {
        imprimir_teleplot();
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    for (int i = 0; i < NUM_MOTORES; i++) {
        if (g_motores[i].pasos_total > 0) {
            ESP_ERROR_CHECK(gptimer_stop(g_motores[i].timer));
        }
        gpio_set_level(g_motores[i].pin_step, 0);
        g_motores[i].pos_pasos = g_motores[i].pos_inicio + g_motores[i].pasos_orden;
    }
    imprimir_teleplot();
    vTaskDelay(pdMS_TO_TICKS(PAUSA_ENTRE_MOV_MS));
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

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

    construir_tablas_rampa();

    for (int i = 0; i < NUM_MOTORES; i++) {
        motor_t *m = &g_motores[i];
        gptimer_config_t timer_cfg = {
            .clk_src = GPTIMER_CLK_SRC_DEFAULT,
            .direction = GPTIMER_COUNT_UP,
            .resolution_hz = 1000000,   // 1 tick = 1 us
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
    printf("\n=== Control de 2 motores NEMA17 + TB6600 ===\n");
    printf("Resolucion: %.2f grados/paso (%.0f pasos/vuelta)\n", GRADOS_POR_PASO, PASOS_POR_VUELTA);
    printf("Formato: M1,M2,Servo; servo incremental, limitado a 0..180 grados.\n\n");

    while (1) {
        double d[NUM_MOTORES + 1];
        pedir_comando(&d[0], &d[1], &d[2]);

        bool alguno = false;

        for (int i = 0; i < NUM_MOTORES; i++) {
            motor_t *m = &g_motores[i];

            // Compensación del redondeo: se acumula lo pedido y se calcula el paso objetivo
            // absoluto, así el error nunca pasa de medio paso (0.45 grados) por mucho que muevas.
            m->ideal_grados += (i == 1) ? d[i] * 1.0826 : d[i] * 1.8;
            int32_t objetivo_pasos = (int32_t)llround(m->ideal_grados / GRADOS_POR_PASO);
            m->pasos_orden = objetivo_pasos - m->pos_pasos;

            if (m->pasos_orden != 0) alguno = true;

            printf("%s: %ld pasos (%.2f grados) | pedido acumulado %.2f | posicion final %.2f | error %+.2f\n",
                   m->nombre, (long)m->pasos_orden, m->pasos_orden * GRADOS_POR_PASO,
                   m->ideal_grados, objetivo_pasos * GRADOS_POR_PASO,
                   objetivo_pasos * GRADOS_POR_PASO - m->ideal_grados);
        }

        if (fabs(d[2]) > 0.0) {
            servo_fijar_grados(g_servo_grados + (float)d[2]);
            printf("Servo: posicion %.1f grados\n", g_servo_grados);
        }

        if (!alguno) {
            printf("Nada que mover.\n\n");
            continue;
        }

        mover_motores();
        printf("Movimiento completado.\n\n");
    }
}