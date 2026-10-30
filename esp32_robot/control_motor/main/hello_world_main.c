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
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_attr.h"

// --- DEFINICIÓN DE PINES MOTORES ---
#define TB6600_1_STEP   GPIO_NUM_26
#define TB6600_1_DIR    GPIO_NUM_25
#define TB6600_1_ENABLE GPIO_NUM_27

#define TB6600_2_STEP   GPIO_NUM_14
#define TB6600_2_DIR    GPIO_NUM_12
#define TB6600_2_ENABLE GPIO_NUM_13

// --- LIMIT SWITCHES (HOME) ---
// Cableado: COM -> 3.3V, NA -> pin del ESP + resistencia 1k a GND (pull-down externa)
// En tu montaje se observó la lógica invertida: presionado = 0 | libre = 1
// (si un switch sale al revés, cambia su LIMIT_x_NIVEL_PRESIONADO)
// Nota: GPIO34 y GPIO35 son solo entrada y NO tienen pull interno, por eso se usa la resistencia externa.
#define LIMIT_1_PIN             GPIO_NUM_33
#define LIMIT_2_PIN             GPIO_NUM_34
// Nivel lógico cuando CADA switch está presionado (home). Se configuran por separado
// por si un switch está cableado distinto al otro (NA vs NC).
#define LIMIT_1_NIVEL_PRESIONADO  0
#define LIMIT_2_NIVEL_PRESIONADO  0
#define LIMIT_DEBUG             0      // 1 = muestra el nivel crudo de cada pin (solo para diagnosticar)

// --- POTENCIÓMETRO (ángulo real de M1) ---
// Cableado: pin derecho -> 3.3V | pin central (cursor) -> POT_PIN | pin izquierdo -> GND
// OJO: el GPIO17 NO tiene ADC en el ESP32. El cursor debe ir a un pin ADC1 (32,33,34,35,36,39).
// GPIO35 está libre y es solo entrada, ideal para esto.
#define POT_PIN                 35
#define POT_RECORRIDO_GRADOS    270.0f   // vuelta total del potenciómetro
#define POT_RAW_HOME            4095     // lectura ADC con el pot totalmente a la DERECHA (home = 0°)
#define POT_RAW_FIN             0        // lectura ADC con el pot totalmente a la IZQUIERDA (270°)
#define POT_MUESTRAS            16       // lecturas que se promedian para reducir ruido
#define POT_ANGULO_OBJETIVO     100.0f   // ángulo que se quiere confirmar
#define POT_TOLERANCIA          2.0f     // tolerancia en grados
#define POT_POLL_MS             50       // cada cuánto la tarea del pot lee el ángulo
#define POT_CAMBIO_MIN_GRADOS   1.0f     // el pot solo se imprime cuando cambia al menos esto

// --- QUÉ SE MUESTRA EN CONSOLA ---
#define MOSTRAR_DETALLE         0      // 1 = muestra pasos por motor y "Movimiento completado"
#define MOSTRAR_TELEPLOT        0      // 1 = manda los datos >M1_Pos_Actual etc. durante el movimiento
#define LIMIT_POLL_MS           10     // cada cuánto se lee el pin (con tick de 100 Hz el mínimo real es 10 ms)
#define LIMIT_DEBOUNCE_MS       30     // tiempo que debe mantenerse estable un cambio (antirrebote)

// --- PARÁMETROS DEL MECANISMO ---
#define PASOS_POR_VUELTA       400.0
#define GRADOS_POR_PASO        (360.0 / PASOS_POR_VUELTA)    // 0.9 grados por paso
#define MAX_GRADOS_ENTRADA     3600.0                        // límite de seguridad por comando

// --- PARÁMETROS DE LA RAMPA EXPONENCIAL ---
#define RETARDO_MIN_US         400    // Periodo a velocidad máxima (us)
#define RETARDO_MAX_US         2500   // Periodo de arranque/parada suave (us)
#define PASOS_ACEL             100     // Pasos dedicados a ACELERAR al inicio
#define PASOS_DESACEL          100     // Pasos dedicados a DESACELERAR al final

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
    gpio_num_t pin_limit;     // limit switch de home de este motor
    int        nivel_presionado; // nivel lógico del pin cuando este limit está presionado
    const char *nombre;
    gptimer_handle_t timer;

    // Compartido con la ISR
    volatile bool     activo;
    volatile bool     pulso_alto;
    volatile uint32_t pasos_total;
    volatile uint32_t pasos_hechos;
    volatile int32_t  espera_ticks;
    volatile uint32_t periodo_ticks;

    // Estado del limit switch (true = presionado / en home), lo escribe la tarea de limits
    volatile bool     en_home;

    // Solo tarea principal
    int32_t pos_pasos;        // posición real acumulada (pasos con signo desde el arranque)
    double  ideal_grados;     // posición ideal acumulada (suma exacta de lo que pediste)
    int32_t pasos_orden;      // pasos con signo del movimiento actual
    int32_t pos_inicio;       // pos_pasos al inicio del movimiento actual
} motor_t;

static motor_t g_motores[NUM_MOTORES] = {
    { .pin_step = TB6600_1_STEP, .pin_dir = TB6600_1_DIR, .pin_enable = TB6600_1_ENABLE, .pin_limit = LIMIT_1_PIN, .nivel_presionado = LIMIT_1_NIVEL_PRESIONADO, .nombre = "M1" },
    { .pin_step = TB6600_2_STEP, .pin_dir = TB6600_2_DIR, .pin_enable = TB6600_2_ENABLE, .pin_limit = LIMIT_2_PIN, .nivel_presionado = LIMIT_2_NIVEL_PRESIONADO, .nombre = "M2" },
};

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

// --- LIMIT SWITCHES ---
// Lee el limit de UN motor (true = presionado) según su propio pin y su propia polaridad
static inline bool leer_limit_motor(const motor_t *m)
{
    return gpio_get_level(m->pin_limit) == m->nivel_presionado;
}

// Línea de comando en edición (compartida para poder volver a mostrar el prompt tras un aviso de limit)
#define TEXTO_PROMPT "Comando M1,M2 (grados a mover, ej: 100,150): "
static char            g_linea[48];
static volatile size_t g_linea_len = 0;
static volatile bool   g_esperando_cmd = false;

// --- POTENCIÓMETRO ---
static adc_oneshot_unit_handle_t g_adc;
static adc_channel_t             g_pot_canal;

static void iniciar_pot(void)
{
    adc_unit_t unidad;
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(POT_PIN, &unidad, &g_pot_canal));

    adc_oneshot_unit_init_cfg_t cfg_unidad = { .unit_id = unidad };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&cfg_unidad, &g_adc));

    adc_oneshot_chan_cfg_t cfg_canal = {
        .atten = ADC_ATTEN_DB_12,           // rango ~0 a 3.1 V
        .bitwidth = ADC_BITWIDTH_DEFAULT,   // 12 bits (0..4095)
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc, g_pot_canal, &cfg_canal));
}

// Devuelve el ángulo del pot en grados (home = totalmente a la derecha = 0°) y el valor ADC promedio
static float leer_angulo_pot(int *raw_prom)
{
    int suma = 0, n = 0, v = 0;
    for (int i = 0; i < POT_MUESTRAS; i++) {
        if (adc_oneshot_read(g_adc, g_pot_canal, &v) == ESP_OK) {
            suma += v;
            n++;
        }
    }
    int raw = (n > 0) ? suma / n : 0;
    if (raw_prom) *raw_prom = raw;

    return POT_RECORRIDO_GRADOS * (float)(POT_RAW_HOME - raw) / (float)(POT_RAW_HOME - POT_RAW_FIN);
}

// Último ángulo medido por la tarea del pot (independiente de los motores y de los limits)
static volatile float g_pot_grados = 0.0f;

// Tarea del potenciómetro: solo lee el pot y guarda su valor. Imprime "pot=xx.x" cuando
// cambia, pero no mientras algún motor se está moviendo.
static void tarea_pot(void *arg)
{
    TickType_t t_poll = pdMS_TO_TICKS(POT_POLL_MS);
    if (t_poll < 1) t_poll = 1;

    float ultimo = leer_angulo_pot(NULL);
    g_pot_grados = ultimo;

    while (1) {
        vTaskDelay(t_poll);
        float ang = leer_angulo_pot(NULL);
        g_pot_grados = ang;

        bool moviendo = g_motores[0].activo || g_motores[1].activo;
        if (!moviendo && fabsf(ang - ultimo) >= POT_CAMBIO_MIN_GRADOS) {
            ultimo = ang;
            printf("\npot=%.1f\n", ang);
            if (g_esperando_cmd) {
                printf(TEXTO_PROMPT "%.*s", (int)g_linea_len, g_linea);
            }
            fflush(stdout);
        }
    }
}

// Imprime el estado de home de cada motor (si = limit presionado)
static void imprimir_estado_limits(void)
{
    float ang_pot = g_pot_grados;   // lo mantiene la tarea del pot

    printf("\nM1 home: %s\nM2 home: %s\npot=%.1f\n",
           g_motores[0].en_home ? "si" : "no",
           g_motores[1].en_home ? "si" : "no",
           ang_pot);
#if LIMIT_DEBUG
    printf("(crudo: GPIO%d=%d GPIO%d=%d)\n",
           (int)g_motores[0].pin_limit, gpio_get_level(g_motores[0].pin_limit),
           (int)g_motores[1].pin_limit, gpio_get_level(g_motores[1].pin_limit));
#endif

    // Si se está esperando un comando, vuelve a mostrar el prompt y lo que ya se había escrito
    if (g_esperando_cmd) {
        printf(TEXTO_PROMPT "%.*s", (int)g_linea_len, g_linea);
    }
    fflush(stdout);
}

// Una tarea POR MOTOR: cada una vigila solo su propio limit, con su propio antirrebote,
// y solo modifica el en_home de su motor. Imprime cuando SU estado cambia.
static void tarea_limit_motor(void *arg)
{
    motor_t *m = (motor_t *)arg;

    // Mínimo 1 tick (vTaskDelay(0) no bloquea y dejaría sin CPU a IDLE -> watchdog)
    TickType_t t_poll = pdMS_TO_TICKS(LIMIT_POLL_MS);
    if (t_poll < 1) t_poll = 1;
    int cuentas_necesarias = LIMIT_DEBOUNCE_MS / (int)(t_poll * portTICK_PERIOD_MS);
    if (cuentas_necesarias < 1) cuentas_necesarias = 1;

    // Estado inicial (lo imprime app_main una vez que ambas tareas arrancaron)
    bool candidato = leer_limit_motor(m);
    int  cuenta = cuentas_necesarias;
    m->en_home = candidato;
#if LIMIT_DEBUG
    int raw_prev = gpio_get_level(m->pin_limit);
#endif

    while (1) {
        vTaskDelay(t_poll);
        bool lect = leer_limit_motor(m);

#if LIMIT_DEBUG
        int raw = gpio_get_level(m->pin_limit);
        if (raw != raw_prev) {
            raw_prev = raw;
            printf("\n[DEBUG] %s GPIO%d = %d\n", m->nombre, (int)m->pin_limit, raw);
        }
#endif

        if (lect == candidato) {
            if (cuenta < cuentas_necesarias) cuenta++;
        } else {
            candidato = lect;
            cuenta = 1;
        }

        // El cambio se acepta solo si se mantuvo estable el tiempo de antirrebote
        if (cuenta >= cuentas_necesarias && candidato != m->en_home) {
            m->en_home = candidato;
            imprimir_estado_limits();
        }
    }
}

// --- ENTRADA POR CONSOLA (lectura bloqueante con eco) ---
static void leer_linea(char *buf, size_t max)
{
    uint8_t c;
    g_linea_len = 0;

    while (1) {
        if (uart_read_bytes(UART_NUM_0, &c, 1, portMAX_DELAY) != 1) continue;

        if (c == '\r' || c == '\n') {
            if (g_linea_len == 0) continue;
            uart_write_bytes(UART_NUM_0, "\r\n", 2);
            break;
        } else if (c == 0x08 || c == 0x7F) {
            if (g_linea_len > 0) {
                g_linea_len--;
                uart_write_bytes(UART_NUM_0, "\b \b", 3);
            }
        } else if (g_linea_len < max - 1 && c >= 32 && c < 127) {
            buf[g_linea_len++] = (char)c;
            uart_write_bytes(UART_NUM_0, (const char *)&c, 1);
        }
    }
    buf[g_linea_len] = '\0';
}

// Interpreta "M1,M2" (ej: 100,150). Decimales con punto: 45.5,90
static bool parsear_comando(const char *texto, double *v1, double *v2)
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
    if (*p != '\0') return false;

    return isfinite(*v1) && isfinite(*v2);
}

static void pedir_comando(double *g1, double *g2)
{
    char *buf = g_linea;

    while (1) {
        g_linea_len = 0;
        imprimir_estado_limits();      // M1 home / M2 home justo antes de pedir los ángulos
        printf(TEXTO_PROMPT);
        fflush(stdout);
        g_esperando_cmd = true;
        leer_linea(buf, sizeof(g_linea));
        g_esperando_cmd = false;

        if (!parsear_comando(buf, g1, g2)) {
            printf("  Comando invalido. Use: numero,numero (ej: 100,150)\n");
            continue;
        }
        if (fabs(*g1) > MAX_GRADOS_ENTRADA || fabs(*g2) > MAX_GRADOS_ENTRADA) {
            printf("  Maximo permitido: %.0f grados por motor.\n", MAX_GRADOS_ENTRADA);
            continue;
        }
        return;
    }
}

// --- SALIDA A TELEPLOT ---
static void imprimir_teleplot(void)
{
    if (!MOSTRAR_TELEPLOT) return;

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

    // Limit switches como entradas (la pull-down es la resistencia externa de 1k)
    // GPIO33 sí tiene pull-down interna: se activa como respaldo para que no quede flotando
    gpio_config_t config_limit1 = {
        .pin_bit_mask = (1ULL << LIMIT_1_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config_limit1);

    // GPIO34 es solo entrada y NO tiene pull interno: aquí la resistencia externa a GND es obligatoria
    gpio_config_t config_limit2 = {
        .pin_bit_mask = (1ULL << LIMIT_2_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config_limit2);

    gpio_set_level(TB6600_1_STEP, 0);
    gpio_set_level(TB6600_2_STEP, 0);
    gpio_set_level(TB6600_1_ENABLE, 0);
    gpio_set_level(TB6600_2_ENABLE, 0);

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

    iniciar_pot();

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
    printf("Formato: M1,M2  (negativo = sentido contrario, 0 = ese motor quieto)\n");

    // Tarea que monitorea los limit switches (imprime el estado inicial y cada cambio)
    xTaskCreate(tarea_pot, "pot", 4096, NULL, 5, NULL);
    xTaskCreate(tarea_limit_motor, "limit_M1", 4096, &g_motores[0], 5, NULL);
    xTaskCreate(tarea_limit_motor, "limit_M2", 4096, &g_motores[1], 5, NULL);
    vTaskDelay(pdMS_TO_TICKS(50));
    imprimir_estado_limits();   // estado inicial de ambos
    printf("\n");

    while (1) {
        double d[NUM_MOTORES];
        pedir_comando(&d[0], &d[1]);

        bool alguno = false;

        for (int i = 0; i < NUM_MOTORES; i++) {
            motor_t *m = &g_motores[i];

            // Compensación del redondeo: se acumula lo pedido y se calcula el paso objetivo
            // absoluto, así el error nunca pasa de medio paso (0.45 grados) por mucho que muevas.
            m->ideal_grados += (i == 1) ? d[i] * 1.0825 : d[i] * 1.8;
            int32_t objetivo_pasos = (int32_t)llround(m->ideal_grados / GRADOS_POR_PASO);
            m->pasos_orden = objetivo_pasos - m->pos_pasos;

            if (m->pasos_orden != 0) alguno = true;

            if (MOSTRAR_DETALLE) {
                printf("%s: %ld pasos (%.2f grados) | pedido acumulado %.2f | posicion final %.2f | error %+.2f\n",
                       m->nombre, (long)m->pasos_orden, m->pasos_orden * GRADOS_POR_PASO,
                       m->ideal_grados, objetivo_pasos * GRADOS_POR_PASO,
                       objetivo_pasos * GRADOS_POR_PASO - m->ideal_grados);
            }
        }

        if (!alguno) {
            if (MOSTRAR_DETALLE) printf("Nada que mover.\n\n");
            continue;
        }

        mover_motores();
        if (MOSTRAR_DETALLE) printf("Movimiento completado.\n\n");
    }
}