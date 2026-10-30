#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// =====================================================================
//  PRUEBA DE LOS LIMIT SWITCHES (solo lectura, sin motores)
//  Dos tareas TOTALMENTE INDEPENDIENTES: limit1 (eslabon 1) y limit2 (eslabon 2).
//  - Cada tarea lee SOLO su pin y usa SOLO sus propias variables.
//  - No comparten nada: una no sabe que hace la otra.
//  - Cada tarea imprime SOLO cuando SU switch cambia.
//
//  Ejemplo:
//     E1home=no         (estado inicial de E1)
//     E2home=no         (estado inicial de E2)
//     E1home=si         (presionas E1)
//     E2home=si         (presionas E2)
//     E1home=no         (sueltas E1)
// =====================================================================

// --- PINES ---
#define LIMIT1_GPIO          GPIO_NUM_32    // eslabon 1
#define LIMIT2_GPIO          GPIO_NUM_34    // eslabon 2

// Nivel que lee el pin cuando el switch ESTA PRESIONADO ("si" en pantalla).
// Tus modulos dan 0 al presionar y 1 en reposo.
// Si sale "si" sin presionar, invierte este valor.
#define LIMIT_NIVEL_ACTIVO   0

// --- AJUSTE DE RESPUESTA ---
// El periodo debe ser de 10 ms o mas (tick de FreeRTOS), si no salta el watchdog.
#define PERIODO_LECTURA_MS   10    // cada cuanto se lee el pin
#define LECTURAS_ESTABLES    2     // lecturas iguales seguidas para aceptar un cambio (antirrebote)

// --- CONFIGURACION DE LAS TAREAS ---
#define TAREA_STACK          3072
#define TAREA_PRIORIDAD      5

// Datos que recibe cada tarea (solo el pin y el nombre; el estado es local de cada tarea)
typedef struct {
    gpio_num_t  pin;
    const char *nombre;
} limit_cfg_t;

static const limit_cfg_t CFG_LIMIT1 = { .pin = LIMIT1_GPIO, .nombre = "E1" };
static const limit_cfg_t CFG_LIMIT2 = { .pin = LIMIT2_GPIO, .nombre = "E2" };

// ---------------------------------------------------------------------
//  TAREA: vigila UN limit switch. Se crea una vez por cada eslabon.
//  Todas sus variables son locales: cada tarea tiene las suyas.
// ---------------------------------------------------------------------
static void tarea_limit(void *arg)
{
    const limit_cfg_t *cfg = (const limit_cfg_t *)arg;

    // Estado inicial: se imprime una sola vez
    int nivel_estable = gpio_get_level(cfg->pin);
    printf("%shome=%s\n", cfg->nombre, (nivel_estable == LIMIT_NIVEL_ACTIVO) ? "si" : "no");

    // Nivel que se evalua como posible cambio y cuantas veces seguidas se ha visto
    int nivel_candidato = nivel_estable;
    int contador = 0;

    while (1) {
        int nivel = gpio_get_level(cfg->pin);

        if (nivel != nivel_estable) {
            // El pin es distinto al estado confirmado: se cuenta cuantas veces seguidas
            if (nivel == nivel_candidato) {
                contador++;
            } else {
                nivel_candidato = nivel;
                contador = 1;
            }
            // Si el cambio se mantuvo suficiente tiempo, se acepta y se imprime
            if (contador >= LECTURAS_ESTABLES) {
                nivel_estable = nivel;
                contador = 0;
                printf("%shome=%s\n", cfg->nombre, (nivel_estable == LIMIT_NIVEL_ACTIVO) ? "si" : "no");
            }
        } else {
            // Sin cambio: se descarta cualquier cambio a medias (rebote o ruido)
            contador = 0;
        }

        // Espera hasta la siguiente lectura (cede la CPU a las demas tareas)
        vTaskDelay(pdMS_TO_TICKS(PERIODO_LECTURA_MS));
    }
}

void app_main(void)
{
    // Pines como entrada. El modulo ya trae su propia resistencia;
    // GPIO34 no tiene pull-up/pull-down interno.
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LIMIT1_GPIO) | (1ULL << LIMIT2_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    vTaskDelay(pdMS_TO_TICKS(300));
    printf("\n=== Prueba limit switches: E1 (GPIO32) y E2 (GPIO34) ===\n");

    // Dos tareas independientes, una por switch
    xTaskCreate(tarea_limit, "limit1", TAREA_STACK, (void *)&CFG_LIMIT1, TAREA_PRIORIDAD, NULL);
    xTaskCreate(tarea_limit, "limit2", TAREA_STACK, (void *)&CFG_LIMIT2, TAREA_PRIORIDAD, NULL);
}