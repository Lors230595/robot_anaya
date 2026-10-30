#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// =====================================================================
//  PRUEBA DEL LIMIT SWITCH (solo lectura, sin motores)
//  Imprime en pantalla SOLO cuando el estado cambia.
// =====================================================================

// --- LIMIT SWITCH ---
// Pin con el que se probo y funciono: GPIO34 (eslabon 2).
// Para el eslabon 1 cambia a GPIO_NUM_33 y el nombre a "E1".
#define LIMIT_GPIO           GPIO_NUM_34
#define LIMIT_NOMBRE         "E1"

// Nivel que lee el pin cuando el switch ESTA PRESIONADO (0 o 1).
// Si sale "si" sin presionar, invierte este valor.
#define LIMIT_NIVEL_ACTIVO   1

// --- AJUSTE DE RESPUESTA ---
// IMPORTANTE: el tick de FreeRTOS suele ser de 10 ms, asi que el periodo
// debe ser de 10 ms o mas. Un valor menor (ej. 5 ms) se redondea a 0 ticks,
// la tarea nunca cede la CPU y salta el watchdog.
#define PERIODO_LECTURA_MS   10    // cada cuanto se lee el pin
#define LECTURAS_ESTABLES    2    // lecturas iguales seguidas para aceptar un cambio (antirrebote)

// Imprime el estado en pantalla
static void imprimir_estado(bool presionado, int nivel)
{
    printf("%s home: %s   (nivel crudo del pin: %d)\n",
           LIMIT_NOMBRE, presionado ? "No" : "si", nivel);
}

void app_main(void)
{
    // Pin como entrada. GPIO34 no tiene pull-up/pull-down interno;
    // el modulo ya trae su propia resistencia.
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LIMIT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    vTaskDelay(pdMS_TO_TICKS(300));
    printf("\n=== Prueba limit switch (%s) ===\n", LIMIT_NOMBRE);

    // Estado inicial: se imprime una sola vez
    int nivel_estable = gpio_get_level(LIMIT_GPIO);
    imprimir_estado(nivel_estable == LIMIT_NIVEL_ACTIVO, nivel_estable);

    // Nivel que se evalua como posible cambio y cuantas veces seguidas se ha visto
    int nivel_candidato = nivel_estable;
    int contador = 0;

    while (1) {
        int nivel = gpio_get_level(LIMIT_GPIO);

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
                imprimir_estado(nivel_estable == LIMIT_NIVEL_ACTIVO, nivel_estable);
            }
        } else {
            // Sin cambio: se descarta cualquier cambio a medias (rebote o ruido)
            contador = 0;
        }

        // Espera hasta la siguiente lectura (cede la CPU a las demas tareas)
        vTaskDelay(pdMS_TO_TICKS(PERIODO_LECTURA_MS));
    }
}