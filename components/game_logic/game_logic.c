#include "game_logic.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "system_common.h"
#include <stdio.h>

// O sistema define o pino GPIO conectado ao sensor indutivo.
#define INDUCTIVE_SENSOR_PIN GPIO_NUM_13 

/**
 * @brief Inicializa o hardware necessário para a lógica do jogo.
 * O sistema configura o pino do sensor como entrada e habilita o resistor de pull-up interno,
 * garantindo estabilidade de sinal para sensores do tipo coletor aberto (NPN).
 */
void game_logic_init(void) {
    gpio_set_direction(INDUCTIVE_SENSOR_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INDUCTIVE_SENSOR_PIN, GPIO_PULLUP_ONLY);
}

/**
 * @brief Tarefa responsável por gerenciar os estados de vitória e reinício do jogo.
 * O sistema monitora continuamente o sensor indutivo e aplica temporizadores para evitar
 * leituras falsas (debounce) tanto para a condição de vitória quanto para o reset.
 */
void win_check_task(void *pvParameters) {
    // O sistema utiliza este contador para acumular o tempo em que a bola está presente no sensor.
    int detected_counter_ms = 0; 

    // O sistema utiliza este contador para acumular o tempo em que a bola está ausente após a vitória.
    int absent_counter_ms = 0;   

    for (;;) {
        // O sistema lê o nível lógico do pino. 
        // Nível 0 indica que o sensor indutivo detectou metal (bola).
        // Nível 1 indica que não há metal próximo.
        int estado = gpio_get_level(INDUCTIVE_SENSOR_PIN); 
        bool currently_won = false;

        // O sistema adquire o Mutex para ler de forma segura o estado atual do jogo na variável global.
        if (xSemaphoreTake(data_mutex, portMAX_DELAY)) {
            currently_won = global_data.game_won;
            xSemaphoreGive(data_mutex);
        }

        if (!currently_won) {
            // --- MODO: TENTANDO GANHAR ---
            // O sistema zera o contador de ausência, pois o foco agora é detectar a vitória.
            absent_counter_ms = 0; 

            if (estado == 0) { 
                // Se a bola for detectada, o sistema incrementa o contador de tempo em 20ms.
                detected_counter_ms += 20;
            } else {
                // Se a detecção falhar (bola saiu), o sistema reinicia a contagem imediatamente.
                detected_counter_ms = 0;
            }

            // O sistema verifica se a bola permaneceu estável no sensor por 2 segundos (2000ms).
            if (detected_counter_ms >= 2000) { 
                if (xSemaphoreTake(data_mutex, portMAX_DELAY)) {
                    // O sistema atualiza o estado global para "Vitoria".
                    global_data.game_won = true;
                    printf(">>> VITORIA CONFIRMADA! <<<\n");
                    xSemaphoreGive(data_mutex);
                }
                detected_counter_ms = 0; // O sistema reinicia o contador para evitar múltiplos disparos.
            }
        } 
        else {
            // --- MODO: JÁ GANHOU (Esperando reinício) ---
            // O sistema zera o contador de detecção, pois o foco agora é aguardar a retirada da bola.
            detected_counter_ms = 0; 

            if (estado == 1) { 
                // Se a bola NÃO for detectada (foi retirada), o sistema incrementa o contador de ausência.
                absent_counter_ms += 20;
            } else {
                // Se a bola ainda estiver lá (ou voltar), o sistema reinicia a contagem de reset.
                absent_counter_ms = 0;
            }

            // O sistema verifica se a bola ficou ausente por 2 segundos para confirmar o reinício.
            if (absent_counter_ms >= 2000) { 
                if (xSemaphoreTake(data_mutex, portMAX_DELAY)) {
                    // O sistema reseta o estado global para permitir um novo jogo.
                    global_data.game_won = false;
                    printf(">>> JOGO REINICIADO (BOLA RETIRADA) <<<\n");
                    xSemaphoreGive(data_mutex);
                }
                absent_counter_ms = 0;
            }
        }
        // O sistema aguarda 20ms antes da próxima leitura, definindo a taxa de amostragem da lógica.
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}