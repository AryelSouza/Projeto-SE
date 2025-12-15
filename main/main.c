#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Inclusão dos componentes
// O sistema importa os cabeçalhos dos drivers modulares para ter acesso às funções de inicialização e tarefas.
#include "system_common.h"
#include "joystick_driver.h"
#include "servo_driver.h"
#include "mpu_driver.h"
#include "game_logic.h"

/**
 * @brief Tarefa de Monitoramento e Telemetria.
 * O sistema utiliza esta tarefa para coletar dados de todos os módulos e enviá-los via Serial (UART).
 * Isso permite que ferramentas externas (como Grafana ou Serial Plotter) visualizem o estado do jogo em tempo real.
 */
void status_monitor_task(void *pvParameters) {
    system_data_t local;
    printf("Monitor Iniciado.\n");
    for (;;) {
        // O sistema adquire o Mutex para garantir que não lerá dados corrompidos enquanto outra tarefa está escrevendo.
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            local = global_data; // O sistema cria uma cópia local segura dos dados globais.
            xSemaphoreGive(data_mutex);
        }

        // O sistema formata os dados como uma string JSON.
        // Isso padroniza a saída para: Joystick (Input), MPU (Sensor) e Vitória (Estado do Jogo).
        printf(
            "{\"joy\":{\"x\":%d,\"y\":%d},\"mpu\":{\"ang_x\":%.2f,\"ang_y\":%.2f},\"win\":%d}\n",
            local.joy_x_raw, local.joy_y_raw,
            local.filtered_roll, local.filtered_pitch, // Roll e Pitch corrigidos para a ordem certa
            local.game_won
        );
        
        // O sistema aguarda 200ms, definindo uma taxa de atualização de telemetria de 5Hz.
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/**
 * @brief Ponto de entrada principal da aplicação.
 * O sistema é responsável por alocar recursos globais, inicializar drivers de hardware
 * e despachar as tarefas do sistema operacional em tempo real (RTOS).
 */
void app_main(void) {
    // 1. Inicializa Mutex Global
    // O sistema cria o semáforo binário (Mutex) que será usado para proteger a estrutura de dados compartilhada 'global_data'.
    data_mutex = xSemaphoreCreateMutex();
    
    // 2. Inicializa Hardware dos componentes
    // O sistema configura os ADCs, PWMs e GPIOs antes de iniciar qualquer tarefa lógica.
    joystick_init();
    servos_init();
    game_logic_init();
    // Nota: O MPU6050 é inicializado dentro de sua própria task pois depende do barramento I2C, 
    // que deve ser configurado no contexto da tarefa para garantir timing correto.

    // 3. Cria as Tasks do FreeRTOS
    // O sistema define as prioridades (quanto maior o número, maior a prioridade) e o tamanho da pilha (Stack) de cada tarefa.
    
    // Leitura do Joystick (Prioridade 3): Alta prioridade para garantir resposta rápida aos comandos do usuário.
    xTaskCreate(joystick_read_task, "JoyRead", 2048, NULL, 3, NULL);
    
    // Leitura do MPU (Prioridade 4): Prioridade crítica para manter a integração do giroscópio estável no tempo.
    xTaskCreate(mpu_reading_task,   "MPURead", 2048, NULL, 4, NULL);
    
    // Controle dos Servos (Prioridade 2): Processa o movimento físico da mesa.
    xTaskCreate(servo_control_task, "Servos",  2048, NULL, 2, NULL); 
    
    // Auto Calibração (Prioridade 2): Tarefa esporádica que assume o controle dos servos quando solicitada.
    xTaskCreate(auto_calibration_task, "Calib", 4096, NULL, 2, NULL); 
    
    // Lógica do Jogo (Prioridade 3): Verifica vitória e reinício com alta responsividade.
    xTaskCreate(win_check_task,     "WinCheck", 2048, NULL, 3, NULL); 
    
    // Monitor Serial (Prioridade 1): Baixa prioridade, pois é apenas para visualização e não afeta o controle.
    xTaskCreate(status_monitor_task,"Monitor", 2048, NULL, 1, NULL); 

    printf(">>> Sistema Modular Iniciado <<<\n");
}