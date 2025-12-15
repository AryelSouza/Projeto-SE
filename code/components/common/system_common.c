#include "system_common.h"

// Alocação real das variáveis
// Esta estrutura centraliza o estado do sistema e permite a comunicação entre as diferentes tarefas (Tasks).
system_data_t global_data = {
    // --- Entradas do Controle ---
    0,      // joy_x_raw: O sistema armazena o valor bruto da leitura analógica (ADC) do eixo horizontal do joystick.
    0,      // joy_y_raw: O sistema armazena o valor bruto da leitura analógica (ADC) do eixo vertical do joystick.
    false,  // btn_pressed: Indica se o botão integrado ao joystick encontra-se pressionado no momento.

    // --- Dados do Sensor Inercial (MPU6050) ---
    0.0f,   // filtered_pitch: O sistema mantém o ângulo de inclinação lateral (Pitch) após passar pelo filtro complementar.
    0.0f,   // filtered_roll: O sistema mantém o ângulo de rolagem frontal (Roll) após passar pelo filtro complementar.

    // --- Controle de Calibração ---
    false,  // calibration_mode: Sinaliza se o sistema está atualmente executando a rotina de autocalibração dos servos.
    false,  // calibration_trigger: Atua como um gatilho para iniciar a calibração na próxima iteração da tarefa de servos.

    // --- Limites de Segurança dos Servos (PWM) ---
    700,    // servo_min_x: Define o valor mínimo de Duty Cycle permitido para o servo do eixo X, evitando travamento mecânico.
    900,    // servo_max_x: Define o valor máximo de Duty Cycle permitido para o servo do eixo X.
    675,    // servo_min_y: Define o valor mínimo de Duty Cycle permitido para o servo do eixo Y.
    825,    // servo_max_y: Define o valor máximo de Duty Cycle permitido para o servo do eixo Y.

    // --- Estado do Jogo ---
    false   // game_won: O sistema define como verdadeiro quando o sensor indutivo detecta a presença da bola (vitória).
};

// Handle do Mutex
// Esta variável gerencia o bloqueio de exclusão mútua, garantindo que apenas uma tarefa acesse os dados globais por vez.
SemaphoreHandle_t data_mutex = NULL;