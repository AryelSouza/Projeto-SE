#include "servo_driver.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "system_common.h"
#include <math.h>
#include <stdlib.h>

// --- Configurações de Hardware e PWM ---
// O sistema define os pinos GPIO conectados aos fios de sinal dos servos.
#define SERVO_X_PIN             15
#define SERVO_Y_PIN             2

// O sistema configura o Timer 0 do LEDC em modo Low Speed.
#define LEDC_TIMER_SERVO        LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
// O sistema define a frequência de 50Hz, padrão para servomotores analógicos (período de 20ms).
#define LEDC_FREQ_SERVO         50
// O sistema utiliza resolução de 13 bits (0 a 8191), permitindo um controle fino da posição.
#define LEDC_RES_SERVO          LEDC_TIMER_13_BIT 
#define LEDC_CHANNEL_X          LEDC_CHANNEL_0  
#define LEDC_CHANNEL_Y          LEDC_CHANNEL_1  

// --- Parâmetros de Controle ---
// O sistema define o fator de suavização (0.0 a 1.0). Valores menores simulam um movimento mais "pesado" (hidráulico).
#define SERVO_SMOOTH_FACTOR     0.15f  
// O sistema define a zona morta central para evitar movimentos indesejados quando o joystick está solto.
#define JOY_DEADZONE            50     
#define ADC_MIN_READ            100
#define ADC_MAX_READ            4000

// --- Parâmetros de Varredura (Calibração) ---
// O sistema define os limites brutos de PWM para a rotina de busca de limites físicos.
#define SCAN_X_MIN 650
#define SCAN_X_MAX 950
#define SCAN_Y_MIN 675
#define SCAN_Y_MAX 825

/**
 * @brief Mapeia um valor de um intervalo de entrada para um intervalo de saída.
 * O sistema realiza uma interpolação linear e limita (clamp) o resultado aos
 * máximos e mínimos definidos, garantindo que o sinal PWM nunca exceda os limites seguros.
 */
static int map_value(int value, int in_min, int in_max, int out_min, int out_max) {
    long result = (long)(value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if (out_min < out_max) { 
        if (result < out_min) return out_min; 
        if (result > out_max) return out_max;
    } else { 
        if (result < out_max) return out_max; 
        if (result > out_min) return out_min; 
    }
    return (int)result;
}

/**
 * @brief Aplica uma zona morta ao redor do centro do joystick.
 * O sistema ignora variações de leitura dentro do limiar `JOY_DEADZONE`,
 * retornando o valor central perfeito para evitar jitter (tremores) quando o stick está em repouso.
 */
static int apply_deadzone(int raw_val) {
    int center = (ADC_MAX_READ + ADC_MIN_READ) / 2;
    if (abs(raw_val - center) < JOY_DEADZONE) return center;
    return raw_val;
}

/**
 * @brief Move o servo gradualmente para uma posição alvo (função bloqueante).
 * O sistema é utilizado apenas durante a calibração para mover a mesa lentamente,
 * evitando solavancos bruscos enquanto os sensores leem os ângulos.
 */
static void move_smooth_calib(ledc_channel_t channel, int target_duty) {
    int current_duty = ledc_get_duty(LEDC_MODE, channel);
    int step = (target_duty > current_duty) ? 1 : -1;
    while (current_duty != target_duty) {
        current_duty += step;
        ledc_set_duty(LEDC_MODE, channel, current_duty);
        ledc_update_duty(LEDC_MODE, channel);
        vTaskDelay(pdMS_TO_TICKS(4)); 
    }
    vTaskDelay(pdMS_TO_TICKS(200)); 
}

/**
 * @brief Inicializa o periférico LEDC para controle PWM.
 * O sistema configura o timer base e anexa os canais aos pinos GPIO correspondentes.
 * 
 */
void servos_init(void) {
    ledc_timer_config_t timer_servo = {
        .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER_SERVO, 
        .duty_resolution = LEDC_RES_SERVO, .freq_hz = LEDC_FREQ_SERVO, .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_servo));
    
    ledc_channel_config_t ch_x = { .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_X, .timer_sel = LEDC_TIMER_SERVO, .intr_type = LEDC_INTR_DISABLE, .gpio_num = SERVO_X_PIN, .duty = 0, .hpoint = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_x));
    
    ledc_channel_config_t ch_y = { .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_Y, .timer_sel = LEDC_TIMER_SERVO, .intr_type = LEDC_INTR_DISABLE, .gpio_num = SERVO_Y_PIN, .duty = 0, .hpoint = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_y));
}

/**
 * @brief Tarefa principal de controle de posição dos servos.
 * O sistema lê os dados do joystick, processa a suavização e atualiza o hardware.
 */
void servo_control_task(void *pvParameters) {
    system_data_t local;
    
    // Variáveis estáticas para manter o estado da posição anterior (necessário para o filtro).
    static float current_x_f = 0.0f;
    static float current_y_f = 0.0f;
    bool initialized = false;

    for (;;) {
        bool in_calibration = false;
        
        // O sistema adquire os dados mais recentes do joystick e verifica o modo de operação.
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            local = global_data;
            in_calibration = global_data.calibration_mode;
            xSemaphoreGive(data_mutex);
        }

        // O controle normal só ocorre se o sistema não estiver em modo de calibração.
        if (!in_calibration) {
            // Na primeira execução, o sistema centraliza os servos suavemente baseando-se nos limites atuais.
            if (!initialized) {
                current_x_f = (float)(local.servo_min_x + local.servo_max_x) / 2.0f;
                current_y_f = (float)(local.servo_min_y + local.servo_max_y) / 2.0f;
                initialized = true;
            }

            // 1. Aplicação de Zona Morta
            int joy_x_clean = apply_deadzone(local.joy_x_raw);
            int joy_y_clean = apply_deadzone(local.joy_y_raw);

            // 2. Mapeamento
            // O sistema converte a leitura do ADC (0-4095) para o range PWM calibrado dos servos.
            int target_x = map_value(joy_x_clean, ADC_MIN_READ, ADC_MAX_READ, local.servo_max_x, local.servo_min_x);
            int target_y = map_value(joy_y_clean, ADC_MIN_READ, ADC_MAX_READ, local.servo_max_y, local.servo_min_y);
            
            // 3. Filtro Exponencial (Movimento Hidráulico)
            // O sistema aproxima a posição atual do alvo em passos proporcionais à diferença, criando aceleração/desaceleração suave.
            current_x_f += ((float)target_x - current_x_f) * SERVO_SMOOTH_FACTOR;
            current_y_f += ((float)target_y - current_y_f) * SERVO_SMOOTH_FACTOR;

            // 4. Atualização de Hardware
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, (int)current_x_f);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, (int)current_y_f);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y);
        }
        // O sistema atualiza a posição a cada 20ms (50Hz), coincidindo com a frequência do servo.
        vTaskDelay(pdMS_TO_TICKS(20)); 
    }
}

/**
 * @brief Tarefa de Autocalibração.
 * O sistema varre os servos e monitora o acelerômetro para detectar limites mecânicos.
 */
void auto_calibration_task(void *pvParameters) {
    const int step_delay_ms = 15;
    const int pwm_step = 1;
    // O limiar de variação de ângulo que indica que a mesa ainda está se movendo.
    const float noise_threshold = 0.15f; 

    for (;;) {
        bool start = false;
        
        // O sistema verifica se houve um gatilho (botão pressionado) para iniciar a calibração.
        if (xSemaphoreTake(data_mutex, portMAX_DELAY)) {
            if (global_data.calibration_trigger) {
                global_data.calibration_trigger = false;
                global_data.calibration_mode = true;
                start = true;
            }
            xSemaphoreGive(data_mutex);
        }

        if (start) {
            printf(">>> INICIANDO CALIBRACAO <<<\n");
            int recorded_min_x = 0, recorded_max_x = 0;
            int recorded_min_y = 0, recorded_max_y = 0;
            float last_angle = 0, current_angle = 0;

            // --- Varredura Eixo X ---
            printf("Scan X...\n");
            // O sistema move para a posição mínima de varredura inicial.
            move_smooth_calib(LEDC_CHANNEL_X, SCAN_X_MIN);
            if(xSemaphoreTake(data_mutex, portMAX_DELAY)) { last_angle = global_data.filtered_roll; xSemaphoreGive(data_mutex); }
            vTaskDelay(pdMS_TO_TICKS(200));

            // O sistema incrementa o PWM passo a passo.
            for (int pwm = SCAN_X_MIN; pwm <= SCAN_X_MAX; pwm += pwm_step) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, pwm);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X);
                vTaskDelay(pdMS_TO_TICKS(step_delay_ms));

                // O sistema lê o ângulo atual do MPU6050.
                if(xSemaphoreTake(data_mutex, portMAX_DELAY)) { current_angle = global_data.filtered_roll; xSemaphoreGive(data_mutex); }
                
                // Se o ângulo mudou significativamente, significa que o servo ainda está movendo a mesa (não bateu no fim de curso).
                if (fabs(current_angle - last_angle) > noise_threshold) {
                    if (recorded_min_x == 0) recorded_min_x = pwm; // Registra o primeiro ponto de movimento
                    recorded_max_x = pwm; // Atualiza o último ponto de movimento conhecido
                }
                last_angle = current_angle;
            }

            // O sistema centraliza o eixo X antes de calibrar o Y.
            int center_x = (recorded_min_x + recorded_max_x) / 2;
            if (center_x == 0) center_x = (SCAN_X_MIN + SCAN_X_MAX) / 2;
            move_smooth_calib(LEDC_CHANNEL_X, center_x);

            // --- Varredura Eixo Y ---
            printf("Scan Y...\n");
            move_smooth_calib(LEDC_CHANNEL_Y, SCAN_Y_MIN);
            if(xSemaphoreTake(data_mutex, portMAX_DELAY)) { last_angle = global_data.filtered_pitch; xSemaphoreGive(data_mutex); }
            vTaskDelay(pdMS_TO_TICKS(200));

            for (int pwm = SCAN_Y_MIN; pwm <= SCAN_Y_MAX; pwm += pwm_step) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, pwm);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y);
                vTaskDelay(pdMS_TO_TICKS(step_delay_ms));

                if(xSemaphoreTake(data_mutex, portMAX_DELAY)) { current_angle = global_data.filtered_pitch; xSemaphoreGive(data_mutex); }
                
                if (fabs(current_angle - last_angle) > noise_threshold) {
                    if (recorded_min_y == 0) recorded_min_y = pwm;
                    recorded_max_y = pwm;
                }
                last_angle = current_angle;
            }
            int center_y = (recorded_min_y + recorded_max_y) / 2;
            move_smooth_calib(LEDC_CHANNEL_Y, center_y);

            // --- Salvamento e Margem de Segurança ---
            // O sistema aplica uma margem de segurança de 5 unidades de PWM para evitar forçar os servos nos extremos.
            int safe_min_x = recorded_min_x + 5; int safe_max_x = recorded_max_x - 5;
            int safe_min_y = recorded_min_y + 5; int safe_max_y = recorded_max_y - 5;

            printf("FIM: X[%d-%d] Y[%d-%d]\n", safe_min_x, safe_max_x, safe_min_y, safe_max_y);
            
            // O sistema atualiza os limites globais e libera o controle normal dos servos.
            if (xSemaphoreTake(data_mutex, portMAX_DELAY)) {
                // Validação simples para evitar salvar calibrações inválidas (muito curtas).
                if (recorded_max_x > recorded_min_x + 20) {
                    global_data.servo_min_x = safe_min_x; global_data.servo_max_x = safe_max_x;
                }
                if (recorded_max_y > recorded_min_y + 20) {
                    global_data.servo_min_y = safe_min_y; global_data.servo_max_y = safe_max_y;
                }
                global_data.calibration_mode = false;
                xSemaphoreGive(data_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}