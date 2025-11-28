#include <stdio.h>
#include <stdbool.h>

// Headers do ESP-IDF / FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/ledc.h" // Driver para PWM (Servos)
//#include "esp_adc_cal.h" 

// --- Configuração de Pinos e LEDC ---

// Mapeamento do Joystick (Assumindo que os pinos que funcionaram foram mantidos)
#define JOYSTICK_VERT_CHANNEL   ADC1_CHANNEL_4  // GPIO 32 (VRy)
#define JOYSTICK_HOR_CHANNEL    ADC1_CHANNEL_5  // GPIO 33 (VRx)
#define JOYSTICK_SW_PIN         GPIO_NUM_25     // Botão

#define SERVO_X_PIN             GPIO_NUM_18
#define SERVO_Y_PIN             GPIO_NUM_19

// Configuração LEDC (PWM) para Servo Motor
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_FREQUENCY          50              // 50 Hz
#define LEDC_RESOLUTION         LEDC_TIMER_13_BIT // 13 bits (0 a 8191)

// Canais LEDC
#define LEDC_CHANNEL_X          LEDC_CHANNEL_0  
#define LEDC_CHANNEL_Y          LEDC_CHANNEL_1  

// Mapeamento de Duty Cycle para 90° de Movimento (45° a 135°)
#define SERVO_DUTY_MIN_45       370             // Duty para 45°
#define SERVO_DUTY_MAX_45       690             // Duty para 135°

// Calibração de Leitura 
#define ADC_MIN_READ            100             // Mínimo real lido pelo Joystick
#define ADC_MAX_READ            4000            // Máximo real lido pelo Joystick
#define ADC_MAX_VALUE           4095            // Máximo teórico lido do ADC


// --- Estrutura de Dados Compartilhada ---
typedef struct {
    int x_value;
    int y_value;
    bool button_pressed;
} joystick_data_t;

joystick_data_t current_joystick_data = {0, 0, false};
SemaphoreHandle_t joystick_mutex;


/**
 * @brief Implementa o mapeamento de valores de uma faixa para outra (função map).
 */
int map_value(int value, int in_min, int in_max, int out_min, int out_max) {
    if (in_min == in_max) return out_min;

    // Fórmula de mapeamento
    long result = (long)(value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    
    // Limita o resultado (considerando mapeamento invertido ou normal)
    if (out_min < out_max) { 
        if (result < out_min) return out_min;
        if (result > out_max) return out_max;
    } else { // Mapeamento invertido
        if (result < out_max) return out_max; 
        if (result > out_min) return out_min; 
    }
    return (int)result;
}


// --- Funções de Inicialização ---

void servo_ledc_init() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_RESOLUTION,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_x = {
        .speed_mode     = LEDC_MODE, .channel = LEDC_CHANNEL_X, .timer_sel = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE, .gpio_num = SERVO_X_PIN, .duty = 0, .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_x));

    ledc_channel_config_t ledc_channel_y = {
        .speed_mode     = LEDC_MODE, .channel = LEDC_CHANNEL_Y, .timer_sel = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE, .gpio_num = SERVO_Y_PIN, .duty = 0, .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_y));
    
    printf("Driver LEDC (PWM) para Servos inicializado com sucesso.\n");
}


// --- Task 1: Produtora (Leitura do Joystick) ---
void joystick_read_task(void *pvParameters) {
    // 1. Inicializa o ADC1
    adc1_config_width(ADC_WIDTH_BIT_12); 
    adc1_config_channel_atten(JOYSTICK_VERT_CHANNEL, ADC_ATTEN_DB_11); 
    adc1_config_channel_atten(JOYSTICK_HOR_CHANNEL, ADC_ATTEN_DB_11); 
    
    // 2. Configura GPIO do Botão
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << JOYSTICK_SW_PIN), .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn_config);

    printf("Task 1: Leitura do Joystick [P:2] iniciada.\n");

    for (;;) {
        int raw_y = adc1_get_raw(JOYSTICK_VERT_CHANNEL); 
        int raw_x = adc1_get_raw(JOYSTICK_HOR_CHANNEL);  
        bool button_state = (gpio_get_level(JOYSTICK_SW_PIN) == 0); 

        // Protege a escrita
        if (xSemaphoreTake(joystick_mutex, portMAX_DELAY) == pdTRUE) {
            current_joystick_data.x_value = raw_x;
            current_joystick_data.y_value = raw_y;
            current_joystick_data.button_pressed = button_state;
            xSemaphoreGive(joystick_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz de leitura
    }
}


// --- Task 2: Consumidora (Controle dos Servos) ---
void actuator_control_task(void *pvParameters) {
    joystick_data_t local_data;

    printf("Task 2: Controle dos Servos [P:1] iniciada.\n");

    for (;;) {
        // 1. LÊ os dados brutos (Protegido)
        if (xSemaphoreTake(joystick_mutex, portMAX_DELAY) == pdTRUE) {
            local_data = current_joystick_data;
            xSemaphoreGive(joystick_mutex);
        }

        // 2. MAPEIA e INVERTE para a Faixa de 90° (45° a 135°)
        
        // Eixo X (Horizontal): (Invertido: Alto ADC -> Baixo Duty)
        int duty_x = map_value(local_data.x_value, 
                               ADC_MIN_READ, ADC_MAX_READ, 
                               SERVO_DUTY_MAX_45, SERVO_DUTY_MIN_45); // Mapeia para [690, 370]
        
        // Eixo Y (Vertical): (Invertido: Alto ADC -> Baixo Duty)
        int duty_y = map_value(local_data.y_value, 
                               ADC_MIN_READ, ADC_MAX_READ, 
                               SERVO_DUTY_MAX_45, SERVO_DUTY_MIN_45); // Mapeia para [690, 370]

        // 3. ATUALIZA os Servos
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, duty_x));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X));

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, duty_y));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y));
        
        vTaskDelay(pdMS_TO_TICKS(10)); // 100 Hz de atuação para movimento suave
    }
}


// --- Task 3: Monitoramento de Status (Logs) ---
void status_monitor_task(void *pvParameters) {
    joystick_data_t status_data;
    
    printf("Task 3: Monitoramento de Status [P:1] iniciada.\n");

    for (;;) {
        // LÊ os dados para monitoramento (Protegido)
        if (xSemaphoreTake(joystick_mutex, portMAX_DELAY) == pdTRUE) {
            status_data = current_joystick_data;
            xSemaphoreGive(joystick_mutex);
        }
        
        // Mapeia os dados lidos para o duty cycle para fins de log
        int duty_x = map_value(status_data.x_value, ADC_MIN_READ, ADC_MAX_READ, SERVO_DUTY_MAX_45, SERVO_DUTY_MIN_45);
        int duty_y = map_value(status_data.y_value, ADC_MIN_READ, ADC_MAX_READ, SERVO_DUTY_MAX_45, SERVO_DUTY_MIN_45);

        // Imprime o status atual
        printf("STATUS | X_Raw: %d (Duty: %d) | Y_Raw: %d (Duty: %d) | Button: %s\n", 
               status_data.x_value, duty_x, 
               status_data.y_value, duty_y, 
               status_data.button_pressed ? "PRESSIONADO" : "SOLTO");
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Log a cada 500ms
    }
}


// --- Função Principal do ESP-IDF ---
void app_main(void) {
    // 1. CRIA O MUTEX
    joystick_mutex = xSemaphoreCreateMutex();
    if (joystick_mutex == NULL) {
        printf("ERRO FATAL: Falha ao criar o Mutex do Joystick!\n");
        return;
    }

    // 2. INICIALIZA O PWM PARA OS SERVOS
    servo_ledc_init();

    // 3. CRIA AS TASKS
    
    // T1: Leitura (Maior Prioridade: 2)
    xTaskCreate(joystick_read_task, "JoystickRead", 2048, NULL, 2, NULL);

    // T2: Controle (Prioridade Média: 1)
    xTaskCreate(actuator_control_task, "ActuatorControl", 2048, NULL, 1, NULL);

    // T3: Monitoramento (Prioridade Média/Baixa: 1)
    xTaskCreate(status_monitor_task, "StatusMonitor", 2048, NULL, 1, NULL);

    printf("Fase 1 COMPLETA: 3 Tasks FreeRTOS iniciadas.\n");
}
