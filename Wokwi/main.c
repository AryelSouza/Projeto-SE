#include <stdio.h>
#include <stdbool.h>
#include <math.h>             // Necessário para cálculos de ângulo (atan2, sqrt)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"       // Driver I2C para o MPU6050

// --- CONFIGURAÇÃO DE PINOS ---

// Joystick (Mantido da Fase 1)
#define JOYSTICK_VERT_CHANNEL   ADC1_CHANNEL_4  // GPIO 32 (VRy)
#define JOYSTICK_HOR_CHANNEL    ADC1_CHANNEL_5  // GPIO 33 (VRx)
#define JOYSTICK_SW_PIN         GPIO_NUM_25     // Botão

// Servos (Novos pinos solicitados)
#define SERVO_X_PIN             GPIO_NUM_15
#define SERVO_Y_PIN             GPIO_NUM_2

// MPU6050 (I2C) - Fase 2
#define I2C_MASTER_SCL_IO       GPIO_NUM_19     // SCL
#define I2C_MASTER_SDA_IO       GPIO_NUM_18     // SDA
#define I2C_MASTER_NUM          I2C_NUM_0       // Porta I2C 0
#define I2C_MASTER_FREQ_HZ      100000          // Frequência 100kHz
#define MPU6050_ADDR            0x68            // Endereço padrão do MPU6050

// Registradores do MPU6050
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_GYRO_XOUT_H     0x43
#define COMPLEMENTARY_ALPHA 0.98f

// --- CONFIGURAÇÃO SERVO (PWM) ---
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_FREQUENCY          50
#define LEDC_RESOLUTION         LEDC_TIMER_13_BIT 

#define LEDC_CHANNEL_X          LEDC_CHANNEL_0  
#define LEDC_CHANNEL_Y          LEDC_CHANNEL_1  

// Valores de calibração (Ajuste conforme seus servos de rotação contínua ou padrão)
#define SERVO_DUTY_MIN          370             
#define SERVO_DUTY_MAX          690             

// Joystick Calibração
#define ADC_MIN_READ            100
#define ADC_MAX_READ            4000

// Constante Math
#define RAD_TO_DEG              57.2957795131

// --- ESTRUTURA DE DADOS COMPARTILHADA ---
typedef struct {
    int joy_x_raw;
    int joy_y_raw;
    bool btn_pressed;

    float angle_pitch;
    float angle_roll;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float filtered_pitch;
    float filtered_roll;
} system_data_t;

system_data_t global_data = {0, 0, false, 0.0f, 0.0f};
SemaphoreHandle_t data_mutex;

// --- FUNÇÕES AUXILIARES ---

int map_value(int value, int in_min, int in_max, int out_min, int out_max) {
    if (in_min == in_max) return out_min;
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

// Inicializa I2C
void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
    printf("I2C Inicializado: SDA=%d, SCL=%d\n", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
}

// Inicializa MPU6050 (Acorda o sensor)
void mpu6050_init() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0x00, true);  // Acorda o MPU
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    printf(ret == ESP_OK ? "MPU6050 ligado!\n" : "Falha ao inicializar MPU6050\n");
}

// Inicializa PWM Servos
void servo_ledc_init() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER, .duty_resolution = LEDC_RESOLUTION,
        .freq_hz = LEDC_FREQUENCY, .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_ch_x = { .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_X, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = SERVO_X_PIN, .duty = 0, .hpoint = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch_x));

    ledc_channel_config_t ledc_ch_y = { .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_Y, .timer_sel = LEDC_TIMER, .intr_type = LEDC_INTR_DISABLE, .gpio_num = SERVO_Y_PIN, .duty = 0, .hpoint = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch_y));
}

// --- TASKS ---

// Task 1: Leitura do Joystick (Fase 1)
void joystick_read_task(void *pvParameters) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(JOYSTICK_VERT_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(JOYSTICK_HOR_CHANNEL, ADC_ATTEN_DB_11);
    
    gpio_set_direction(JOYSTICK_SW_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(JOYSTICK_SW_PIN, GPIO_PULLUP_ONLY);

    for (;;) {
        int raw_y = adc1_get_raw(JOYSTICK_VERT_CHANNEL);
        int raw_x = adc1_get_raw(JOYSTICK_HOR_CHANNEL);
        bool btn = (gpio_get_level(JOYSTICK_SW_PIN) == 0);

        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            global_data.joy_x_raw = raw_x;
            global_data.joy_y_raw = raw_y;
            global_data.btn_pressed = btn;
            xSemaphoreGive(data_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz
    }
}

// Task 2: Leitura do MPU6050 (Fase 2 - NOVA)
void mpu_reading_task(void *pvParameters) {
    uint8_t raw_data[14];
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;

    float dt = 0.05f; // 50 ms (20Hz)
    TickType_t last_wake = xTaskGetTickCount();

    mpu6050_init();

    for (;;) {
        // --- Leitura Burst 14 bytes ---
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, true);

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
        i2c_master_read(cmd, raw_data, 13, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &raw_data[13], I2C_MASTER_NACK);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {

            // ----- ACELERAÇÃO -----
            accel_x = (raw_data[0] << 8) | raw_data[1];
            accel_y = (raw_data[2] << 8) | raw_data[3];
            accel_z = (raw_data[4] << 8) | raw_data[5];

            // ----- GIROSCÓPIO -----
            gyro_x = (raw_data[8]  << 8) | raw_data[9];
            gyro_y = (raw_data[10] << 8) | raw_data[11];
            gyro_z = (raw_data[12] << 8) | raw_data[13];

            // Conversão do giroscópio para °/s
            float gx = gyro_x / 131.0f;
            float gy = gyro_y / 131.0f;
            float gz = gyro_z / 131.0f;

            // Calculo Pitch/Roll do acelerômetro
            float accel_roll  = atan2(accel_y, accel_z) * RAD_TO_DEG;
            float accel_pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * RAD_TO_DEG;

            // ======== FILTRO COMPLEMENTAR ========
            static float filt_pitch = 0;
            static float filt_roll  = 0;

            // Integração do giroscópio
            float gyro_pitch = filt_pitch + gy * dt;  
            float gyro_roll  = filt_roll  + gx * dt;

            // Combinação com acelerômetro
            filt_pitch = COMPLEMENTARY_ALPHA * gyro_pitch + (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;
            filt_roll  = COMPLEMENTARY_ALPHA * gyro_roll  + (1.0f - COMPLEMENTARY_ALPHA) * accel_roll;

            // Atualização global
            if (xSemaphoreTake(data_mutex, portMAX_DELAY)) {
                global_data.angle_pitch = accel_pitch;
                global_data.angle_roll  = accel_roll;

                global_data.gyro_x = gx;
                global_data.gyro_y = gy;
                global_data.gyro_z = gz;

                global_data.filtered_pitch = filt_pitch;
                global_data.filtered_roll  = filt_roll;

                xSemaphoreGive(data_mutex);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50)); // 20Hz
    }
}

// Task 3: Controle Servos (Fase 1)
void servo_control_task(void *pvParameters) {
    system_data_t local;
    for (;;) {
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            local = global_data;
            xSemaphoreGive(data_mutex);
        }

        int duty_x = map_value(local.joy_x_raw, ADC_MIN_READ, ADC_MAX_READ, SERVO_DUTY_MAX, SERVO_DUTY_MIN);
        int duty_y = map_value(local.joy_y_raw, ADC_MIN_READ, ADC_MAX_READ, SERVO_DUTY_MAX, SERVO_DUTY_MIN);

        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, duty_x);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X);
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, duty_y);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y);
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// Task 4: Monitoramento e JSON (Fase 2 - Atualizada)
void status_monitor_task(void *pvParameters) {
    system_data_t local;
    printf("Monitor Serial Iniciado. Saida em JSON.\n");

    for (;;) {
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            local = global_data;
            xSemaphoreGive(data_mutex);
        }

        // Formatação JSON Obrigatória da Fase 2
        // Exemplo: {"joystick": {"x": 2048, "y": 2048, "btn": 0}, "mpu": {"pitch": 10.5, "roll": -5.2}}
        printf(
            "{\"joystick\": {\"x\": %d, \"y\": %d, \"btn\": %d}, "
            "\"mpu\": {\"pitch\": %.2f, \"roll\": %.2f, "
            "\"gyro\": {\"x\": %.2f, \"y\": %.2f, \"z\": %.2f}, "
            "\"filt\": {\"pitch\": %.2f, \"roll\": %.2f}}}\n",
            local.joy_x_raw,
            local.joy_y_raw,
            local.btn_pressed ? 1 : 0,
            local.angle_pitch,
            local.angle_roll,
            local.gyro_x,
            local.gyro_y,
            local.gyro_z,
            local.filtered_pitch,
            local.filtered_roll
        );
        
        vTaskDelay(pdMS_TO_TICKS(200)); // Envio a 5Hz (a cada 200ms)
    }
}

void app_main(void) {
    data_mutex = xSemaphoreCreateMutex();

    // Inicializações de Hardware
    i2c_master_init();   // I2C
    servo_ledc_init();   // PWM

    // Criação das Tarefas
    xTaskCreate(joystick_read_task, "JoyRead", 2048, NULL, 2, NULL);   // Leitura Controle
    xTaskCreate(mpu_reading_task,   "MPURead", 2048, NULL, 3, NULL);   // Leitura Sensor (Alta prioridade)
    xTaskCreate(servo_control_task, "Servos",  2048, NULL, 1, NULL);   // Atuação Mecânica
    xTaskCreate(status_monitor_task,"Monitor", 2048, NULL, 1, NULL);   // Telemetria/JSON
}
