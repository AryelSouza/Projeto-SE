#include "mpu_driver.h"
#include "driver/i2c.h" // Driver legado para compatibilidade
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "system_common.h"
#include <math.h>

// --- PINOS E ENDEREÇOS ---
// O sistema define os pinos GPIO para o barramento de comunicação I2C.
#define I2C_MASTER_SCL_IO       19
#define I2C_MASTER_SDA_IO       18
// O sistema seleciona a porta I2C 0 do ESP32 e define a frequência de clock para 100kHz (Standard Mode).
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      100000
// O sistema define o endereço padrão do MPU6050 e os registradores de controle.
#define MPU6050_ADDR            0x68
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_ACCEL_XOUT_H    0x3B
// O sistema define o fator de peso para o filtro complementar (98% Giroscópio, 2% Acelerômetro).
#define COMPLEMENTARY_ALPHA     0.98f

/**
 * @brief Configura o hardware I2C do ESP32.
 * O driver inicializa o periférico no modo Master, configura os pinos SDA/SCL
 * e ativa os resistores de pull-up internos necessários para o protocolo.
 */
static void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

/**
 * @brief Inicializa o sensor MPU6050.
 * O sistema envia um comando para o registrador de gerenciamento de energia
 * para despertar o sensor, que inicia em modo de suspensão (sleep) por padrão.
 */
static void mpu6050_init_hw() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

/**
 * @brief Normaliza ângulos para o intervalo [-180, +180].
 * O sistema garante que a aritmética angular permaneça consistente, evitando
 * saltos abruptos (ex: passar de 359 para 0).
 */
static float normalize_angle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/**
 * @brief Tarefa principal de leitura e processamento inercial.
 * O sistema lê os dados brutos, funde os sensores (Acelerômetro + Giroscópio)
 * e disponibiliza os ângulos filtrados para o restante da aplicação.
 */
void mpu_reading_task(void *pvParameters) {
    i2c_master_init();
    mpu6050_init_hw();
    
    uint8_t raw_data[14];
    float dt = 0.015f; // Delta Time (intervalo de tempo estimado do loop)
    TickType_t last_wake = xTaskGetTickCount();

    // OFFSETS DE CALIBRAÇÃO (Ajuste fino da mesa)
    // O sistema define valores fixos para zerar a inclinação mecânica natural da montagem.
    const float FIXED_OFFSET_ROLL  = -174.62f; 
    const float FIXED_OFFSET_PITCH = -8.38f; 

    // Offsets do giroscópio (drift)
    // O sistema compensa o erro estático natural do giroscópio.
    float gyro_offset_x = -176.0f; 
    float gyro_offset_y = -10.0f;    
    
    // Variáveis do filtro complementar
    static float filt_pitch = FIXED_OFFSET_PITCH;
    static float filt_roll  = FIXED_OFFSET_ROLL;
    
    // Variáveis para suavização visual (display/gráfico)
    float smooth_pitch_display = 0.0f;
    float smooth_roll_display = 0.0f;
    const float SMOOTH_FACTOR = 0.4f; 

    printf(">>> MPU INICIADO COM TARA: X=%.2f Y=%.2f <<<\n", FIXED_OFFSET_ROLL, FIXED_OFFSET_PITCH);

    for (;;) {
        // --- Leitura I2C em Burst ---
        // O sistema lê 14 bytes sequenciais começando do acelerômetro X.
        // Isso garante que todos os dados (Accel, Temp, Gyro) sejam do mesmo instante de tempo.
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, true); 
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
        
        i2c_master_read(cmd, raw_data, 13, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, &raw_data[13], I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 20 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            // --- Conversão de Dados ---
            // O sistema combina os bytes High e Low (Big Endian) para formar inteiros de 16 bits.
            int16_t accel_x = (raw_data[0] << 8) | raw_data[1];
            int16_t accel_y = (raw_data[2] << 8) | raw_data[3];
            int16_t accel_z = (raw_data[4] << 8) | raw_data[5];
            // int16_t temp = (raw_data[6] << 8) | raw_data[7]; // Temperatura ignorada
            int16_t gyro_x  = (raw_data[8]  << 8) | raw_data[9];
            int16_t gyro_y  = (raw_data[10] << 8) | raw_data[11];

            // O sistema converte os dados brutos do giroscópio para graus por segundo (LSB Sensitivity).
            float gx = (gyro_x - gyro_offset_x) / 131.0f;
            float gy = (gyro_y - gyro_offset_y) / 131.0f;
            
            // --- Cálculo Trigonométrico (Acelerômetro) ---
            // O sistema calcula a inclinação estática baseada na gravidade (vetor Z e componentes X/Y).
            float accel_roll  = atan2(accel_y, accel_z) * RAD_TO_DEG;
            float accel_pitch = atan2(-accel_x, sqrt(accel_y*accel_y + accel_z*accel_z)) * RAD_TO_DEG;

            accel_roll = normalize_angle(accel_roll);
            accel_pitch = normalize_angle(accel_pitch);

            // --- Filtro Complementar ---
            // O sistema integra a velocidade angular do giroscópio (gx * dt) para resposta rápida.
            // O sistema funde com o ângulo do acelerômetro para corrigir o drift (deriva) do giroscópio a longo prazo.
            float gyro_pitch = filt_pitch + gy * dt;  
            float gyro_roll  = filt_roll  + gx * dt;
            
            filt_pitch = COMPLEMENTARY_ALPHA * gyro_pitch + (1.0f - COMPLEMENTARY_ALPHA) * accel_pitch;
            filt_roll  = COMPLEMENTARY_ALPHA * gyro_roll  + (1.0f - COMPLEMENTARY_ALPHA) * accel_roll;

            // --- Aplicação da Tara ---
            // O sistema subtrai o offset fixo para que a posição nivelada da mesa corresponda a 0 graus.
            float final_pitch = normalize_angle(filt_pitch - FIXED_OFFSET_PITCH);
            float final_roll  = normalize_angle(filt_roll - FIXED_OFFSET_ROLL);

            // --- Suavização Visual ---
            // O sistema aplica um filtro exponencial simples (Low Pass) para remover tremores finos antes da exibição.
            smooth_pitch_display = (smooth_pitch_display * (1.0f - SMOOTH_FACTOR)) + (final_pitch * SMOOTH_FACTOR);
            smooth_roll_display  = (smooth_roll_display  * (1.0f - SMOOTH_FACTOR)) + (final_roll  * SMOOTH_FACTOR);

            // --- Atualização Global ---
            // O sistema adquire o Mutex para atualizar os dados compartilhados com segurança.
            if (xSemaphoreTake(data_mutex, portMAX_DELAY)) {
                global_data.filtered_pitch = smooth_pitch_display;
                global_data.filtered_roll  = smooth_roll_display;
                xSemaphoreGive(data_mutex);
            }
        }
        
        // O sistema garante uma taxa de atualização constante de aproximadamente 66Hz (15ms).
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(15));
    }
}