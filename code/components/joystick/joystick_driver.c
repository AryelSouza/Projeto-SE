#include "joystick_driver.h"
#include "esp_adc/adc_oneshot.h" 
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "system_common.h" 

// --- Configurações de Hardware ---
// O sistema define o canal ADC correspondente ao eixo vertical (geralmente GPIO 32).
#define JOYSTICK_VERT_CHAN    ADC_CHANNEL_4 
// O sistema define o canal ADC correspondente ao eixo horizontal (geralmente GPIO 33).
#define JOYSTICK_HOR_CHAN     ADC_CHANNEL_5 
// O sistema define o pino GPIO conectado ao botão digital do joystick.
#define JOYSTICK_SW_PIN       GPIO_NUM_25
// O sistema define o tamanho da janela do filtro de média móvel para estabilização do sinal.
#define JOY_AVG_SAMPLES       16    

// O sistema mantém um handle global para gerenciar a instância do driver ADC OneShot.
adc_oneshot_unit_handle_t adc1_handle;

/**
 * @brief Inicializa o periférico ADC e o GPIO do botão.
 */
void joystick_init(void) {
    // 1. Configuração da Unidade ADC
    // O sistema configura a unidade ADC1 para operar no modo OneShot (leitura sob demanda).
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    // 2. Configuração dos Canais
    // O sistema define a atenuação de 12dB para permitir leituras de tensão de 0V até ~3.3V (padrão ESP32).
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    // O sistema aplica a configuração aos canais vertical e horizontal.
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK_VERT_CHAN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, JOYSTICK_HOR_CHAN, &config));

    // 3. Configuração do Botão
    // O sistema configura o pino do botão como entrada e ativa o resistor de pull-up interno.
    gpio_set_direction(JOYSTICK_SW_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(JOYSTICK_SW_PIN, GPIO_PULLUP_ONLY);
}

/**
 * @brief Tarefa responsável pela leitura contínua, filtragem e atualização dos dados do joystick.
 */
void joystick_read_task(void *pvParameters) {
    // O sistema aloca buffers circulares para armazenar o histórico de leituras para a média móvel.
    int buf_x[JOY_AVG_SAMPLES] = {0};
    int buf_y[JOY_AVG_SAMPLES] = {0};
    int idx = 0;
    long sum_x = 0;
    long sum_y = 0;
    
    // --- Pré-carregamento dos Filtros ---
    // O sistema realiza uma sequência rápida de leituras iniciais para preencher os buffers.
    // Isso evita que a média comece em zero (o que causaria um movimento brusco dos servos ao iniciar).
    int val_x, val_y;
    long cx=0, cy=0;
    for(int k=0; k<50; k++) { 
        adc_oneshot_read(adc1_handle, JOYSTICK_HOR_CHAN, &val_x);
        adc_oneshot_read(adc1_handle, JOYSTICK_VERT_CHAN, &val_y);
        cx+=val_x; cy+=val_y; 
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
    // O sistema calcula a média inicial e preenche todo o buffer com este valor estável.
    int rcx = cx/50; int rcy = cy/50;
    for(int i=0; i<JOY_AVG_SAMPLES; i++) { buf_x[i]=rcx; buf_y[i]=rcy; sum_x+=rcx; sum_y+=rcy; }

    int last_btn = 1;
    uint32_t last_press = 0;

    for (;;) {
        // --- Leitura ADC ---
        // O sistema solicita a conversão analógico-digital atual para ambos os eixos.
        int raw_x = 0;
        int raw_y = 0;
        adc_oneshot_read(adc1_handle, JOYSTICK_HOR_CHAN, &raw_x);
        adc_oneshot_read(adc1_handle, JOYSTICK_VERT_CHAN, &raw_y);

        // --- Filtro de Média Móvel ---
        // O sistema subtrai a leitura mais antiga do acumulador e adiciona a nova leitura.
        sum_x -= buf_x[idx]; sum_y -= buf_y[idx]; 
        buf_x[idx] = raw_x; buf_y[idx] = raw_y;
        sum_x += raw_x; sum_y += raw_y;

        // O sistema avança o índice do buffer circular.
        idx++; if (idx >= JOY_AVG_SAMPLES) idx = 0;
        
        // O sistema calcula a média simples dividindo o acumulador pelo número de amostras.
        int sx = sum_x / JOY_AVG_SAMPLES;
        int sy = sum_y / JOY_AVG_SAMPLES;

        // --- Leitura do Botão com Debounce ---
        int btn = gpio_get_level(JOYSTICK_SW_PIN);
        bool trig = false;
        
        // O sistema detecta a borda de descida (pressionamento) e verifica o tempo decorrido
        // para evitar múltiplos disparos (debounce) ou disparos muito longos.
        if (last_btn == 1 && btn == 0) { 
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if ((now - last_press) < 500 && (now - last_press) > 50) trig = true;
            last_press = now;
        }
        last_btn = btn;

        // --- Atualização Global ---
        // O sistema adquire o Mutex para escrever os dados processados na estrutura global compartilhada.
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            global_data.joy_x_raw = sx;
            global_data.joy_y_raw = sy;
            global_data.btn_pressed = (btn == 0);
            
            // Se um clique válido foi detectado, o sistema sinaliza o gatilho de calibração.
            if (trig) global_data.calibration_trigger = true;
            
            xSemaphoreGive(data_mutex);
        }
        // O sistema aguarda 10ms antes da próxima amostragem.
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}