#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "ssd1306.h"
#include "mqtt.h"
#include "wifi.h"
#include "lwip/ip_addr.h"
#include "lwip/dns.h"

// ---- WIFI ----
#define WIFI_SSID      "SuaRede"
#define WIFI_PASS      "SuaSenha"

// ---- ADAFRUIT IO ----
#define MQTT_BROKER_URL "io.adafruit.com"
#define MQTT_USER       "SeuUser"             // seu username Adafruit IO
#define MQTT_PASS       "SuaChaveAIO" // sua chave AIO
#define MQTT_CLIENT_ID  "SeuClientID"

// ---- FEEDS ----
#define MQTT_TOPIC_JOY      "SeuUser/feeds/joy"
#define MQTT_TOPIC_TEMP     "SeuUser/feeds/temp"


// Variáveis e definições globais
const uint led = 12;
const uint eixoX = 26;  // Pino de leitura do eixo X do joystick (ADC0)
const uint eixoY = 27;  // Pino de leitura do eixo Y do joystick (ADC1)
uint16_t valorX, valorY;  // Variáveis para armazenar os valores do joystick
char posicaoJoy[10] = " "; // Posição atual do joystick
char posicaoJoyAnterior[10] = " "; // Posição anterior do joystick
const uint I2C_SDA = 14;
const uint I2C_SCL = 15;
float temperatura = 0.0f; // Temperatura em graus Celsius
char texto_log[200]; // Buffer para mensagens de log
const uint ledVerde = 11;

// Buffer de exibição e área de renderização (globais)
uint8_t ssd[ssd1306_buffer_length];
struct render_area frame_area = {
    .start_column = 0,
    .end_column = ssd1306_width - 1,
    .start_page = 0,
    .end_page = ssd1306_n_pages - 1
};

// Mutex para proteger acesso ao display OLED
SemaphoreHandle_t xMutexDisplay;

// Função de inicialização
void setup(void)
{
    stdio_init_all();
    adc_init();  // Inicializa o ADC do RP2040
    gpio_init(led);
    gpio_set_dir(led, GPIO_OUT);
    gpio_init(ledVerde);
    gpio_set_dir(ledVerde, GPIO_OUT);
    adc_gpio_init(4);       // Pino de leitura da temperatura interna
    adc_gpio_init(eixoX);   // Eixo X do joystick
    adc_gpio_init(eixoY);   // Eixo Y do joystick

    // Inicializa I2C e pinos para o OLED
    i2c_init(i2c1, ssd1306_i2c_clock * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa o display OLED
    ssd1306_init();
    calculate_render_area_buffer_length(&frame_area);
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);

    // Configura o ADC para ler a temperatura interna
    adc_set_temp_sensor_enabled(true);

    // Cria o mutex para proteger acesso ao display
    xMutexDisplay = xSemaphoreCreateMutex();
}

// Função para exibir informações no OLED 
void displayOLED()
{
    if (xSemaphoreTake(xMutexDisplay, portMAX_DELAY))
    {
        memset(ssd, 0, ssd1306_buffer_length);

        char tempStr[30];
        char joyStr[30];

        sprintf(tempStr, "Temperatura: %d C", temperatura);
        sprintf(joyStr, "Joy: %s", posicaoJoy);

        ssd1306_draw_string(ssd, 0, 10, tempStr);
        ssd1306_draw_string(ssd, 0, 30, joyStr);

        render_on_display(ssd, &frame_area);

        xSemaphoreGive(xMutexDisplay);
    }
}

// Função para piscar o LED verde indicando que uma publicação foi feita
void ledPublicacao()
{
    for(int i=0; i<3; i++)
    {
        gpio_put(ledVerde, 1);
        vTaskDelay(40 / portTICK_PERIOD_MS);
        gpio_put(ledVerde, 0);
        vTaskDelay(60 / portTICK_PERIOD_MS);
    }
}

// Tarefa: Piscar LED 
void vBlinkTask(void *pvParameters)
{
    while (true)
    {    
        gpio_put(led, 1);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_put(led, 0);
        vTaskDelay(950 / portTICK_PERIOD_MS);
    }
}

// Tarefa: Leitura da Temperatura Interna 
void vTemperaturaTask(void *pvParameters)
{
    while (true)
    {
        adc_select_input(4);
        uint16_t leitura_bruta = adc_read();
        const float fator_conversao = 3.3f / (1 << 12); 
        float tensao = leitura_bruta * fator_conversao;
        float temp_float = 27.0f - (tensao - 0.706f) / 0.001721f;

        char tempStr[16];
        snprintf(tempStr, sizeof(tempStr), "%.2f", temp_float);

        mqtt_comm_publish(MQTT_TOPIC_TEMP, (const uint8_t *)tempStr, strlen(tempStr));
        ledPublicacao();
        displayOLED();

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

// Tarefa: Leitura do Joystick 
void vJoyTask(void *pvParameters)
{
    while (true)
    {
        char *estJOY = "";

        adc_select_input(0);  // Canal do eixo X
        valorX = adc_read();
        adc_select_input(1);  // Canal do eixo Y
        valorY = adc_read();

        // Determina a posição do joystick com base nas leituras dos eixos
        if (valorY > 2800 && valorX > 1000 && valorX < 3000)
        {
            strcpy(posicaoJoy, "direita");
        }
        else if (valorY < 1200 && valorX > 1000 && valorX < 3000)
        {
            strcpy(posicaoJoy, "esquerda");
        }
        else if (valorX > 2800 && valorY > 1000 && valorY < 3000)
        {
            strcpy(posicaoJoy, "cima");
        }
        else if (valorX < 1200 && valorY > 1000 && valorY < 3000)
        {
            strcpy(posicaoJoy, "baixo");
        }

        if (strcmp(posicaoJoy, posicaoJoyAnterior) != 0) // Verifica se a posição mudou
        {
            estJOY = posicaoJoy; // Se sim, atualiza a string de estado do joystick
            mqtt_comm_publish(MQTT_TOPIC_JOY, (const uint8_t *)estJOY, strlen(estJOY)); // Publica a nova posição no tópico MQTT
            ledPublicacao(); // Pisca LED verde para indicar publicação
            strcpy(posicaoJoyAnterior, posicaoJoy); // Atualiza a posição anterior do joystick
        }

        displayOLED(); // Atualiza OLED com a nova posição do joystick
        vTaskDelay(100 / portTICK_PERIOD_MS); // Aguarda 100 ms antes da próxima leitura
    }
}

int main()
{
    stdio_init_all();
    setup();

    printf("Iniciando Wi-Fi...\n");
    init_wifi(texto_log);
    printf("%s\n", texto_log);

    printf("Conectando ao Wi-Fi...\n");
    connect_to_wifi(WIFI_SSID, WIFI_PASS, texto_log);
    printf("%s\n", texto_log);

    while (!is_connected())
    {
        printf("Aguardando conexão Wi-Fi...\n");
        sleep_ms(1000);
    }
    printf("Wi-Fi conectado com sucesso!\n");

    // ----------- RESOLUÇÃO DE DNS PARA OBTER IP DO BROKER ------------
    ip_addr_t broker_ip;
    err_t err = dns_gethostbyname(MQTT_BROKER_URL, &broker_ip, NULL, NULL);

    // Se o IP foi resolvido imediatamente
    if (err == ERR_OK) {
        printf("Broker IP resolvido: %s\n", ipaddr_ntoa(&broker_ip));
    } else {
        // Espera pela resolução DNS de forma simples (bloqueante)
        while (dns_gethostbyname(MQTT_BROKER_URL, &broker_ip, NULL, NULL) != ERR_OK) {
            printf("Resolvendo DNS do broker...\n");
            sleep_ms(500);
        }
        printf("Broker IP resolvido após espera: %s\n", ipaddr_ntoa(&broker_ip));
    }

    // Passa o IP resolvido para mqtt_setup
    printf("Conectando ao broker MQTT...\n");
    mqtt_setup(MQTT_CLIENT_ID, ipaddr_ntoa(&broker_ip), MQTT_USER, MQTT_PASS, texto_log);
    printf("%s\n", texto_log);

    while (!mqtt_is_connected()) // Aguarda conexão com o broker MQTT
    {
        printf("Aguardando conexão MQTT...\n");
        sleep_ms(1000);
    }
    printf("Conectado ao broker MQTT!\n");

    // Criação das tarefas do FreeRTOS
    xTaskCreate(vBlinkTask, "Blink Task", 96, NULL, 1, NULL);
    xTaskCreate(vTemperaturaTask, "Temperatura Task", 1024, NULL, 1, NULL);
    xTaskCreate(vJoyTask, "Joystick Task", 1024, NULL, 1, NULL);

    vTaskStartScheduler(); // Inicia o escalonador do FreeRTOS

    while (true) {}
}