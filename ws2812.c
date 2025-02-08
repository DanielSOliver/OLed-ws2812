#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

const uint LED_AZUL = 12;
const uint LED_VERMELHO = 13;
const uint LED_VERDE = 11;
const uint BOTAO_VERDE = 5 ;  // Botão para LED Verde
const uint BOTAO_AZUL = 6 ;   // Botão para LED Azul

#define NUM_PIXELS 25
#define WS2812_PIN 7
#define IS_RGBW false
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define MAX_STRING_LENGTH 100


// Definição das cores (valores iniciais)
uint8_t led_r = 0;
uint8_t led_g = 0;
uint8_t led_b = 20;


volatile uint8_t buffer_ativo = 0;
static volatile uint32_t last_time = 0; // Armazena o tempo do último evento (em microssegundos)

volatile bool estado_led_verde = false;  // Inicia como desligado
volatile bool estado_led_azul = false;   // Inicia como desligado


// Função para enviar um valor de cor para o LED
static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

// Função para converter RGB em 32 bits
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

// Definição dos 11 buffers de LEDs
bool led_buffers[10][NUM_PIXELS] = {
  

   {0, 1, 1, 1, 0,
    0, 1, 0, 1, 0,
    0, 1, 0, 1, 0,
    0, 1, 0, 1, 0,
    0, 1, 1, 1, 0},

   {0, 1, 1, 1, 0,
    0, 0, 1, 0, 0,
    0, 0, 1, 0, 0,
    0, 1, 1, 0, 0,
    0, 0, 1, 0, 0},


   {0, 1, 1, 1, 0,
    0, 1, 0, 0, 0,
    0, 1, 1, 1, 0,
    0, 0, 0, 1, 0,
    0, 1, 1, 1, 0},


   {0, 1, 1, 1, 0,
    0, 0, 0, 1, 0,
    0, 1, 1, 1, 0,
    0, 0, 0, 1, 0,
    0, 1, 1, 1, 0},

   {0, 1, 0, 0, 0,
    0, 0, 0, 1, 0,
    0, 1, 1, 1, 0,
    0, 1, 0, 1, 0,
    0, 1, 0, 1, 0},

    {0, 1, 1, 1, 0,
     0, 0, 0, 1, 0, 
     0, 1, 1, 1, 0, 
     0, 1, 0, 0, 0, 
     0, 1, 1, 1, 0},

   {0, 1, 1, 1, 0,
    0, 1, 0, 1, 0,
    0, 1, 1, 1, 0,
    0, 1, 0, 0, 0,
    0, 1, 1, 1, 0},

   {0, 0, 1, 0, 0,
    0, 0, 1, 0, 0, 
    0, 1, 0, 0, 0,
    0, 0, 0, 0, 1,
    1, 1, 1, 1, 0},

   {0, 1, 1, 1, 0,
    0, 1, 0, 1, 0,
    0, 1, 1, 1, 0,
    0, 1, 0, 1, 0, 
    0, 1, 1, 1, 0},


   {0, 1, 1, 1, 0,
    0, 0, 0, 1, 0,
    0, 1, 1, 1, 0,
    0, 1, 0, 1, 0,
    0, 1, 1, 1, 0}
};


// Função auxiliar para alternar estado do LED e exibir mensagem
void alternar_led(uint gpio_led, const char *nome_led, volatile bool *estado_led) {
    bool estado_atual = gpio_get(gpio_led);
    gpio_put(gpio_led, !estado_atual);
    *estado_led = !estado_atual;  // Atualiza o estado do LED na variável global

    printf("%s %s\n", nome_led, estado_atual ? "Desligado" : "Ligado");
}

// Função de interrupção com debouncing aprimorada
void debounce_irq_handler(uint gpio, uint32_t events) {
    uint32_t current_time = to_us_since_boot(get_absolute_time());

    // Verifica se passou tempo suficiente desde o último evento (200ms debounce)
    if (current_time - last_time > 300000) {
        last_time = current_time;

        if (gpio == BOTAO_VERDE) {
            alternar_led(LED_VERDE, "LED Verde", &estado_led_verde);
        } else if (gpio == BOTAO_AZUL) {
            alternar_led(LED_AZUL, "LED Azul", &estado_led_azul);
        }
    }
}


// Função para atualizar os LEDs com base no buffer ativo
void set_one_led(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t color = urgb_u32(r, g, b);
    bool* buffer = led_buffers[buffer_ativo];
    for (int i = 0; i < NUM_PIXELS; i++) {
        put_pixel(buffer[i] ? color : 0);
    }
}


int main()
{

    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);


    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400 * 1000);
    stdio_init_all(); // Inicializa comunicação USB CDC para monitor serial

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C); // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA); // Pull up the data line
    gpio_pull_up(I2C_SCL); // Pull up the clock line
    ssd1306_t ssd; // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd); // Configura o display
    ssd1306_send_data(&ssd); // Envia os dados para o display

     // Configuração CORRETA dos LEDs (LED Azul inicializado)
    gpio_init(LED_VERDE);
    gpio_set_dir(LED_VERDE, GPIO_OUT);
   
    gpio_init(LED_AZUL);  // Correção crítica: inicializa o pino do LED Azul
    gpio_set_dir(LED_AZUL, GPIO_OUT);

    gpio_init(LED_VERMELHO);
    gpio_set_dir(LED_VERMELHO, GPIO_OUT);
    
    // Configuração CORRETA dos botões (Botão 5 para Verde, Botão 6 para Azul)
    gpio_init(BOTAO_VERDE);
    gpio_set_dir(BOTAO_VERDE, GPIO_IN);
    gpio_pull_up(BOTAO_VERDE);

    gpio_init(BOTAO_AZUL);
    gpio_set_dir(BOTAO_AZUL, GPIO_IN);
    gpio_pull_up(BOTAO_AZUL);

   
    gpio_set_irq_enabled_with_callback(BOTAO_VERDE, GPIO_IRQ_EDGE_FALL, true, &debounce_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_AZUL, GPIO_IRQ_EDGE_FALL, true, &debounce_irq_handler);


    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    bool cor = true;
    
    while (true) {
        if (stdio_usb_connected()) {
            // Limpa a tela e desenha elementos
            ssd1306_fill(&ssd, false);
            ssd1306_send_data(&ssd);
    
            // Atualiza mensagens dos LEDs
            const char *msg_verde = (estado_led_verde == 0) ? "led verde off" : "led verde on";
            ssd1306_draw_string(&ssd, msg_verde, 8, 10);
            
            const char *msg_azul = (estado_led_azul == 0) ? "led azul off" : "led azul on";
            ssd1306_draw_string(&ssd, msg_azul, 15, 48);
    
            // Desenha borda
            ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor);
            ssd1306_send_data(&ssd);
    
            // Verifica entrada do usuário (não bloqueante)
            int c = getchar_timeout_us(0);
            if (c != PICO_ERROR_TIMEOUT) { 
                char input = (char)c;
                
                if (input >= '0' && input <= '9') {
                    buffer_ativo = input - '0';
                    set_one_led(led_r, led_g, led_b);
                } else {
                    for (int i = 0; i < NUM_PIXELS; i++) {
                        put_pixel(0);
                    }
                }
    
                // Visual da entrada
                ssd1306_fill(&ssd, false);
                ssd1306_draw_string(&ssd, (char[]){input, '\0'}, 20, 30);
                ssd1306_send_data(&ssd);
                sleep_ms(800);
            }
    
            sleep_ms(300); 
        } else {
            sleep_ms(500); 
        }
    }
  
}