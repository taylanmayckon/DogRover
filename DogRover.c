#include <stdio.h>
#include "pico/stdlib.h"

// Includes do FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include <string.h>              // Biblioteca manipular strings
#include <stdlib.h>              // funções para realizar várias operações, incluindo alocação de memória dinâmica (malloc)

#include "hardware/adc.h"        // Biblioteca da Raspberry Pi Pico para manipulação do conversor ADC
#include "pico/cyw43_arch.h"     // Biblioteca para arquitetura Wi-Fi da Pico com CYW43  

#include "lwip/pbuf.h"           // Lightweight IP stack - manipulação de buffers de pacotes de rede
#include "lwip/tcp.h"            // Lightweight IP stack - fornece funções e estruturas para trabalhar com o protocolo TCP
#include "lwip/netif.h"          // Lightweight IP stack - fornece funções e estruturas para trabalhar com interfaces de rede (netif)

// Imports para configuração dos periféricos da BitDogLab
#include "led_matrix.h"
#include "ssd1306.h"
#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "font.h"

// Credenciais WIFI - Tome cuidado se publicar no github!
#define WIFI_SSID "Jr telecom _ Taylan"
#define WIFI_PASSWORD "Suta3021"

// Definição dos pinos dos LEDs
#define LED_PIN CYW43_WL_GPIO_LED_PIN   // GPIO do CI CYW43
#define LED_BLUE_PIN 12                 // GPIO12 - LED azul
#define LED_GREEN_PIN 11                // GPIO11 - LED verde
#define LED_RED_PIN 13                  // GPIO13 - LED vermelho

// Buzzers
#define BUZZER_A 21 
#define BUZZER_B 10

// Constantes para a matriz de leds
#define IS_RGBW false
#define LED_MATRIX_PIN 7

// Definições da I2C
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C

// Variáveis da PIO declaradas no escopo global
PIO pio;
uint sm;
// Booleano para o display
bool cor = true;

// Variáveis do PWM (setado para freq. de 312,5 Hz)
uint wrap = 2000;
uint clkdiv = 25;

// Para descrever o rover
typedef struct {
    int position;
    int collects;
    float battery;
    bool alert;
} Rover;

// Iniciando o rover centralizado e com tudo zerado
Rover rover = {12, 0, 100.00, false};

Led_frame led_matrix; // Matriz para gerar as cores

typedef enum{ // Tipos de célula
    CELL_FREE,
    CELL_OBSTACLE,
    CELL_PLAYER,
    CELL_COLLECT
} Celltype;

Celltype grid[NUM_PIXELS] = { // Grid com o conteúdo de cada célula (LED)
    CELL_FREE, CELL_FREE, CELL_OBSTACLE, CELL_OBSTACLE, CELL_FREE,
    CELL_FREE, CELL_FREE, CELL_FREE,     CELL_FREE,     CELL_FREE,
    CELL_FREE, CELL_FREE,  CELL_PLAYER,  CELL_FREE,     CELL_FREE,
    CELL_FREE, CELL_OBSTACLE, CELL_FREE, CELL_COLLECT,  CELL_FREE,
    CELL_COLLECT, CELL_FREE, CELL_FREE,  CELL_OBSTACLE, CELL_FREE
};



// PROTÓTIPOS DE FUNÇÕES =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Inicializar os Pinos GPIO para acionamento dos LEDs da BitDogLab
void gpio_led_bitdog(void);

// Função de callback ao aceitar conexões TCP
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);

// Função de callback para processar requisições HTTP
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);

// Leitura da temperatura interna
float temp_read(void);

// Tratamento do request do usuário
void user_request(char **request);


// FUNÇÕES AUXILIARES =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Função para configurar o PWM e iniciar com 0% de DC
void set_pwm(uint gpio, uint wrap){
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_clkdiv(slice_num, clkdiv);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_enabled(slice_num, true); 
    pwm_set_gpio_level(gpio, 0);
}

// Inicializar os Pinos GPIO para acionamento dos LEDs da BitDogLab
void gpio_led_bitdog(void){
    // Configuração dos LEDs como saída
    gpio_init(LED_BLUE_PIN);
    gpio_set_dir(LED_BLUE_PIN, GPIO_OUT);
    gpio_put(LED_BLUE_PIN, false);
    
    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_put(LED_GREEN_PIN, false);
    
    gpio_init(LED_RED_PIN);
    gpio_set_dir(LED_RED_PIN, GPIO_OUT);
    gpio_put(LED_RED_PIN, false);
}

// Função de callback ao aceitar conexões TCP
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err){
    tcp_recv(newpcb, tcp_server_recv);
    return ERR_OK;
}


// Tratamento do request do usuário - digite aqui
void user_request(char **request){
    // Inicialmente tá só ligando/desligando leds para depurar na placa

    grid[rover.position] = CELL_FREE; // Prepara o grid para o movimento
    // Seta para cima
    if (strstr(*request, "GET /up") != NULL){
        if(rover.position>=5){ // Restringe a posição para a anterior, caso o movimento vá além do permitido
            rover.position-=5;
        }
        else{
            rover.alert=true;
        }
    }
    // Seta para baixo
    else if (strstr(*request, "GET /down") != NULL){
        if(rover.position<=19){ // Restringe a posição para a anterior, caso o movimento vá além do permitido
            rover.position+=5;
        }
        else{
            rover.alert=true;
        }
    }
    // Seta para a esquerda
    else if (strstr(*request, "GET /left") != NULL){
        // Quando permite o movimento
        if(rover.position % 5 != 0){
            rover.position--;
        }
        // Movimento invalido
        else{
            rover.alert = true;
        }
    }
    // Seta para a direita
    else if (strstr(*request, "GET /right") != NULL){
        // Quando permite o movimento
        if(rover.position % 5 != 4){
            rover.position++;
        }
        // Movimento invalido
        else{
            rover.alert = true;
        }
    }
    // Coletar
    else if (strstr(*request, "GET /collect") != NULL){
        gpio_put(LED_RED_PIN, 1);
    }
    // Automatico
    else if (strstr(*request, "GET /local") != NULL){
        gpio_put(LED_RED_PIN, 0);
    }

    // Relacionados ao módulo Wi-Fi
    else if (strstr(*request, "GET /on") != NULL){
        cyw43_arch_gpio_put(LED_PIN, 1);
    }
    else if (strstr(*request, "GET /off") != NULL){
        cyw43_arch_gpio_put(LED_PIN, 0);
    }

    grid[rover.position] = CELL_PLAYER; // Atualiza o GRID com a nova posição do player
};


// Leitura da temperatura interna
float temp_read(void){
    adc_select_input(4);
    uint16_t raw_value = adc_read();
    const float conversion_factor = 3.3f / (1 << 12);
    float temperature = 27.0f - ((raw_value * conversion_factor) - 0.706f) / 0.001721f;
    return temperature;
}

// Função de callback para processar requisições HTTP
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err){
    if (!p){
        tcp_close(tpcb);
        tcp_recv(tpcb, NULL);
        return ERR_OK;
    }

    // Alocação do request na memória dinámica
    char *request = (char *)malloc(p->len + 1);
    memcpy(request, p->payload, p->len);
    request[p->len] = '\0';

    printf("Request: %s\n", request);

    // Tratamento de request - Controle dos LEDs
    user_request(&request);
    
    // Leitura da temperatura interna
    float temperature = temp_read();

    // Cria a resposta HTML
    char html[1024];

    // Instruções html do webserver
    snprintf(html, sizeof(html),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "\r\n"
        "<html>\n"
        "<head>\n"
        "<title>DogRover</title>\n"
        "<style>\n"
        "body { background : #5dff65; font-family: Arial, sans-serif; text-align: center; margin-top: 50px; }\n"
        "h1 { font-size: 64px; margin-bottom: 30px; }\n"
        "button { background-color: LightGray; font-size: 36px; margin: 10px; padding: 20px 40px; border-radius: 10px; }\n"
        ".subtitle { font-size: 48px; margin-top: 30px; color: #333; }\n"
        "</style>\n"
        "</head>\n"
        "<body>\n"
        "<h1> DogRover </h1>\n"
        "<div>\n"
        "<p class=\"subtitle\">Direcionais</p>\n"
        "<form action='./up'><button>&#8593; Cima</button></form>\n"
        "<form action='./left'><button>&#8592; Esquerda</button></form>\n"
        "<form action='./right'><button>Direita &#8594;</button></form>\n"
        "<form action='./down'><button>&#8595; Baixo</button></form>\n"
        "</div>\n"
        "<p class=\"subtitle\">Acoes</p>\n"
        "<form action='./collect'><button>Coletar</button></form>\n"
        "<form action='./local'><button>Local</button></form>\n"
        "<p class=\"subtitle\">Bateria: %.2f %%</p>\n"
        "<p class=\"subtitle\">Materiais coletados: %d</p>\n"
        "<p class=\"subtitle\">Temperatura do Robo: %.2f &deg;C</p>\n"
        "</body>\n"
        "</html>\n",
        rover.battery, rover.collects, temperature); // Aqui vão as variaveis: bateria, count_materiais, temperatura

    // Escreve dados para envio (mas não os envia imediatamente).
    tcp_write(tpcb, html, strlen(html), TCP_WRITE_FLAG_COPY);

    // Envia a mensagem
    tcp_output(tpcb);

    //libera memória alocada dinamicamente
    free(request);
    
    //libera um buffer de pacote (pbuf) que foi alocado anteriormente
    pbuf_free(p);

    return ERR_OK;
}


// TASKS CRIADAS =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Task do servidor web
void vWebServerTask(){
    //Inicializa a arquitetura do cyw43
    while (cyw43_arch_init()){
        printf("Falha ao inicializar Wi-Fi\n");
        sleep_ms(100);
        return;
    }

    // GPIO do CI CYW43 em nível baixo
    cyw43_arch_gpio_put(LED_PIN, 0);

    // Ativa o Wi-Fi no modo Station, de modo a que possam ser feitas ligações a outros pontos de acesso Wi-Fi.
    cyw43_arch_enable_sta_mode();

    // Conectar à rede WiFI - fazer um loop até que esteja conectado
    printf("Conectando ao Wi-Fi...\n");
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 20000)){
        printf("Falha ao conectar ao Wi-Fi\n");
        sleep_ms(100);
        return;
    }
    printf("Conectado ao Wi-Fi\n");

    // Caso seja a interface de rede padrão - imprimir o IP do dispositivo.
    if (netif_default){
        printf("IP do dispositivo: %s\n", ipaddr_ntoa(&netif_default->ip_addr));
    }

    // Configura o servidor TCP - cria novos PCBs TCP. É o primeiro passo para estabelecer uma conexão TCP.
    struct tcp_pcb *server = tcp_new();
    if (!server){
        printf("Falha ao criar servidor TCP\n");
        return;
    }

    //vincula um PCB (Protocol Control Block) TCP a um endereço IP e porta específicos.
    if (tcp_bind(server, IP_ADDR_ANY, 80) != ERR_OK){
        printf("Falha ao associar servidor TCP à porta 80\n");
        return;
    }

    // Coloca um PCB (Protocol Control Block) TCP em modo de escuta, permitindo que ele aceite conexões de entrada.
    server = tcp_listen(server);

    // Define uma função de callback para aceitar conexões TCP de entrada. É um passo importante na configuração de servidores TCP.
    tcp_accept(server, tcp_server_accept);
    printf("Servidor ouvindo na porta 80\n");

    while (true){
        /* 
        * Efetuar o processamento exigido pelo cyw43_driver ou pela stack TCP/IP.
        * Este método deve ser chamado periodicamente a partir do ciclo principal 
        * quando se utiliza um estilo de sondagem pico_cyw43_arch 
        */
        cyw43_arch_poll(); // Necessário para manter o Wi-Fi ativo
        vTaskDelay(pdMS_TO_TICKS(100));      // Reduz o uso da CPU
    }

    //Desligar a arquitetura CYW43.
    cyw43_arch_deinit();
    return;
}


// Task para controlar a matriz de LEDs endereçáveis
// Função para atualizar a matriz de LEDs com as cores do grid
void update_led_matrix_from_grid(){
    Led_color green = {0, 255, 0};
    Led_color blue = {0, 0, 255};
    Led_color orange = {255, 165, 0};
    Led_color off = {0, 0, 0};

    for (int i = 0; i < NUM_PIXELS; i++) {
        switch (grid[i]) {
            case CELL_FREE:
                led_matrix.led[i] = off;
                break;
            case CELL_OBSTACLE:
                led_matrix.led[i] = orange;
                break;
            case CELL_PLAYER:
                led_matrix.led[i] = green;
                break;
            case CELL_COLLECT:
                led_matrix.led[i] = blue;
                break;
        }
    }
}

void vLedMatrixTask(){
    // Inicializando a PIO
    pio = pio0;
    sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    ws2812_program_init(pio, sm, offset, LED_MATRIX_PIN, 800000, IS_RGBW);

    float matrix_intensity;

    Led_color green = {0, 255, 0};
    Led_color blue = {0, 0, 255};
    Led_color orange = {255, 165, 0};
    Led_color off = {0, 0, 0};

    while(true){
        // Atualiza as cores na matriz RGB
        update_led_matrix_from_grid();

        // Envia para a matriz física da BitDogLab
        matrix_update_leds(&led_matrix, 0.01);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// Task para controlar o display OLED
void vDisplayOLEDTask(){
    // Configurando a I2C
    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);                    // Set the GPIO pin function to I2C
    gpio_pull_up(I2C_SDA);                                        // Pull up the data line
    gpio_pull_up(I2C_SCL);                                        // Pull up the clock line
    ssd1306_t ssd;                                                // Inicializa a estrutura do display
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
    ssd1306_config(&ssd);                                         // Configura o display
    ssd1306_send_data(&ssd);                                      // Envia os dados para o display
    // Limpa o display. O display inicia com todos os pixels apagados.
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    while(true){
        ssd1306_fill(&ssd, false); // Limpa o display

        // Frame que será reutilizado para todos
        ssd1306_rect(&ssd, 0, 0, 128, 64, cor, !cor);
        // Nome superior
        ssd1306_rect(&ssd, 0, 0, 128, 12, cor, cor); // Fundo preenchido
        ssd1306_draw_string(&ssd, "DogRover", 4, 3, true); // String: Semaforo
        ssd1306_draw_string(&ssd, "TM", 107, 3, true);
        // Bateria
        ssd1306_draw_string(&ssd, "BATERIA: 67", 4, 16, false);
        // Status
        ssd1306_draw_string(&ssd, "STATUS: COLETA", 4, 28, false);
        // Coletas
        ssd1306_draw_string(&ssd, "COLETAS: 5", 4, 40, false);

        ssd1306_send_data(&ssd); // Envia os dados para o display, atualizando o mesmo
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Task para os alertas sonoros
void vBuzzerTask(){
    set_pwm(BUZZER_A, wrap);
    set_pwm(BUZZER_B, wrap);

    int buzzer_count = 0;

    while(true){
        // Quando algum alerta é acionado
        if(rover.alert){
            if(buzzer_count%2==0){
                pwm_set_gpio_level(BUZZER_A, wrap*0.05);
                pwm_set_gpio_level(BUZZER_B, wrap*0.05);
            }
            else{
                pwm_set_gpio_level(BUZZER_A, 0);
                pwm_set_gpio_level(BUZZER_B, 0);
            }
            buzzer_count++;

            if(buzzer_count==4){ // Condição que para o alerta sonoro
                buzzer_count=0;
                rover.alert=false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Pequeno delay para reduzir o consumo de CPU
          
    }
}


// FUNÇÃO PRINCIPAL =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
void main(){
    //Inicializa todos os tipos de bibliotecas stdio padrão presentes que estão ligados ao binário.
    stdio_init_all();

    // Inicializar os Pinos GPIO para acionamento dos LEDs da BitDogLab
    gpio_led_bitdog();

    // Inicializa o conversor ADC
    adc_init();
    adc_set_temp_sensor_enabled(true);

    // Iniciando as Tasks
    xTaskCreate(vWebServerTask, "Web Server Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vLedMatrixTask, "Led Matrix Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vDisplayOLEDTask, "Display OLED Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL); 
    xTaskCreate(vBuzzerTask, "Buzzer Alert Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

    vTaskStartScheduler();
    panic_unsupported();
}