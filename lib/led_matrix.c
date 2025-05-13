#include "led_matrix.h"

#define MATRIX_PIN 7
// Quantidade de pixels
#define NUM_PIXELS 25

// Buffer que armazena o frame atual, incluindo cores
Led_frame led_buffer= {{
    {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, 
    {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, 
    {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, 
    {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, 
    {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, 
}};

// FRAMES DA COR VERDE =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
Led_frame green_frame1 = {{
    {0,0,0},   {0,0,0},   {0,255,0}, {0,0,0},   {0,0,0}, 
    {0,0,0},   {0,0,0},   {0,0,0},   {0,255,0}, {0,0,0},
    {0,255,0}, {0,255,0}, {0,255,0}, {0,255,0}, {0,255,0},
    {0,0,0},   {0,0,0},   {0,0,0},   {0,255,0}, {0,0,0},
    {0,0,0},   {0,0,0},   {0,255,0}, {0,0,0},   {0,0,0},
}};

Led_frame green_frame2 = {{
    {0,0,0},   {0,0,0},   {0,0,0},   {0,255,0}, {0,0,0},
    {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},   {0,255,0},
    {0,0,0},   {0,255,0}, {0,255,0}, {0,255,0}, {0,255,0},
    {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},   {0,255,0},
    {0,0,0},   {0,0,0},   {0,0,0},   {0,255,0}, {0,0,0},
}};

Led_frame green_frame3 = {{
    {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},   {0,255,0}, 
    {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},   
    {0,255,0}, {0,0,0},   {0,255,0}, {0,255,0}, {0,255,0},
    {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},   
    {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},   {0,255,0}, 
}};

Led_frame green_frame4 = {{
    {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},   
    {0,255,0}, {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},  
    {0,255,0}, {0,255,0}, {0,0,0},   {0,255,0}, {0,255,0},
    {0,255,0}, {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},    
    {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},   
}};

Led_frame green_frame5 = {{
    {0,255,0}, {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},     
    {0,0,0},   {0,255,0}, {0,0,0},   {0,0,0},   {0,0,0},     
    {0,255,0}, {0,255,0}, {0,255,0}, {0,0,0},   {0,255,0}, 
    {0,0,0},   {0,255,0}, {0,0,0},   {0,0,0},   {0,0,0},      
    {0,255,0}, {0,0,0},   {0,0,0},   {0,0,0},   {0,0,0},    
}};

Led_frame green_frame6 = {{
    {0,0,0},   {0,255,0}, {0,0,0},   {0,0,0},   {0,0,0},     
    {0,0,0},   {0,0,0},   {0,255,0}, {0,0,0},   {0,0,0},     
    {0,255,0}, {0,255,0}, {0,255,0}, {0,255,0}, {0,0,0},   
    {0,0,0},   {0,0,0},   {0,255,0}, {0,0,0},   {0,0,0},        
    {0,0,0},   {0,255,0}, {0,0,0},   {0,0,0},   {0,0,0},     
}};
// Faz referência aos frames criados anteriormente
Led_frame *green_frames[] = {
    &green_frame1, 
    &green_frame2, 
    &green_frame3, 
    &green_frame4, 
    &green_frame5,
    &green_frame6, 
};

// FRAME DA COR AMARELA =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
Led_frame yellow_frame = {{
    {0,0,0}, {0,0,0}, {255,255,0}, {0,0,0}, {0,0,0}, 
    {0,0,0}, {0,0,0}, {255,255,0}, {0,0,0}, {0,0,0}, 
    {0,0,0}, {0,0,0}, {255,255,0}, {0,0,0}, {0,0,0}, 
    {0,0,0}, {0,0,0},   {0,0,0},   {0,0,0}, {0,0,0}, 
    {0,0,0}, {0,0,0}, {255,255,0}, {0,0,0}, {0,0,0}, 
}};

// FRAME DA COR VERMELHA =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
Led_frame red_frame = {{
    {0,0,0},     {255,0,0},     {255,0,0},     {255,0,0},    {0,0,0},
    {255,0,0}, {255,255,255}, {255,255,255}, {255,255,255}, {255,0,0},
    {255,0,0}, {255,255,255},   {255,0,0},   {255,255,255}, {255,0,0},
    {255,0,0}, {255,255,255}, {255,255,255}, {255,255,255}, {255,0,0},
    {0,0,0},     {255,0,0},     {255,0,0},     {255,0,0},    {0,0,0},
}};

static inline void put_pixel(uint32_t pixel_grb){
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

// Função que vai transformar valores correspondentes ao padrão RGB em dados binários
uint32_t urgb_u32(double r, double g, double b){
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

// Função que atualiza os Leds do vetor
void set_leds(float intensidade){
    uint32_t color; // Armazena os valores das cores

    // Define todos os LEDs com a cor especificada
    // Faz o processo de virar de cabeça para baixo o arranjo
    for (int i = NUM_PIXELS-1; i >= 0; i--){
        color = urgb_u32(intensidade*led_buffer.led[i].red, intensidade*led_buffer.led[i].green, intensidade*led_buffer.led[i].blue); // Converte as cores para o padrão aceito pela matriz
        put_pixel(color); // Liga o LED com um no buffer
    }
}

// Cor verde
void green_animation(uint frame_index){
    // Ordenando corretamente o vetor recebido no buffer
    int j = 0; // Variável para controle do index espelhado
    for(int i=0; i<25; i++){
        if(i>4 && i<10){
            led_buffer.led[i] = green_frames[frame_index]->led[9-j];
            j++;
        }
        else if(i>14 && i<20){
            led_buffer.led[i] = green_frames[frame_index]->led[19-j];
            j++;
        }
        else{
            j=0;
            led_buffer.led[i] = green_frames[frame_index]->led[i];
        }
    }
    set_leds(0.05);
}

// Cor amarela
void yellow_animation(float intensidade){
    // Ordenando corretamente o vetor recebido no buffer
    int j = 0; // Variável para controle do index espelhado
    for(int i=0; i<25; i++){
        if(i>4 && i<10){
            led_buffer.led[i] = yellow_frame.led[9-j];
            j++;
        }
        else if(i>14 && i<20){
            led_buffer.led[i] = yellow_frame.led[19-j];
            j++;
        }
        else{
            j=0;
            led_buffer.led[i] = yellow_frame.led[i];
        }
    }
    set_leds(intensidade);
}

// Cor vermelha
void red_animation(float intensidade){
    // Ordenando corretamente o vetor recebido no buffer
    int j = 0; // Variável para controle do index espelhado
    for(int i=0; i<25; i++){
        if(i>4 && i<10){
            led_buffer.led[i] = red_frame.led[9-j];
            j++;
        }
        else if(i>14 && i<20){
            led_buffer.led[i] = red_frame.led[19-j];
            j++;
        }
        else{
            j=0;
            led_buffer.led[i] = red_frame.led[i];
        }
    }
    set_leds(intensidade);
}
