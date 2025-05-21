# 🤖 DogRover - BitDogLab 🤖

O DogRover é a 4º atividade da Fase 2 do EmbarcaTech, se trata de um projeto com a temática de Robótica Móvel, sendo uma interface para simulação que controla e montiora um rover espacial de exploração em um outro planeta.
O sistema possui uma interface WEB que envia dados via Wi-Fi para a BitDogLab, que por sua vez interpreta esses dados recebidos, gerando uma interface gráfica com uso dos periféricos presentes na placa, simulando a atuação do rover espacial em um planeta extraterrestre.

Desenvolvido por: Taylan Mayckon

---
## 📽️ Link do Video de Demonstração:
[YouTube](https://youtu.be/0b_vUmmwDDg)
---

## 📌 **Funcionalidades Implementadas**

- FreeRTOS para geração de diferentes Tasks: Foram geradas sete tasks para o desenvolvimento do projeto
- Servidor WEB: Foi gerado um servidor WEB através da RP2040, com o qual o usuário consegue interagir
- Mapa interativo: A matriz de LEDs 5x5 presente na BitDogLab ilustra um mapa que indica a posição do Rover, obstáculos e pontos de coleta
- Logs de informações: Informações são continuamente exibidas no Display OLED da BitDogLab e no servidor WEB, sendo eles a bateria atual do Rover, o modo de operação (WEB ou BitDogLab) e quantidade de coletas

---

## 🛠 **Recursos Utilizados**

- **FreeRTOS:** é um sistema operacional de código aberto e tempo real (RTOS) projetado para microcontroladores e dispositivos embarcados. Ele permite a criação de diferentes tarefas e faz o gerenciamento das mesmas para serem executadas de forma paralela.
- **Display OLED:** foi utilizado o ssd1306, que possui 128x64 pixels, para informações visuais dos dados lidos.
- **Matriz de LEDs Endereçáveis:** A BitDogLab possui uma matriz de 25 LEDs WS2812, que foi operada com o auxílio de uma máquina de estados para gerar o mapa interativo do sistema.
- **LED RGB:** Utilizado para sinalizar locais bloqueados e nível de bateria, através de sua intensidade.
- **Buzzers:** Emite sons para gerar alertas sonoros para as coletas e locais bloqueados.

---

## 📂 **Estrutura do Código**

```
📂 AlertaEnchentes/
├── 📄 DogRover.c                      # Código principal do projeto
├── 📄 lwipopts.h                      # Configurações para o lwIP
├──── 📂lib
├───── 📄 FreeRTOSConfig.h             # Arquivos de configuração para o FreeRTOS
├───── 📄 font.h                       # Fonte utilizada no Display I2C
├───── 📄 led_matrix.c                 # Funções para manipulação da matriz de LEDs endereçáveis
├───── 📄 led_matrix.h                 # Cabeçalho para o led_matrix.c
├───── 📄 ssd1306.c                    # Funções que controlam o Display I2C
├───── 📄 ssd1306.h                    # Cabeçalho para o ssd1306.c
├───── 📄 ws2812.pio                   # Máquina de estados para operar a matriz de LEDs endereçáveis
├── 📄 CMakeLists.txt                  # Configurações para compilar o código corretamente
└── 📄 README.md                       # Documentação do projeto
```

---
