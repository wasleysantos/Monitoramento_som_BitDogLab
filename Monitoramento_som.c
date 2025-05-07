#include <stdio.h>               
#include <math.h>               
#include "pico/stdlib.h"        
#include "hardware/adc.h"       
#include "hardware/dma.h"       
#include "hardware/timer.h"     
#include "neopixel.c"           

// --- Configuração do ADC e microfone ---
#define CANAL_MICROFONE 2                                // Canal 2 do ADC (microfone)
#define PINO_MICROFONE (26 + CANAL_MICROFONE)            // Pino GPIO 28 para leitura analógica
#define DIVISOR_ADC 96.f                                 // Frequência de amostragem do ADC
#define AMOSTRAS 200                                     // Número de amostras a serem coletadas
#define AJUSTAR_ADC(x) (x * 3.3f / (1 << 12u) - 1.65f)    // Conversão de valor ADC para tensão centrada em 0V

// --- Configuração dos LEDs ---
#define PINO_LED 7                                        // Pino GPIO do NeoPixel
#define TOTAL_LEDS 25                                     // Total de LEDs conectados
#define COLUNAS 5                                         // Número de colunas de LEDs (5 colunas)
#define LEDS_NIVEL (TOTAL_LEDS / COLUNAS)                 // LEDs por linha (nível)
#define LIMIAR_SOM 0.2f                                  // Limite de detecção de som
#define ABS(x) ((x < 0) ? (-x) : (x))                    // Macro para calcular o valor absoluto

// --- Variáveis globais ---
uint dma_canal;                          // Canal de DMA
dma_channel_config dma_config;           // Configuração do canal de DMA
uint16_t adc_buffer[AMOSTRAS];           // Buffer para armazenar amostras do ADC
volatile bool mic_pronto = false;        // Flag para indicar se a leitura do microfone está pronta
float media_som = 0.0f;                  // Valor médio ajustado da leitura do microfone (nível de som)

// --- Função: coleta amostras via DMA ---
void amostrar_microfone() {
    adc_fifo_drain();  // Limpa o FIFO do ADC
    adc_run(false);    // Garante que o ADC não está rodando
    dma_channel_configure(dma_canal, &dma_config,
        adc_buffer, &(adc_hw->fifo), AMOSTRAS, true); // Configura DMA para ler dados do ADC
    adc_run(true);     // Inicia a leitura do ADC
    dma_channel_wait_for_finish_blocking(dma_canal); // Aguarda o fim da coleta
    adc_run(false);    // Para o ADC após a leitura
}

// --- Função: calcula a potência RMS do microfone ---
float potencia_microfone() {
    float soma = 0.f;
    for (uint i = 0; i < AMOSTRAS; ++i)
        soma += adc_buffer[i] * adc_buffer[i]; // Soma dos quadrados dos valores do ADC
    return sqrt(soma / AMOSTRAS); // Retorna a raiz quadrada da média (potência RMS)
}

// --- Função: converte a voltagem em nível de som (0 a 5) ---
uint8_t obter_nivel(float valor) {
    if (valor < LIMIAR_SOM)
        return 0; // Se o valor for abaixo do limiar, não há som detectado

    uint nivel = 0;
    if (valor > 0.55f) nivel = 5;     // Muito alto
    else if (valor > 0.35f) nivel = 4; // Alto
    else if (valor > 0.25f) nivel = 3; // Médio
    else if (valor > 0.18f) nivel = 2; // Baixo
    else if (valor > 0.10f) nivel = 1; // Muito baixo
    return nivel;
}

// --- Callback do timer: chamado a cada 50ms para ler o microfone ---
bool ler_microfone_callback(struct repeating_timer *t) {
    amostrar_microfone();                         // Coleta amostras do microfone
    float potencia = potencia_microfone();         // Calcula a potência RMS
    media_som = 2.f * ABS(AJUSTAR_ADC(potencia));  // Ajusta e armazena o nível de som
    mic_pronto = true;                            // Sinaliza que a leitura está pronta
    return true;
}

// --- Função principal ---
int main() {
    stdio_init_all();          // Inicializa a comunicação serial
    sleep_ms(2000);            // Aguarda 2 segundos para inicializar

    npInit(PINO_LED, TOTAL_LEDS); // Inicializa os LEDs NeoPixel

    // --- Configuração do ADC ---
    printf("Preparando ADC...\n");
    adc_gpio_init(PINO_MICROFONE);    // Configura o pino do microfone para entrada analógica
    adc_init();                       // Inicializa o ADC
    adc_select_input(CANAL_MICROFONE); // Seleciona o canal 2 do ADC para leitura
    adc_fifo_setup(true, true, 1, false, false); // Configura o FIFO para leitura de 1 bit
    adc_set_clkdiv(DIVISOR_ADC);        // Configura o divisor de clock do ADC

    // --- Configuração do DMA ---
    printf("Preparando DMA...\n");
    dma_canal = dma_claim_unused_channel(true); // Requisita um canal de DMA não utilizado
    dma_config = dma_channel_get_default_config(dma_canal); // Obtém configuração padrão para o DMA
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_16); // Define o tamanho dos dados para 16 bits
    channel_config_set_read_increment(&dma_config, false); // Desabilita incremento de leitura
    channel_config_set_write_increment(&dma_config, true);  // Habilita incremento de escrita
    channel_config_set_dreq(&dma_config, DREQ_ADC); // Define a fonte de dados como ADC

    // --- Timer periódico (50ms) para capturar o som ---
    struct repeating_timer timer;
    add_repeating_timer_ms(50, ler_microfone_callback, NULL, &timer); // Configura timer para callback a cada 50ms

    printf("Início do Loop\n");

    // --- Loop principal ---
    while (true) {
        if (!mic_pronto) {
            sleep_ms(10); // Aguarda nova leitura
            continue;
        }
        mic_pronto = false;

        uint nivel = obter_nivel(media_som); // Converte a voltagem em nível de som

        npClear(); // Limpa todos os LEDs

        // Define as cores para cada linha de LEDs (nível de som)
        uint8_t cores[5][3] = {
            {0, 80, 0},     // Verde (linha 1)
            {80, 80, 0},     // Amarelo (linha 2)
            {80, 80, 0},    // Amarelo (linha 3)
            {80, 0, 0},     // Vermelho (linha 4)
            {80, 0, 0}      // Vermelho (linha 5)
        };

        // Linha base (linha 1) sempre acesa
        for (int coluna = 0; coluna < COLUNAS; coluna++) {
            int indice_led_topo = 0 * COLUNAS + coluna;
            npSetLED(indice_led_topo,
                cores[0][0], cores[0][1], cores[0][2]); // Acende LEDs da linha base com cor verde
        }

        // Acende os LEDs de acordo com o nível de som nas outras linhas
        if (nivel > 0) {
            for (int coluna = 0; coluna < COLUNAS; coluna++) {
                int variacao = rand() % 3 - 1;  // Introduz variação aleatória para efeitos
                int nivel_temp = (int)nivel + variacao;
                if (nivel_temp < 0) nivel_temp = 0;  // Garante que o nível não seja negativo
                if (nivel_temp > 5) nivel_temp = 5;  // Garante que o nível não ultrapasse 5

                for (int linha = 1; linha < nivel_temp; linha++) {
                    int indice_led = linha * COLUNAS + coluna;
                    npSetLED(indice_led,
                        cores[linha][0],
                        cores[linha][1],
                        cores[linha][2]); // Acende o LED com a cor correspondente ao nível de som
                }
            }
        }

        npWrite(); // Atualiza os LEDs com as alterações feitas
        printf("Nível do som: %d  Voltagem: %.2f\n", nivel, media_som); // Exibe o nível de som e a voltagem
    }
}
