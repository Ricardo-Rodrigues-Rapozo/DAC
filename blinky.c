#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

// Configuração de parâmetros
#define BUFFER_SIZE 1000  // Tamanho do buffer de recepção

uint16_t buffer[BUFFER_SIZE]; // Buffer para armazenar os dados recebidos
uint32_t ui32SysClkFreq;  // Frequência de clock do sistema

// Inicialização da UART
void UART0_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configura pinos para RX e TX da UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configuração da UART
    UARTConfigSetExpClk(UART0_BASE, ui32SysClkFreq, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void UART0_Flush(void) {
    // Lê continuamente os dados até que o buffer esteja vazio
    while (UARTCharsAvail(UART0_BASE)) {
        UARTCharGet(UART0_BASE);  // Lê e descarta o byte do buffer de recepção
    }
}
// Função para receber dados pela UART
void UART0_ReceiveData(void) {
    static uint32_t index = 0;

    while (UARTCharsAvail(UART0_BASE)) {
        // Lê um byte recebido
        uint8_t byte1 = UARTCharGet(UART0_BASE);
        uint8_t byte2 = UARTCharGet(UART0_BASE);

        // Combina os dois bytes em um valor uint16_t
        uint16_t value = ((uint16_t)byte2 << 8) | byte1;


        // Armazena o valor no buffer
        buffer[index] = value;  // 65535 é o valor máximo de um uint16_t;

        // Incrementa o índice e faz a rotação para o início se necessário
        index = (index + 1) % BUFFER_SIZE;
    }
}

int main(void) {
    // Configuração do clock do sistema
    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                         SYSCTL_CFG_VCO_480), 120000000);

    // Inicialização da UART
    UART0_Init();


    while (1) {
        // Chama a função para receber dados pela UART
        UART0_ReceiveData();
    }
}
