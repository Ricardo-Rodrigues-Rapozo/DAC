#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

// Configura��o de par�metros
#define BUFFER_SIZE 1000  // Tamanho do buffer de recep��o

uint16_t buffer[BUFFER_SIZE]; // Buffer para armazenar os dados recebidos
uint32_t ui32SysClkFreq;  // Frequ�ncia de clock do sistema

// Inicializa��o da UART
void UART0_Init(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configura pinos para RX e TX da UART
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configura��o da UART
    UARTConfigSetExpClk(UART0_BASE, ui32SysClkFreq, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void UART0_Flush(void) {
    // L� continuamente os dados at� que o buffer esteja vazio
    while (UARTCharsAvail(UART0_BASE)) {
        UARTCharGet(UART0_BASE);  // L� e descarta o byte do buffer de recep��o
    }
}
// Fun��o para receber dados pela UART
void UART0_ReceiveData(void) {
    static uint32_t index = 0;

    while (UARTCharsAvail(UART0_BASE)) {
        // L� um byte recebido
        uint8_t byte1 = UARTCharGet(UART0_BASE);
        uint8_t byte2 = UARTCharGet(UART0_BASE);

        // Combina os dois bytes em um valor uint16_t
        uint16_t value = ((uint16_t)byte2 << 8) | byte1;


        // Armazena o valor no buffer
        buffer[index] = value;  // 65535 � o valor m�ximo de um uint16_t;

        // Incrementa o �ndice e faz a rota��o para o in�cio se necess�rio
        index = (index + 1) % BUFFER_SIZE;
    }
}

int main(void) {
    // Configura��o do clock do sistema
    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                         SYSCTL_CFG_VCO_480), 120000000);

    // Inicializa��o da UART
    UART0_Init();


    while (1) {
        // Chama a fun��o para receber dados pela UART
        UART0_ReceiveData();
    }
}
