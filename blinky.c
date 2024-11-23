#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "math.h"

#define VECTOR_SIZE 99 // Tamanho do vetor de dados
#define ADC_BUFFER_SIZE 4000
#define FS 80000        // Taxa de amostragem em Hz


// Tabela de controle do uDMA (necessária para operações de transferência de dados)
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__((aligned(1024)));
#endif


// resolução tamanho do buffer /load  ---> = 100/ 750
// Variáveis globais
uint32_t ui32SysClkFreq; // Frequência do clock do sistema
uint32_t ui32PWMClockRate; // Frequência do clock do PWM
uint32_t freq_port = 2000; // Frequência da portadora PWM
float duty_cycle = 0;       // Ciclo de trabalho do PWM
volatile uint32_t flag = 0; // Flag de interrupção do Timer
uint32_t idx = 0;           // Índice do vetor
uint32_t idx_adc = 0;
uint32_t buffer[ADC_BUFFER_SIZE]; // Buffer de dados
float dataVector[VECTOR_SIZE]; // Vetor de dados recebidos
volatile uint32_t load;
// Manipulador de interrupção do Timer 0
void Timer0IntHandler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // Limpa a interrupção

    // Calcula o pulso com base no ciclo de trabalho
    float duty_cycle = dataVector[idx];
    uint32_t pulse_width = (duty_cycle * load);

    // Define a largura do pulso do PWM
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pulse_width);

    // Incrementa o índice, garantindo que ele permaneça dentro do tamanho do vetor
    idx = (idx + 1) % VECTOR_SIZE;

}

// Configura UART para comunicação serial
void ConfigureUART(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, ui32SysClkFreq, 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);
    UARTEnable(UART0_BASE);
}

// Configura uDMA para transferência de dados
void ConfigureUDMA(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA)) {}

    uDMAEnable();
    uDMAControlBaseSet(pui8ControlTable); // Define base de controle do uDMA

    UARTDMAEnable(UART0_BASE, UART_DMA_TX | UART_DMA_RX);

    // Configuração para recepção UART
    uDMAChannelAssign(UDMA_CH8_UART0RX);
    uDMAChannelAttributeDisable(UDMA_CHANNEL_UART0RX, UDMA_ATTR_ALL);
    uDMAChannelControlSet(UDMA_CHANNEL_UART0RX | UDMA_PRI_SELECT,
                          UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4);

    // Configuração para transmissão UART
    uDMAChannelAssign(UDMA_CH9_UART0TX);
}

// Recebe um vetor de float via UART utilizando uDMA
// Função para receber vetor de floats via UART utilizando uDMA
void UART0_ReceiveFloatVector(float* vector, uint32_t size) {
    // Configura a transferência do uDMA
    uDMAChannelTransferSet(UDMA_CHANNEL_UART0RX | UDMA_PRI_SELECT,
                           UDMA_MODE_BASIC,
                           (void *)(UART0_BASE + UART_O_DR),
                           (void *)vector,
                           size * sizeof(float));

    // Habilita o canal do uDMA
    uDMAChannelEnable(UDMA_CHANNEL_UART0RX);

    // Aguarda até que a transferência seja concluída
    while (uDMAChannelIsEnabled(UDMA_CHANNEL_UART0RX));
}


// Inicializa o módulo PWM
void InitPWM(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    PWMClockSet(PWM0_BASE, ui32SysClkFreq);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    uint32_t ui32Load = (ui32SysClkFreq / (2*freq_port));
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load);

    uint32_t duty_cycle = 25; // 25%
    uint32_t pulse_width = 0.25*ui32Load;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pulse_width);

    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

// Configura ADC para captura de dados
void ConfigureTimer(void) {
    // Habilita o periférico TIMER0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)){}

    // Desabilita o timer antes de configurar
    TimerDisable(TIMER0_BASE, TIMER_A);

    // Configura o timer no modo periódico
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Define o valor de recarga do timer
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32SysClkFreq / FS - 1);

    // Habilita o disparo por trigger
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

    // Habilita a interrupção do timer
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Habilita interrupções globais
    IntMasterEnable();

    // Inicia o timer
    TimerEnable(TIMER0_BASE, TIMER_A);
}


int main(void) {
    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                         SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                         SYSCTL_CFG_VCO_480), 120000000);

    load = ui32SysClkFreq / (2*freq_port);

    ConfigureUART();
    ConfigureUDMA();
    ConfigureTimer();
    InitPWM();

    while (1) {
        UART0_ReceiveFloatVector(dataVector, VECTOR_SIZE);
    }
}
