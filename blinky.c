#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c1294ncpdt.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "inc/hw_uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/fpu.h"

#define BUFFER_SIZE 1000  // Tamanho do buffer de recepção
int i = 0;
uint32_t ui32ADC0Value[1]; // Armazena valores do ADC
uint32_t FS = 40000;
uint16_t buffer[BUFFER_SIZE]; // Buffer para armazenar os dados recebidos
uint16_t ADCbuffer[BUFFER_SIZE];
uint32_t ui32SysClkFreq;  // Frequência de clock do sistema

// Inicialização do PWM com duty cycle configurável
void InitPWM(uint32_t dutyCycle) {
    // Habilita o módulo PWM0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) {}

    // Habilita o GPIOG (para PG0)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG)) {}

    // Configura o pino PG0 para saída de PWM
    GPIOPinConfigure(GPIO_PG0_M0PWM4);  // Configura o PG0 para PWM0 canal 0 (não canal 4)
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);  // Definindo PG0 como saída PWM
    // Configura o gerador de PWM para o modo de contagem decrescente com atualizações imediatas dos parâmetros
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Define o período do PWM (frequência da portadora)
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 400);

    // Define a largura do pulso de acordo com o duty cycle recebido
    uint32_t pulseWidth = (dutyCycle * PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0)) / 100;
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pulseWidth);

    // Inicia os temporizadores no gerador 0
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    // Habilita as saídas
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
}

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

// Função para receber dados pela UART
void UART0_ReceiveData(void) {
    static uint32_t index = 0;

    while (UARTCharsAvail(UART0_BASE)) {
        // Lê um byte recebido
        uint8_t byte1 = UARTCharGet(UART0_BASE);
        uint8_t byte2 = UARTCharGet(UART0_BASE);

        // Combina os dois bytes em um valor uint16_t
        uint16_t value = ((uint16_t)byte2 << 8) | byte1;
        // Espera até a conversão do ADC estar completa
        if (ADCIntStatus(ADC0_BASE, 1, false))
        {
            //Limpa a flag de interrupção do ADC e obtém os dados do sequenciador
            ADCIntClear(ADC0_BASE, 1);
            ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
            // Armazena o valor no buffer
            buffer[index] = value;

            // Atualiza o duty cycle do PWM com base nos valores do buffer
            uint32_t dutyCycle = (buffer[index] * 100) / 65535; // Converte para porcentagem de 0 a 100
            InitPWM(dutyCycle); // Atualiza o PWM

            // Atribui o valor do ADC ao buffer ADC
            ADCbuffer[index] = ui32ADC0Value[0];

            // Incrementa o índice e faz a rotação para o início se necessário
            index = (index + 1) % BUFFER_SIZE;
        }
    }
}




void ConfigureADC(void)
{

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32SysClkFreq/FS - 1);

    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

    TimerEnable(TIMER0_BASE, TIMER_A);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0);

    ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

    ADCSequenceEnable(ADC0_BASE, 1);

    ADCIntClear(ADC0_BASE, 1);


}

int main(void)
{
    // Configuração do clock do sistema
    ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                         SYSCTL_CFG_VCO_480), 120000000);

    // Inicialização da UART
    UART0_Init();
    // Inicializa o PWM com um duty cycle inicial (ajustável posteriormente pelo buffer)
    InitPWM(25);  // Inicia com 25% de duty cycle (ajuste conforme necessário)
    ConfigureADC(); // inicializa o ADC
    while (1) {
            // Recebe dados pela UART e armazena no buffer
            UART0_ReceiveData();
               }

}
