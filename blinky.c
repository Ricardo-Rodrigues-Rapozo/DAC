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
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "math.h"

#define BUFFER_SIZE 1000  // Tamanho do buffer de recepção
int i = 0;
uint32_t ui32PWMClockRate;
uint32_t freq_port = 2000;
uint32_t duty_cycle = 10;
uint32_t ui32ADC0Value[1]; // Armazena valores do ADC
uint32_t FS = 40000;
uint16_t buffer[BUFFER_SIZE]; // Buffer para armazenar os dados recebidos
uint16_t ADCbuffer[BUFFER_SIZE];
uint32_t ui32SysClkFreq;  // Frequência de clock do sistema
float offset = 50.0;    // Offset para centralizar entre 0% a 100%
uint32_t pulse_width;

//
//--------------------------------------- Inicialização do PWM ----------------------------------------------------------------------------------
//
void InitPWM(void)
{

    // HABILITA O MODULO PWM
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    //
    // HABILITA O GPIO
    //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // CONFIGURA OS PINOS
    //
    MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);
    MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Configurando o clk do pwm
    //
    MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);
    //
    // Use a local variable to store the PWM clock rate which will be
    // 120 MHz / 8 = 15 MHz. This variable will be used to set the
    // PWM generator period.
    //
    ui32PWMClockRate = ui32SysClkFreq / 8;

    //
    // Configure PWM2 to count up/down without synchronization.
    //
    MAP_PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                        PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    //
    // Set the PWM period to 250Hz.  To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * PWMClk.  Where N is the
    // function parameter, f is the desired frequency, and PWMClk is the
    // PWM clock frequency based on the system clock.
    MAP_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, (ui32PWMClockRate / freq_port));

    //
    // Set PWM2 to a duty cycle of 25%.  You set the duty cycle as a function
    // of the period.  Since the period was set above, you can use the
    // PWMGenPeriodGet() function.  For this example the PWM will be high for
    // 25% of the time or (PWM Period / 4).
    //Configurar frequência do
    //gerador e razão cíclica inicial
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                         MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) / 4);

    //
    // Enable PWM Out Bit 2 (PF2) output signal.
    //
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);

    //
    // Enable the PWM generator block.
    //
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1);

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
void UART0_ReceiveData(void)
{
    static uint32_t index = 0;

    while (UARTCharsAvail(UART0_BASE))
    {
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
            buffer[index] = ui32ADC0Value[0];

            // Atualiza o duty cycle do PWM com base nos valores do buffer
            //uint32_t dutyCycle = (buffer[index] * 100) / 65535; // Converte para porcentagem de 0 a 100

            // Atribui o valor do ADC ao buffer ADC
            ADCbuffer[index] = ui32ADC0Value[0];
            float duty_cycle =  (uint32_t)(value/100);
            // Verificação para manter os limites do duty cycle
            if (duty_cycle > 100.0)
            {
                duty_cycle = 100.0;
            }
            if (duty_cycle < 0.0) {
                duty_cycle = 0.0;
            }
            // Converter o valor do ciclo de trabalho para a largura de pulso do PWM
            pulse_width = (duty_cycle  * PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1));
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pulse_width);
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
    InitPWM();  // Inicia com 25% de duty cycle (ajuste conforme necessário)
    ConfigureADC(); // inicializa o ADC
    while (1) {
            // Recebe dados pela UART e armazena no buffer
            UART0_ReceiveData();
               }

}
