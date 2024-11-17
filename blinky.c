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



#define VECTOR_SIZE 100

// Configuração da Tabela de controle do uDMA
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] _attribute_ ((aligned(1024)));
#endif

uint32_t control1 = 0;
uint32_t ui32SysClkFreq;
uint32_t idx = 0;
uint32_t buffer[VECTOR_SIZE];
float dataVector[VECTOR_SIZE];
uint32_t FS = 40000;
uint32_t ui32PWMClockRate;
uint32_t freq_port = 10000;
uint32_t duty_cycle = 0 ;
uint32_t ui32ADC0Value[1];
volatile uint32_t flag = 0;

void Timer0IntHandler(void)
{
// Clear the timer interrupt
TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
// Read the current state of the GPIO pin and
// write back the opposite state
flag = 1;

}

void ConfigureUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, ui32SysClkFreq, 115200,(UART_CONFIG_WLEN_8 |
            UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    UARTFIFOLevelSet(UART0_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);

    UARTEnable(UART0_BASE);

}

void ConfigureUDMA(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA)){}

    uDMAEnable();
    uDMAControlBaseSet(pui8ControlTable); //determina o endereço de memória para o canal de controle

    //Habilita o uDMA para a UART
    UARTDMAEnable(UART0_BASE, UART_DMA_TX | UART_DMA_RX);

    //Determinar os canais uDMA respectivos para o UART, conferir data sheet
    uDMAChannelAssign(UDMA_CH8_UART0RX); //canal 8 associado ao UART 0 RX
    // Configuração para UART RX (recepção)
    uDMAChannelAttributeDisable(UDMA_CHANNEL_UART0RX, UDMA_ATTR_ALL);
    uDMAChannelControlSet(UDMA_CHANNEL_UART0RX | UDMA_PRI_SELECT, UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 | UDMA_ARB_4);


    uDMAChannelAssign(UDMA_CH9_UART0TX); //canal 9 associado ao UART 0 TX

}


void UART0_ReceiveFloatVector(float* vector, uint32_t size)
{

    while(!UARTCharsAvail(UART0_BASE));

    uDMAChannelTransferSet(UDMA_CHANNEL_UART0RX | UDMA_PRI_SELECT,
                           UDMA_MODE_BASIC,
                           (void *)(UART0_BASE + UART_O_DR),
                           (uint8_t*)vector,
                           size * sizeof(float));

    uDMAChannelEnable(UDMA_CHANNEL_UART0RX);

    while(uDMAChannelIsEnabled(UDMA_CHANNEL_UART0RX));


}

void InitPWM(void)
{

    //
//    // hABILITA O MODULO PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    //
    //hABILITA O GPIO
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));


    //
    // CONFIGURA o pino f2
    //
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // configurando o clk do pwm
    //
//    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);
    //
    // Use a local variable to store the PWM clock rate which will be
    // 120 MHz / 8 = 15 MHz. This variable will be used to set the
    // PWM generator period.
    //
    ui32PWMClockRate = ui32SysClkFreq / freq_port;

    //
    // Configure PWM2 to count up/down without synchronization.
    //
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                        PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    //
    // Set the PWM period to 250Hz.  To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * PWMClk.  Where N is the
    // function parameter, f is the desired frequency, and PWMClk is the
    // PWM clock frequency based on the system clock.
    //
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32PWMClockRate);

    //
    // Set PWM2 to a duty cycle of 25%.  You set the duty cycle as a function
    // of the period.  Since the period was set above, you can use the
    // PWMGenPeriodGet() function.  For this example the PWM will be high for
    // 25% of the time or (PWM Period / 4).
    //Configurar frequência do
    //gerador e razão cíclica inicial
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,
                         MAP_PWMGenPeriodGet(PWM0_BASE, PWM_GEN_1) / 4);

    //
    // Enable PWM Out Bit 2 (PF2) output signal.
    //
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);

    //
    // Enable the PWM generator block.
    //
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

}


void setPWM(float* vector, uint32_t size)
{

    control1 = 8;
    if (ADCIntStatus(ADC0_BASE, 1, false))
    {
        // Exemplo de processamento: imprimir os valores (ou qualquer outra operação)
        if(flag == 1)
        {
            //control1 = 2;
            ADCIntClear(ADC0_BASE, 1);
            ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
            buffer[idx] =  ui32ADC0Value[0];  // Mantém apenas os 16 bits menos significativos
            //duty_cycle = dataVector[idx];
            // Converter o valor do ciclo de trabalho para a largura de pulso do PWM
            uint32_t pulse_width = (round((dataVector[idx] - 0.75)/200  * ui32PWMClockRate ));
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pulse_width);
            idx = (idx + 1)% VECTOR_SIZE;// idx = (idx+1)%VECTOR_SIZE
            flag = 0;
        }
    }//    ui32PWMClockRate = ui32SysClkFreq / freq_port;

}




void ConfigureADC(void)
{
    // ------------------ Timer ---------------------------------------------------------

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32SysClkFreq / FS - 1);

    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);



    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntMasterEnable();

    TimerEnable(TIMER0_BASE, TIMER_A);

    // ------------------------- ADC ---------------------------------------
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
        // Clk do sistema
        ui32SysClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
    SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

        ConfigureUART();

        ConfigureUDMA();

        ConfigureADC();

        InitPWM();

        while(1)
        {
            // Recebe o vetor do python
            UART0_ReceiveFloatVector(dataVector, VECTOR_SIZE);
            // Quando o vetor estiver cheio, processa os valores usando um loop `for`
            setPWM(dataVector, VECTOR_SIZE);
        }


}
