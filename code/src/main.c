#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Device header file.
#include "stm32f1xx.h"

#define BAUD_RATE 1200
uint32_t SystemCoreClock = 8000000; // as default 8MHz is used
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

// Reset handler: set the stack pointer and branch to main().
__attribute__( ( naked ) ) void reset_handler( void ) {
  // Set the stack pointer to the 'end of stack' value.
  __asm__( "LDR r0, =_estack\n\t"
           "MOV sp, r0" );
  // Branch to main().
  __asm__( "B main" );
}

// Override the 'write' clib method to implement 'printf' over UART.
int _write( int handle, char* data, int size ) {
  int count = size;
  while( count-- ) {
    while( !( USART2->SR & USART_SR_TXE ) ) {};
    USART2->DR = *data++;
  }
  return size;
}

/*--------------------------------------------------------------------------*/
// INIT ADC and USART functions
void init_adc(void)
{
    // ADC clock must be < 14MHz; Config prescale 8MHz / 2 = 4Mhz
    RCC->CFGR       |= RCC_CFGR_ADCPRE_DIV4;
    
    RCC->APB2ENR    |= RCC_APB2ENR_ADC1EN;  // enable ADC clock
    RCC->APB2ENR    |= RCC_APB2ENR_IOPAEN;  // enable GPIOA clock
    
    // reset MODE and CNF bits to 0000; MODE = input : CNF = analog mode
    GPIOA->CRL &= ~(GPIO_CRL_CNF0  | GPIO_CRL_MODE0);   // reset PA0
    
    // set sample time for ch 0 to 28.5 cycles (0b011)
    ADC1->SMPR2 |= ( ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_0);
    
    // ADC1->SQR1 L[3:0] = 0b0000 at reset; set for 1 conversion
	ADC1->SQR1 = 0x00000000;
    // ADC1->SQR3 SQ1[4:0] = 0b00000 at reset; 1st conversion = chan 0
    ADC1->SQR3 = 0x00000000;
    
    // put ADC1 into continuous mode and turn on ADC
    ADC1->CR2 |= (ADC_CR2_CONT | ADC_CR2_ADON);
    
    // reset calibration registers
    ADC1->CR2 |= (ADC_CR2_RSTCAL);
    
    // wait for calibration register initalized
    while(ADC1->CR2 & ADC_CR2_RSTCAL);
    
    // enable calibration
    ADC1->CR2 |= (ADC_CR2_CAL);
    
    // wait for calibration completed
    while(ADC1->CR2 & ADC_CR2_CAL);
    
    // not concerned about power consumption, just start the continuous
    // conversions and will read the DR periodically
    
    // start conversions
    ADC1->CR2 |= ADC_CR2_ADON;
}

void init_usart()
{
    // Copy initialized data from .sidata (Flash) to .sdata (RAM)
    memcpy( &_sdata, &_sidata, ( ( uint32_t ) &_edata - ( uint32_t ) &_sdata ) );
    // Clear the .sbss section (RAM)
    memset( &_sbss, 0, ( ( uint32_t ) &_ebss - ( uint32_t ) &_sbss ) );

    // Enable floating-pont unit.
    SCB->CPACR |= ( ( 3UL << 10 * 2 ) | ( 3UL << 11 * 2 ) );

    // Enable peripheral clocks and set up GPIO pins.
    // Enable GPIOA clock.
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    // Enable USART2 clock.
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configure PA2 as alternate function push-pull.
    GPIOA->CRL &= ~( GPIO_CRL_CNF2 | GPIO_CRL_MODE2 );
    GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1;

    // Configure PA3 as input floating.
    GPIOA->CRL &= ~( GPIO_CRL_CNF3 | GPIO_CRL_MODE3 );

    // Configure USART2.
    // Set baud rate.
    USART2->BRR = SystemCoreClock / BAUD_RATE;

    // Enable USART2.
    USART2->CR1 |= USART_CR1_UE;
    // Enable transmitter.
    USART2->CR1 |= USART_CR1_TE;
    // Enable receiver.
    USART2->CR1 |= USART_CR1_RE;
}
/*--------------------------------------------------------------------------*/
// delay function
void delay( uint32_t us ) {
  us *= ( SystemCoreClock / 1000000 ) / 5;
  while( us-- ) {
    __asm__( "NOP" );
  }
}
/*--------------------------------------------------------------------------*/
// MAIN function
int main( void ) {
  // Initialize ADC and USART
  init_usart();
  init_adc();
  
  // Print a message.
  printf( "Desafio 2022\r\n" );

    // Loop forever.
    while( 1 ) {
        // read ADC1 channel 0
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while(!(ADC1->SR & ADC_SR_EOC));
        uint16_t adc_value = ADC1->DR;
        printf("ADC1: %d; ", adc_value);
        
        float voltage = (float)adc_value * 3.3 / 4096;
        int voltage_int = (int)voltage;
        int voltage_dec = (int)((voltage - voltage_int) * 1000);
        printf("Voltage: %d.%d\r\n", voltage_int, voltage_dec);
        delay(2000000);
    }
}
