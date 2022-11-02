#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Device header file.
#include "stm32f1xx.h"

#define BAUD_RATE 1200

uint16_t ADC_value = 0;

uint32_t SystemCoreClock = 8000000;
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

void init() {
    // Copy initialized data from .sidata (Flash) to .data (RAM)
    memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
    // Clear the .bss section in RAM.
    memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );

    SCB->CPACR    |=  ( 0xF << 20 ); // Enable FPU

    RCC->APB2ENR = 0x0204; // Enable clock for GPIOA and ADC1
    RCC->APB1ENR = 0x20000; // Enable clock for USART2
    RCC->CFGR = 0x8000;

    GPIOA->CRL = 0x00004A00; // Configure PA1 as analog input

    USART2->CR1 = 0x200C;
    uint16_t uartdiv = SystemCoreClock / BAUD_RATE;
    USART2->BRR = ( ( ( uartdiv / 16 ) << USART_BRR_DIV_Mantissa_Pos ) |
                    ( ( uartdiv % 16 ) << USART_BRR_DIV_Fraction_Pos ) );
    
    ADC1->SMPR2 = 0x0038; // Set sample time for channel 1
    ADC1->SQR3 = 0x0001; // Set channel 1 as first conversion
    ADC1->CR2 = 0x0003; // Enable ADC

}

// delay in microseconds
void delay( uint32_t us ) {
  us *= ( SystemCoreClock / 1000000 ) / 5;
  while( us-- ) {
    __asm__( "NOP" );
  }
}

int main() {
    init();
    while (1) {
        ADC1->CR2 |= 0x40000000; // Start conversion
        while ( !( ADC1->SR & 0x0002 ) ); // Wait for conversion to complete
        ADC_value = ADC1->DR; // Read conversion result
        printf("ADC value: %d", ADC_value);
        printf("\r\n");
        delay(2000000);
    }
}