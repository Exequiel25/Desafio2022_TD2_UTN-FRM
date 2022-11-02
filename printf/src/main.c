// Standard library includes.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Device header file.
#include "stm32f1xx.h"

// ADC functions
void ADC_Init (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable ADC and GPIO clock
	2. Set the prescalar in the Common Control Register (CCR)
	3. Set the Scan Mode and Resolution in the Control Register 1 (CR1)
	4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	5. Set the Sampling Time for the channels in ADC_SMPRx
	6. Set the Regular channel sequence length in ADC_SQR1
	7. Set the Respective GPIO PINs in the Analog Mode
	************************************************/
	
//1. Enable ADC and GPIO clock
	RCC->APB2ENR |= 0x0204;  // enable ADC1 clock and GPIOA
	
//2. Set the prescalar in the Common Control Register (CCR)	
	RCC->CFGR |= 0x8000;  		 // PCLK2 divide by 2
	
//3. Set the Scan Mode and Resolution in the Control Register 1 (CR1)	
	ADC1->CR1 = (1<<8);    // SCAN mode enabled
	ADC1->CR1 &= ~(1<<24);   // 12 bit RES
	
//4. Set the Continuous Conversion, EOC, and Data Alignment in Control Reg 2 (CR2)
	// ADC1->CR2 |= (1<<1);     // enable continuous conversion mode
	// ADC1->CR1 |= (1<<5);    // EOC after each conversion
	// ADC1->CR2 &= ~(1<<11);   // Data Alignment RIGHT
  ADC1->CR2 = 0x00400000;
	
//5. Set the Sampling Time for the channels	
	ADC1->SMPR2 = 0x0038;  // Sampling time of 3 cycles for channel 1 and channel 4

//6. Set the Regular channel sequence length in ADC_SQR1
	ADC1->SQR3 = 0x0001;   // SQR1_L =1 for 2 conversions
	
//7. Set the Respective GPIO PINs in the Analog Mode	
	// GPIOA->MODER |= (3<<2);  // analog mode for PA 1 (chennel 1)
	// GPIOA->MODER |= (3<<8);  // analog mode for PA 4 (channel 4)
  GPIOA->CRL |= 0x00000000;  // analog mode for PA 1 (chennel 1)
}

void ADC_Enable (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Enable the ADC by setting ADON bit in CR2
	2. Wait for ADC to stabilize (approx 10us) 
	************************************************/
	ADC1->CR2 |= 0x0003;   // ADON =1 enable ADC1
	
	uint32_t delay = 10000;
	while (delay--);
}


void ADC_Start (int channel)
{
	/************** STEPS TO FOLLOW *****************
	1. Set the channel Sequence in the SQR Register
	2. Clear the Status register
	3. Start the Conversion by Setting the SWSTART bit in CR2
	************************************************/
	
	
/**	Since we will be polling for each channel, here we will keep one channel in the sequence at a time
		ADC1->SQR3 |= (channel<<0); will just keep the respective channel in the sequence for the conversion **/
	
	ADC1->SQR3 = 0;
	ADC1->SQR3 |= (channel<<0);    // conversion in regular sequence
	
	ADC1->SR = 0;        // clear the status register
	
	ADC1->CR2 |= 0x40000000;  // start the conversion
}


void ADC_WaitForConv (void)
{
	/*************************************************
	EOC Flag will be set, once the conversion is finished
	*************************************************/
	while (!(ADC1->SR & 2));  // wait for EOC flag to set
}

uint16_t ADC_GetVal (void)
{
	return ADC1->DR;  // Read the Data Register
}

void ADC_Disable (void)
{
	/************** STEPS TO FOLLOW *****************
	1. Disable the ADC by Clearing ADON bit in CR2
	************************************************/	
	ADC1->CR2 &= ~(1<<0);  // Disable ADC
}

// END OF ADC functions

#define BAUT_RATE 1200

uint16_t ADC_VAL = 0;

uint32_t SystemCoreClock = 0;
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

/**
 * Main program.
 */
int main( void ) {
  // Enable the GPIOA peripheral.
  ADC_Init();
  ADC_Enable();

  // Copy initialized data from .sidata (Flash) to .data (RAM)
  memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
  // Clear the .bss section in RAM.
  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );

  // Enable floating-point unit.
  SCB->CPACR    |=  ( 0xF << 20 );

  // Set the core system clock speed.
    // Default clock source is the 8MHz internal oscillator.
    SystemCoreClock = 8000000;

  // Enable peripheral clocks and set up GPIO pins.
    // Enable peripheral clocks: GPIOA, USART2.
    RCC->APB1ENR  |=  ( RCC_APB1ENR_USART2EN );
    RCC->APB2ENR  |=  ( RCC_APB2ENR_IOPAEN );
    // Configure pins A2, A3 for USART2.
    GPIOA->CRL    &= ~( GPIO_CRL_MODE2 |
                        GPIO_CRL_CNF2 |
                        GPIO_CRL_MODE3 |
                        GPIO_CRL_CNF3 );
    GPIOA->CRL    |= ( ( 0x1 << GPIO_CRL_MODE2_Pos ) |
                       ( 0x2 << GPIO_CRL_CNF2_Pos ) |
                       ( 0x0 << GPIO_CRL_MODE3_Pos ) |
                       ( 0x1 << GPIO_CRL_CNF3_Pos ) );

  // Set the baud rate to 1200.
  uint16_t uartdiv = SystemCoreClock / BAUT_RATE;
    USART2->BRR = ( ( ( uartdiv / 16 ) << USART_BRR_DIV_Mantissa_Pos ) |
                    ( ( uartdiv % 16 ) << USART_BRR_DIV_Fraction_Pos ) );

  // Enable the USART peripheral.
  USART2->CR1 |= ( USART_CR1_RE | USART_CR1_TE | USART_CR1_UE );

  // Main loop: wait for a new byte, then echo it back.
  char rxb = '\0';
  while ( 1 ) {
    ADC_Start(1);  // channel 1
    ADC_WaitForConv();
    ADC_VAL = ADC_GetVal();
    float tension = (ADC_VAL*3.3)/4095;
    //  while( !( USART2->SR & USART_SR_RXNE ) ) {};
    //  rxb = USART2->DR;
    //printf( "RX: %c\r\n", rxb);
    printf("V= %.2f\n", tension);
  }
}
