// Standard library includes.
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Device header file.
#ifdef VVC_F1
  #include "stm32f1xx.h"
#elif VVC_L4
  #include "stm32l4xx.h"
#endif

uint32_t BaudRate = 1200;
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
    #ifdef VVC_F1
      while( !( USART2->SR & USART_SR_TXE ) ) {};
      USART2->DR = *data++;
    #elif VVC_L4
      while( !( USART2->ISR & USART_ISR_TXE ) ) {};
      USART2->TDR = *data++;
    #endif
  }
  return size;
}
/*--------------------------------------------------------------------------*/
// delay function (in milliseconds)
void delay( uint32_t ms ) {
  uint32_t i;
  for( i = 0; i < ms; i++ ) {
    uint32_t j;
    for( j = 0; j < 464; j++ ) {
      __asm__( "NOP" );
    }
  }
}
/*--------------------------------------------------------------------------*/
void init_usart()
{
// Copy initialized data from .sidata (Flash) to .data (RAM)
  memcpy( &_sdata, &_sidata, ( ( void* )&_edata - ( void* )&_sdata ) );
  // Clear the .bss section in RAM.
  memset( &_sbss, 0x00, ( ( void* )&_ebss - ( void* )&_sbss ) );

  // Enable floating-point unit.
  SCB->CPACR    |=  ( 0xF << 20 );

  // Set the core system clock speed.
  #ifdef VVC_F1
    // Default clock source is the 8MHz internal oscillator.
    SystemCoreClock = 8000000;
  #elif VVC_L4
    // Default clock source is the "multi-speed" internal oscillator.
    // Switch to the 16MHz HSI oscillator.
    RCC->CR |=  ( RCC_CR_HSION );
    while ( !( RCC->CR & RCC_CR_HSIRDY ) ) {};
    RCC->CFGR &= ~( RCC_CFGR_SW );
    RCC->CFGR |=  ( RCC_CFGR_SW_HSI );
    while ( ( RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_HSI ) {};
    SystemCoreClock = 16000000;
  #endif

  // Enable peripheral clocks and set up GPIO pins.
  #ifdef VVC_F1
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
  #elif VVC_L4
    // Enable peripheral clocks: GPIOA, USART2.
    RCC->APB1ENR1 |= ( RCC_APB1ENR1_USART2EN );
    RCC->AHB2ENR  |= ( RCC_AHB2ENR_GPIOAEN );
    // Configure pins A2, A15 for USART2 (AF7).
    GPIOA->MODER    &= ~( ( 0x3 << ( 2 * 2 ) ) |
                          ( 0x3 << ( 15 * 2 ) ) );
    GPIOA->MODER    |=  ( ( 0x2 << ( 2 * 2 ) ) |
                          ( 0x2 << ( 15 * 2 ) ) );
    GPIOA->OTYPER   &= ~( ( 0x1 << 2 ) |
                          ( 0x1 << 15 ) );
    GPIOA->OSPEEDR  &= ~( ( 0x3 << ( 2 * 2 ) ) |
                          ( 0x3 << ( 15 * 2 ) ) );
    GPIOA->OSPEEDR  |=  ( ( 0x2 << ( 2 * 2 ) ) |
                          ( 0x2 << ( 15 * 2 ) ) );
    GPIOA->AFR[ 0 ] &= ~( ( 0xF << ( 2 * 4 ) ) );
    GPIOA->AFR[ 0 ] |=  ( ( 0x7 << ( 2 * 4 ) ) );
    GPIOA->AFR[ 1 ] &= ~( ( 0xF << ( ( 15 - 8 ) * 4 ) ) );
    GPIOA->AFR[ 1 ] |=  ( ( 0x3 << ( ( 15 - 8 ) * 4 ) ) );
  #endif

  // Set the baud rate.
  uint16_t uartdiv = SystemCoreClock / BaudRate;
  #ifdef VVC_F1
    USART2->BRR = ( ( ( uartdiv / 16 ) << USART_BRR_DIV_Mantissa_Pos ) |
                    ( ( uartdiv % 16 ) << USART_BRR_DIV_Fraction_Pos ) );
  #elif VVC_L4
    USART2->BRR = uartdiv;
  #endif

  // Enable the USART peripheral.
  USART2->CR1 |= ( USART_CR1_RE | USART_CR1_TE | USART_CR1_UE );
}
/*--------------------------------------------------------------------------*/
// adc init function
void init_adc()
{
  // Enable peripheral clocks: GPIOA, ADC1.
  #ifdef VVC_F1
    RCC->APB2ENR |= ( RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN );
  #elif VVC_L4
    RCC->AHB2ENR |= ( RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_ADCEN );
  #endif

  // Configure pin A0 for ADC1.
  #ifdef VVC_F1
    GPIOA->CRL &= ~( GPIO_CRL_MODE0 | GPIO_CRL_CNF0 );
    GPIOA->CRL |=  ( 0x0 << GPIO_CRL_MODE0_Pos | 0x0 << GPIO_CRL_CNF0_Pos );
  #elif VVC_L4
    GPIOA->MODER &= ~( 0x3 << ( 0 * 2 ) );
    GPIOA->MODER |=  ( 0x3 << ( 0 * 2 ) );
  #endif

  // Set the ADC clock prescaler.
  #ifdef VVC_F1
    RCC->CFGR &= ~( RCC_CFGR_ADCPRE );
    RCC->CFGR |=  ( 0x0 << RCC_CFGR_ADCPRE_Pos );
  #elif VVC_L4
    RCC->CCIPR &= ~( RCC_CCIPR_ADCSEL );
    RCC->CCIPR |=  ( 0x0 << RCC_CCIPR_ADCSEL_Pos );
  #endif

  // Set the ADC sample time.
  #ifdef VVC_F1
    ADC1->SMPR2 &= ~( ADC_SMPR2_SMP0 );
    ADC1->SMPR2 |=  ( 0x7 << ADC_SMPR2_SMP0_Pos );
  #elif VVC_L4
    ADC->SMPR &= ~( ADC_SMPR_SMP_0 );
    ADC->S
[...]

MPR |=  ( 0x7 << ADC_SMPR_SMP_0_Pos );
  #endif

  // Enable the ADC peripheral.
  #ifdef VVC_F1
    ADC1->CR2 |= ( ADC_CR2_ADON );
  #elif VVC_L4
    ADC->CR |= ( ADC_CR_ADEN );
    while ( ( ADC->ISR & ADC_ISR_ADRDY ) == 0 ) {};
  #endif
}
/*--------------------------------------------------------------------------*/
// adc read function
uint16_t read_adc()
{
  // Start the ADC conversion.
  #ifdef VVC_F1
    ADC1->CR2 |= ( ADC_CR2_SWSTART );
  #elif VVC_L4
    ADC->CR |= ( ADC_CR_ADSTART );
  #endif

  // Wait for the ADC conversion to complete.
  #ifdef VVC_F1
    while ( ( ADC1->SR & ADC_SR_EOC ) == 0 ) {};
  #elif VVC_L4
    while ( ( ADC->ISR & ADC_ISR_EOC ) == 0 ) {};
  #endif

  // Read the ADC conversion result.
  #ifdef VVC_F1
    return ADC1->DR;
  #elif VVC_L4
    return ADC->DR;
  #endif
}
/*--------------------------------------------------------------------------*/
// main function
int main()
{
  // Initialize the USART.
  init_usart();

  // Initialize the ADC.
  init_adc();

  // Delay for a bit
  delay( 200 );
  printf( "\r\nDesafio 2022\r\n");

  // Loop forever.
  while ( 1 )
  {
    // Read the ADC.
    uint16_t adc = read_adc();

    // Send the ADC value over the USART.
    printf( "ADC:%u\r\n",adc);

    // Calculate the ADC voltage.
    float voltage = ( ( float ) adc / 4095.0 ) * 3.3;
    // Convert float to int
    int voltage_int = (int) (voltage);
    int voltage_dec = (int) ((voltage - voltage_int) * 1000);
    
    // Send the ADC voltage over the USART.
    printf( "V:%d.%d\r\n",voltage_int,voltage_dec);

    // delay 2 seconds
    delay( 2000 );
    
  }
}
