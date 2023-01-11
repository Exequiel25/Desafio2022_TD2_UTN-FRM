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
    uint32_t SystemCoreClock = 8000000;
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
  uint16_t BaudRate = 1200;
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
// init GPIOA and GPIOB for using a 7 segment display
void init_gpio()
{
  // Enable peripheral clocks: GPIOA, GPIOB.
  #ifdef VVC_F1
    RCC->APB2ENR |= ( RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN );
  #elif VVC_L4
    RCC->AHB2ENR |= ( RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN );
  #endif

  // Configure pins A9-A15
  #ifdef VVC_F1
    GPIOA->CRH &= ~( GPIO_CRH_MODE9 | GPIO_CRH_CNF9 |
                     GPIO_CRH_MODE10 | GPIO_CRH_CNF10 |
                     GPIO_CRH_MODE11 | GPIO_CRH_CNF11 |
                     GPIO_CRH_MODE12 | GPIO_CRH_CNF12 |
                     GPIO_CRH_MODE13 | GPIO_CRH_CNF13 |
                     GPIO_CRH_MODE14 | GPIO_CRH_CNF14 |
                     GPIO_CRH_MODE15 | GPIO_CRH_CNF15 );
    GPIOA->CRH |=  ( 0x3 << GPIO_CRH_MODE9_Pos | 0x0 << GPIO_CRH_CNF9_Pos |
                     0x3 << GPIO_CRH_MODE10_Pos | 0x0 << GPIO_CRH_CNF10_Pos |
                     0x3 << GPIO_CRH_MODE11_Pos | 0x0 << GPIO_CRH_CNF11_Pos |
                     0x3 << GPIO_CRH_MODE12_Pos | 0x0 << GPIO_CRH_CNF12_Pos |
                     0x3 << GPIO_CRH_MODE13_Pos | 0x0 << GPIO_CRH_CNF13_Pos |
                     0x3 << GPIO_CRH_MODE14_Pos | 0x0 << GPIO_CRH_CNF14_Pos |
                     0x3 << GPIO_CRH_MODE15_Pos | 0x0 << GPIO_CRH_CNF15_Pos );
  #elif VVC_L4
    GPIOA->MODER &= ~( 0x3 << ( 9 * 2 ) | 0x3 << ( 10 * 2 ) |
                       0x3 << ( 11 * 2 ) | 0x3 << ( 12 * 2 ) |
                       0x3 << ( 13 * 2 ) | 0x3 << ( 14 * 2 ) |
                       0x3 << ( 15 * 2 ) );
    GPIOA->MODER |=  ( 0x1 << ( 9 * 2 ) | 0x1 << ( 10 * 2 ) |
                       0x1 << ( 11 * 2 ) | 0x1 << ( 12 * 2 ) |
                       0x1 << ( 13 * 2 ) | 0x1 << ( 14 * 2 ) |
                       0x1 << ( 15 * 2 ) );
  #endif
  
  // Configure pins B0-B7 for output.
  #ifdef VVC_F1
    GPIOB->CRL &= ~( GPIO_CRL_MODE0 | GPIO_CRL_CNF0 |
                     GPIO_CRL_MODE1 | GPIO_CRL_CNF1 |
                     GPIO_CRL_MODE2 | GPIO_CRL_CNF2 |
                     GPIO_CRL_MODE3 | GPIO_CRL_CNF3 |
                     GPIO_CRL_MODE4 | GPIO_CRL_CNF4 |
                     GPIO_CRL_MODE5 | GPIO_CRL_CNF5 |
                     GPIO_CRL_MODE6 | GPIO_CRL_CNF6 |
                     GPIO_CRL_MODE7 | GPIO_CRL_CNF7 );
    GPIOB->CRL |=  ( 0x3 << GPIO_CRL_MODE0_Pos | 0x0 << GPIO_CRL_CNF0_Pos |
                     0x3 << GPIO_CRL_MODE1_Pos | 0x0 << GPIO_CRL_CNF1_Pos |
                     0x3 << GPIO_CRL_MODE2_Pos | 0x0 << GPIO_CRL_CNF2_Pos |
                     0x3 << GPIO_CRL_MODE3_Pos | 0x0 << GPIO_CRL_CNF3_Pos |
                     0x3 << GPIO_CRL_MODE4_Pos | 0x0 << GPIO_CRL_CNF4_Pos |
                     0x3 << GPIO_CRL_MODE5_Pos | 0x0 << GPIO_CRL_CNF5_Pos |
                     0x3 << GPIO_CRL_MODE6_Pos | 0x0 << GPIO_CRL_CNF6_Pos |
                     0x3 << GPIO_CRL_MODE7_Pos | 0x0 << GPIO_CRL_CNF7_Pos );
  #elif VVC_L4
    GPIOB->MODER &= ~( 0x3 << ( 0 * 2 ) | 0x3 << ( 1 * 2 ) |
                       0x3 << ( 2 * 2 ) | 0x3 << ( 3 * 2 ) |
                       0x3 << ( 4 * 2 ) | 0x3 << ( 5 * 2 ) |
                       0x3 << ( 6 * 2 ) | 0x3 << ( 7 * 2 ) );
    GPIOB->MODER |=  ( 0x1 << ( 0 * 2 ) | 0x1 << ( 1 * 2 ) |
                       0x1 << ( 2 * 2 ) | 0x1 << ( 3 * 2 ) |
                       0x1 << ( 4 * 2 ) | 0x1 << ( 5 * 2 ) |
                       0x1 << ( 6 * 2 ) | 0x1 << ( 7 * 2 ) );
  #endif
  
  // Configure pins B8-B15 for output.
  #ifdef VVC_F1
    GPIOB->CRH &= ~( GPIO_CRH_MODE8 | GPIO_CRH_CNF8 |
                     GPIO_CRH_MODE9 | GPIO_CRH_CNF9 |
                     GPIO_CRH_MODE10 | GPIO_CRH_CNF10 |
                     GPIO_CRH_MODE11 | GPIO_CRH_CNF11 |
                     GPIO_CRH_MODE12 | GPIO_CRH_CNF12 |
                     GPIO_CRH_MODE13 | GPIO_CRH_CNF13 |
                     GPIO_CRH_MODE14 | GPIO_CRH_CNF14 |
                     GPIO_CRH_MODE15 | GPIO_CRH_CNF15 );
    GPIOB->CRH |=  ( 0x3 << GPIO_CRH_MODE8_Pos | 0x0 << GPIO_CRH_CNF8_Pos |
                     0x3 << GPIO_CRH_MODE9_Pos | 0x0 << GPIO_CRH_CNF9_Pos |
                     0x3 << GPIO_CRH_MODE10_Pos | 0x0 << GPIO_CRH_CNF10_Pos |
                     0x3 << GPIO_CRH_MODE11_Pos | 0x0 << GPIO_CRH_CNF11_Pos |
                     0x3 << GPIO_CRH_MODE12_Pos | 0x0 << GPIO_CRH_CNF12_Pos |
                     0x3 << GPIO_CRH_MODE13_Pos | 0x0 << GPIO_CRH_CNF13_Pos |
                     0x3 << GPIO_CRH_MODE14_Pos | 0x0 << GPIO_CRH_CNF14_Pos |
                     0x3 << GPIO_CRH_MODE15_Pos | 0x0 << GPIO_CRH_CNF15_Pos );
  #elif VVC_L4
    GPIOB->MODER &= ~( 0x3 << ( 8 * 2 ) | 0x3 << ( 9 * 2 ) |
                       0x3 << ( 10 * 2 ) | 0x3 << ( 11 * 2 ) |
                       0x3 << ( 12 * 2 ) | 0x3 << ( 13 * 2 ) |
                       0x3 << ( 14 * 2 ) | 0x3 << ( 15 * 2 ) );
    GPIOB->MODER |=  ( 0x1 << ( 8 * 2 ) | 0x1 << ( 9 * 2 ) |
                       0x1 << ( 10 * 2 ) | 0x1 << ( 11 * 2 ) |
                       0x1 << ( 12 * 2 ) | 0x1 << ( 13 * 2 ) |
                       0x1 << ( 14 * 2 ) | 0x1 << ( 15 * 2 ) );
  #endif
}
/* -------------------------------------------------------------------------- */
// Function to convert int value to 7-segment display value
uint16_t int2seg7( uint8_t value )
{
  uint16_t seg7 = 0;
  switch( value )
  {
    case 0:  seg7 = 0x3F; break;
    case 1:  seg7 = 0x06; break;
    case 2:  seg7 = 0x5B; break;
    case 3:  seg7 = 0x4F; break;
    case 4:  seg7 = 0x66; break;
    case 5:  seg7 = 0x6D; break;
    case 6:  seg7 = 0x7D; break;
    case 7:  seg7 = 0x07; break;
    case 8:  seg7 = 0x7F; break;
    case 9:  seg7 = 0x6F; break;
    default: seg7 = 0x00; break;
  }
  return seg7;
}

/* -------------------------------------------------------------------------- */
// main function
int main()
{
  // Initialize the USART.
  init_usart();

  // Initialize the ADC.
  init_adc();

  // Initialize the GPIO.
  init_gpio();

  delay(200);
  // printf( "\r\nDesafio 2022\r\n");

  // Loop forever.
  while ( 1 )
  {
    // Read the ADC.
    uint16_t adc = read_adc();

    // Send the ADC value over the USART.
    printf( "ADC:%u\r\n",adc);

    // Calculate the ADC value in volts.
    float volts = (float)adc * 3.3 / 4096.0;

    // Calculate the ADC value in volts

    // Send the ADC value in volts over the USART.
    // converting each digit to a character making a total of 3 digits.
    // printf( "V:%u.%u%u\r\n",(uint8_t)volts,(uint8_t)(volts*10)%10,(uint8_t)(volts*100)%10);
    
    // Write digits to equivalent 7-segment display pins.
    GPIOA->ODR = int2seg7( (uint8_t)volts ) << 9;
    GPIOB->ODR = int2seg7( (uint8_t)(volts*10)%10 ) << 0;
    GPIOB->ODR |= int2seg7( (uint8_t)(volts*100)%10 ) << 7;
    
    // delay 2 seconds
    delay(2000);
  }
}