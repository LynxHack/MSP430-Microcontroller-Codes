//ADC - DC Motor Throttle (Potentiometer)

#include <msp430.h>

#define BUFFER_SIZE  50

volatile unsigned int top = 0;  //ind = (ind + 1) % 50;
volatile unsigned int end = 0;
volatile unsigned int size = 0;
volatile unsigned char rx_byte;
int buffer[BUFFER_SIZE];
int writeflag = 0;
volatile unsigned int watch = 0;

void DriveMotor(int dutycycle, int direction){  //DutyCycle is 16 bit number range
    if(direction == 0){
        TB0CCTL1 = OUTMOD_2;                      // Set/Reset Mode
        TB0CCR1 = dutycycle;
        TB0CCR2 = 0;
        TB0CTL = TBSSEL_1 + MC_2;                  // ACLK, continuous mode
    }
    if(direction == 1){                   // Set/Reset Mode
        TB0CCTL2 = OUTMOD_2;                      // Set/Reset Mode
        TB0CCR2 = dutycycle;
        TB0CCR1 = 0;
        TB0CTL = TBSSEL_1 + MC_2;                  // ACLK, continuous mode
    }
}

#pragma vector = USCI_A1_VECTOR             // USCI ISR
__interrupt void USCI_A1_ISR(void)
{
//    switch(__even_in_range(UCA1IV,0x08))
//    {
//    case 0:break;                             // Vector 0 - no interrupt
//    case 2:                                   // Vector 2 - RXIFG
      while (!(UCA1IFG&UCTXIFG));             // USCI_A0 TX buffer ready?

      if(end==50){
          end = 0;

      }
      buffer[end] = UCA1RXBUF; // grab byte from RX buffer
//      else{
          end = end + 1;
//      }
      size = size + 1;
//      UCA1TXBUF = UCA1RXBUF;                  // TX -> RXed character

//      break;
//    case 4:break;                             // Vector 4 - TXIFG
//    default: break;
//    }
}


volatile unsigned int previousvalue;
volatile unsigned int result;
volatile unsigned int firstcheck = 0;
int main(void)
{
    unsigned int Packet[5];
    unsigned int i;
    unsigned int sixteenBitNumber;
    WDTCTL = WDTPW + WDTHOLD;                 // stop watchdog

  // XT1 Setup
  PJSEL0 |= BIT4 + BIT5;

  //Configure Motor Driver Pins
  P1SEL1 &= ~BIT5 + ~BIT4;
  P1SEL0 |= BIT5 + BIT4;
  P1DIR |= BIT5 + BIT4;

  CSCTL0_H = 0xA5; //CSKEY - 0xA5 to enable write
  CSCTL1 |= DCORSEL + DCOFSEL_3;//DCOFSEL_2;//Now set to 16MHz //DCOFSEL_3;             // Set max. DCO setting, set to 24MHz
  CSCTL2 = SELA_3 + SELS_3 + SELM_3;        // set ACLK = DCO; MCLK = DCO
  CSCTL3 = DIVA_1 + DIVS_0 + DIVM_0;        // set all dividers to unity
  CSCTL4 = XT2OFF + XT1OFF;

  P2SEL1 = BIT5+BIT6; //enables UART TX and RX pins
  P2SEL0 &= ~(BIT5+BIT6);

  UCA1CTL1 |= UCSWRST; //enabl1es software reset
  UCA1CTL1 = UCSSEL_1;       //sets UART to use ACLK //UCRXEIE
  UCA1BR0 = 1300 & 0xFF;                // Set baud rate based on 12Mhz BCLK pg425
  UCA1BR1 = 1300 >> 8;

  UCA1CTL1 &= ~UCSWRST;                     // release from reset
  UCA1IE = UCRXIE;                        //Receive interrupt enable

  P1SEL1 |= BIT1;
  P1SEL0 |= BIT1;

  ADC10CTL0 |= ADC10SHT_2 + ADC10ON;        // ADC10ON, S&H=16 ADC clks
  ADC10CTL1 |= ADC10SHP;                    // ADCCLK = MODOSC; sampling timer
  ADC10CTL2 |= ADC10RES;                    // 10-bit conversion results
  ADC10MCTL0 |= ADC10INCH_1;                // A1 ADC input select; Vref=AVCC

  while(1)
  {
    __delay_cycles(1000);                   //   delay in us??
    ADC10CTL0 |= ADC10ENC + ADC10SC;        // Sampling and conversion start
    while (ADC10CTL1 & ADC10BUSY);          // ADC10BUSY?
    __no_operation();                       // For debug only
    TB0CCTL1 = OUTMOD_2;                      // Set/Reset Mode

    if(firstcheck == 1 && previousvalue == ADC10MEM0<<6){

    }
    else{
        previousvalue = ADC10MEM0<<6;
        TB0CCR1 = previousvalue;
        TB0CTL = TBSSEL_1 + MC_2;
        firstcheck = 1;
    }
  }
}
