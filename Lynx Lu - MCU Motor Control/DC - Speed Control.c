//DC Motor Speed Control - Lynx Lu

#include <msp430.h>

#define BUFFER_SIZE  50

volatile unsigned int top = 0;  //ind = (ind + 1) % 50;
volatile unsigned int end = 0;
volatile unsigned int size = 0;
volatile unsigned char rx_byte;
int buffer[BUFFER_SIZE];
int writeflag = 0;
volatile unsigned int watch = 0;
volatile unsigned int previousdirection = 0;

void DriveMotor(int dutycycle, int direction){  //DutyCycle is 16 bit number range
    if(previousdirection != direction){
        TB0CCTL1 = OUTMOD_2;                      // Set/Reset Mode
        TB0CCR1 = 0;
        TB0CCR2 = 0;
        TB0CTL = TBSSEL_1 + MC_2;                  // ACLK, continuous mode
    }
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
    previousdirection = direction;
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


volatile unsigned int result;
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
  /// UART CODE END


//  DriveMotor(0x7FFF,0);

  /// ADC CODE
      // Configure ADC
//      P1SEL1 |= BIT1;
//      P1SEL0 |= BIT1;
//
//      ADC10CTL0 |= ADC10SHT_2 + ADC10ON;        // ADC10ON, S&H=16 ADC clks
//      ADC10CTL1 |= ADC10SHP;                    // ADCCLK = MODOSC; sampling timer
//      ADC10CTL2 |= ADC10RES;                    // 10-bit conversion results
//      ADC10MCTL0 |= ADC10INCH_1;                // A1 ADC input select; Vref=AVCC
//  //    ADC10IE |= ADC10IE0;                      // Enable ADC conv complete interrupt
//  //    __bis_SR_register(GIE);               // enable interrupts
//
//      while(1)
//      {
//        __delay_cycles(1000);                   //   delay in us??
//        ADC10CTL0 |= ADC10ENC + ADC10SC;        // Sampling and conversion start
//        while (ADC10CTL1 & ADC10BUSY);          // ADC10BUSY?
////        __no_operation();                       // For debug only
//  //      unsigned int ADC16_VAL=ADC10MEM0<<6;  //bitshift 10bit num to 16, max 0x3FF<<6 = FFC0
////        result = ADC10MEM0<<6;
//result = ADC10MEM0 << 6;

//        DriveMotor(ADC10MEM0<<6,0);

  //        ADC16_VAL=ADC10_VAL<<8;
  //      FWD AND BCKWARDS CONTRL
  //      if (ADC16_VAL >=0x00FF){
  //          TB0CCR1=0;                            //// P1.4/TB0.1|--> CCR1 - % PWM
  //          TB0CCR2=(ADC16_VAL-0x00FF)*2;                    //P1.5/TB0.2|--> CCR2 - % PWM
  //      }else {
  //          TB0CCR1=(0x00FF-ADC16_VAL)*2;                            //// P1.4/TB0.1|--> CCR1 - % PWM
  //          TB0CCR2=0;                              //P1.5/TB0.2|--> CCR2 - % PWM
  //      }

//      }
  /// ADC code end

  __enable_interrupt();                     // Enable interrupt
  while(1)
  {
     if(size>=5){
          for(i=0; i<5; i++){
              Packet[i] = buffer[top];
              top = (top+1)%BUFFER_SIZE;
              size--;
          }
          while (!(UCA1IFG&UCTXIFG));  // wait for TX buffer to be clear
//          if(Packet[4]!= 0){  //Escape byte check
//              if(Packet[4]==1){
//                  Packet[2] = 0xFF;
//              }
//              else if(Packet[4] == 2){
//                  Packet[3] = 0xFF;
//              }
//              else if(Packet[4] == 3){
//                  Packet[2] = 0XFF;
//                  Packzet[3] = 0XFF;
//              }
//          }
          sixteenBitNumber = (Packet[2]<<8) | Packet[3]; //Upper and lower bytes merge
          watch = sixteenBitNumber;
          DriveMotor(0xFFFF - sixteenBitNumber,Packet[1]);
          sixteenBitNumber = 0;
      }
  }
}