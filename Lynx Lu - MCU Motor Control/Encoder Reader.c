//Encoder Reader - Maxon Motor

#include <msp430.h>

#define BUFFER_SIZE  50

volatile unsigned int top = 0;  //ind = (ind + 1) % 50;
volatile unsigned int end = 0;
volatile unsigned int size = 0;
volatile unsigned char rx_byte;
int buffer[BUFFER_SIZE];

volatile unsigned int sectop = 0;  //ind = (ind + 1) % 50;
volatile unsigned int secend = 0;
volatile unsigned int secsize = 0;
volatile unsigned char secrx_byte;
int secondarybuffer[BUFFER_SIZE];

int direction = 0;

int writeflag = 0;
volatile unsigned int watch = 0;
volatile unsigned int previousdirection = 0;


void DriveMotor(int dutycycle, int direction2){  //DutyCycle is 16 bit number range
    if(previousdirection != direction2){
        TB0CCTL1 = OUTMOD_2;                      // Set/Reset Mode
        TB0CCR1 = 0;
        TB0CCR2 = 0;
        TB0CTL = TBSSEL_1 + MC_2;                  // ACLK, continuous mode
    }
    if(direction2 == 0){
        TB0CCTL1 = OUTMOD_2;                      // Set/Reset Mode
        TB0CCR1 = dutycycle;
        TB0CCR2 = 0;
        TB0CTL = TBSSEL_1 + MC_2;                  // ACLK, continuous mode
    }
    if(direction2 == 1){                   // Set/Reset Mode
        TB0CCTL2 = OUTMOD_2;                      // Set/Reset Mode
        TB0CCR2 = dutycycle;
        TB0CCR1 = 0;
        TB0CTL = TBSSEL_1 + MC_2;                  // ACLK, continuous mode
    }
    previousdirection = direction2;
}

#pragma vector = USCI_A1_VECTOR             // USCI ISR
__interrupt void USCI_A1_ISR(void)
{
//    switch(__even_in_range(UCA1IV,0x08))
//    {
//    case 0:break;                             // Vector 0 - no interrupt
//    case 2:                                   // Vector 2 - RXIFG
      if(end==50){
          end = 0;

      }
      buffer[end] = UCA1RXBUF; // grab byte from RX buffer
      end = end + 1;
      size = size + 1;
//      UCA1TXBUF = UCA1RXBUF;                  // TX -> RXed character
//
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

  //Set Pin 1.1 and 1.2 to input and to TACLK (TA0CLK and TA1CLK)
  P1SEL1 |= BIT1 + BIT2;
  TA0CTL = TASSEL_0 + MC_2 + TACLR; //TA0CLK Source, Cont. mode
  TA1CTL = TASSEL_0 + MC_2 + TACLR; //TA1CLK source, cont. mode

  //set delay to 100ms Sampling Time of the Encoder
  TB2CTL = TBSSEL_2 + MC_1 + ID_3; // Select SMCLK SOURCE /8, UP mode
  TB2CCR0 = 50000-1;//25000-1; // clkfreq*time
  TB2CCTL0 = CCIE; //enable interrupt

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
        sixteenBitNumber = (Packet[2]<<8) | Packet[3]; //Upper and lower bytes merge
        watch = sixteenBitNumber;
        DriveMotor((0xFFFF - sixteenBitNumber)<<4,Packet[1]);
        sixteenBitNumber = 0;
        }
  }
}