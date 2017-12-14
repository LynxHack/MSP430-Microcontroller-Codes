//Stepper Motor Control - Lynx Lu
#include <msp430.h>

#define BUFFER_SIZE  50

volatile unsigned int top = 0;  //ind = (ind + 1) % 50;
volatile unsigned int end = 0;
volatile unsigned int size = 0;
volatile unsigned char rx_byte;
int buffer[BUFFER_SIZE];
int writeflag = 0;

volatile unsigned int position = 0; //default 0
//volatile unsigned int directio;

//Lookup table of coils
volatile unsigned int coilone[8]   = {1,1,0,0,0,0,0,1};
volatile unsigned int coiltwo[8]   = {0,0,0,1,1,1,0,0};
volatile unsigned int coilthree[8] = {0,1,1,1,0,0,0,0};
volatile unsigned int coilfour[8]  = {0,0,0,0,0,1,1,1};

int myLUT[8][4] = { {1, 0, 0, 0},
                        {1, 1, 0, 0},
                        {0, 1, 0, 0},
                        {0, 1, 1, 0},
                        {0, 0, 1, 0},
                        {0, 0, 1, 1},
                        {0, 0, 0, 1},
                        {1, 0, 0, 1} };

void DriveStepperMotor(int direction){//int dutycycle,int direction){  //DutyCycle is 16 bit number range
    if(direction == 1){ //increment position state by one
        if(position == 7)
            position = 0;
        else
            position = position + 1;
    }
    else if(direction == 0){
        if(position == 0)
            position = 7;
        else
            position = position - 1;
    }

    P1OUT = (BIT4*myLUT[position][0]) + (BIT5*myLUT[position][1]);
    P3OUT = (BIT5*myLUT[position][2]) + (BIT4*myLUT[position][3]);
//

//    if(coilone[position]==1){
//        P1OUT |= BIT4;
//    }
//    else{
//        P1OUT &= ~BIT4;
//    }
//
//    if(coiltwo[position]==1){
//        P1OUT |= BIT5;
//    }
//    else{
//        P1OUT &= ~BIT5;
//    }
//
//    if(coilthree[position]==1){
//        P3OUT |= BIT4;
//    }
//    else{
//        P3OUT &= ~BIT4;
//    }
//
//    if(coilfour[position]==1){
//        P3OUT |= BIT5;
//    }
//    else{
//        P3OUT &= ~BIT5;
//    }
}

#pragma vector = USCI_A1_VECTOR             // USCI ISR
__interrupt void USCI_A1_ISR(void)
{
    while (!(UCA1IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
    if(end==50){
      end = 0;
    }
    buffer[end] = UCA1RXBUF; // grab byte from RX buffer
      end = end + 1;
    size = size + 1;
}

//#pragma vector=TIMER0_A1_VECTOR
//__interrupt void Timer_A(void)
//{
//    DriveStepperMotor(0x7FFF,direction);
//
//    TA1CTL &= ~CCIFG;
//    TA1CTL &= ~TAIFG;
//  }


volatile unsigned int result;
int main(void)
{
    unsigned int Packet[5];
    unsigned int i;
    unsigned int sixteenBitNumber;

    float fraction = 1;
    unsigned int direction = 0;
    WDTCTL = WDTPW + WDTHOLD;                 // stop watchdog

  // XT1 Setup
  PJSEL0 |= BIT4 + BIT5;

  //Configure Motor Driver Pins
//  P1SEL1 &= ~BIT5 + ~BIT4;
//  P1SEL0 |= BIT5 + BIT4;
  P1DIR |= BIT5 + BIT4;

//  P3SEL1 &= ~BIT5 + ~BIT4;
//  P3SEL0 |= BIT5 + BIT4;
  P3DIR |= BIT5 + BIT4;

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

  P2SEL1 &= ~BIT4;
  P2SEL0 |= BIT4;
  P2DIR |= BIT4;

//  int moveonceflag = 0;
  __enable_interrupt();                     // Enable interrupt
  while(1)
  {
      if(fraction<0.95 && (direction != 2 && direction !=3)){ //if fraction become 0, stop the drive stepper
          DriveStepperMotor(direction);  //Packet[1] is binary 0 and 1 for direction
      }

      if(size>=5){
          for(i=0; i<5; i++){
              Packet[i] = buffer[top];
              top = (top+1)%BUFFER_SIZE;
              size--;
          }
          while (!(UCA1IFG&UCTXIFG));  // wait for TX buffer to be clear
          direction = Packet[1];
//          direction = 0;

          if(direction == 2){
                  DriveStepperMotor(0);
          }
          if(direction == 3){
                  DriveStepperMotor(1);
          }


          sixteenBitNumber = (Packet[2]<<8) | Packet[3]; //Upper and lower bytes merge
          fraction = ((float)(0xFFFF-sixteenBitNumber))/(float)65535;

      }
////      __delay_cycles(1*1000000);            //1 second delay
      if(fraction < 0.1){
          fraction = 0.1;
      }

      long int wait = (long int)200000* fraction;                          // SW Delay
      long int wait2 = 100000;
      do{
          wait--;
          do{
              wait2--;
          }
          while(wait2!=0);
      }
      while (wait != 0);
  }
}

