#include <msp430.h>

volatile int motorDirection = 0;
volatile unsigned int PWM;

#define BUFFER_SIZE  50

volatile unsigned int top = 0;  //ind = (ind + 1) % 50;
volatile unsigned int end = 0;
volatile unsigned int size = 0;
volatile unsigned char rx_byte;
int buffer[BUFFER_SIZE];

int writeflag = 0;
volatile unsigned int watch = 0;
volatile unsigned int previousdirection = 0;
//
float error = 0;
float countdiff = 0;
int closeloopflag = 0;


void DriveMotor(int dutycycle, int direction2){  //DutyCycle is 16 bit number range
    if(dutycycle > 65535){ //saturation
        dutycycle = 65535;
    }

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

int main(void)
{
    unsigned int Packet[5];
    unsigned int i;

    WDTCTL = WDTPW + WDTHOLD;   // stop watchdog timer

    CSCTL0_H = 0xA5; //CSKEY - 0xA5 to enable 0write
    CSCTL1 |= DCORSEL + DCOFSEL_3;//DCOFSEL_2;//// Set max. DCO setting, set to 24MHz
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

    //set delay to 100ms - changed to 50ms
//    TB2CTL = TBSSEL_2 + MC_1 + ID_3; // Select SMCLK SOURCE /8, UP mode
    TB2CTL = TBSSEL_2 + MC_1 + ID_3; // Select SMCLK SOURCE /2, UP mode
//    TB2CCR0 = 25000-1; // clkfreq*time
    TB2CCR0 = 25000-1; // clkfreq*time
    TB2CCTL0 = CCIE; //enable interrupt

    //count encoder on pin 1 and 2 (count into the timers)
    P1SEL1 |= BIT1 + BIT2;
    TA0CTL = TASSEL_0 + MC_2 + TACLR; //TA0CLK Source, Cont. mode
    TA1CTL = TASSEL_0 + MC_2 + TACLR; //TA1CLK source, cont. mode

    //Configure Motor Driver Pins
    P1SEL1 &= ~BIT5 + ~BIT4;
    P1SEL0 |= BIT5 + BIT4;
    P1DIR |= BIT5 + BIT4;

    _EINT();

    float Kp = 50;

    while(1) {
        if(closeloopflag == 1){
            if(error > 0){
                DriveMotor(Kp*error,1);
            }
            else if(error < 0){
                DriveMotor(Kp*-1*error,0);
            }
        }
         if(size>=5){
              for(i=0; i<5; i++){
                  Packet[i] = buffer[top];
                  top = (top+1)%BUFFER_SIZE;
                  size--;
              }
//              if(Packet[0]!=255){
//                  top = 0;  //ind = (ind + 1) % 50;
//                  end = 0;
//                  size = 0;
//                  continue;
//              }
              closeloopflag = 1;
              error = (Packet[2]<<8) | Packet[3]; //Only goes up to 360

              if(Packet[4] == 1){
                  error = -1 * error;
              }
         }
    }
}

int differenceCount;
#pragma vector = TIMER2_B0_VECTOR
__interrupt void Timer2_B0_ISR(void)
{
    differenceCount = TA0R - TA1R;

    if(closeloopflag == 1){
        float test = TA0R;
        float test2 = TA1R;
        error = error - TA0R + TA1R;
    }

    TA0CTL = TASSEL_0 + MC_2 + TACLR;
    TA1CTL = TASSEL_0 + MC_2 + TACLR;
    int dataByteM, dataByteL;

//    transmit start byte
    while(!(UCA1IFG&UCTXIFG));
    UCA1TXBUF = 255;

    //transmit MSB
    dataByteM = (differenceCount >> 8) & (0xFF);
    while(!(UCA1IFG&UCTXIFG));
    UCA1TXBUF = dataByteM;

    //transmit LSB
    dataByteL = (differenceCount) & (0xFF);
    while(!(UCA1IFG&UCTXIFG));
    UCA1TXBUF = dataByteL;

    TB2CCTL0 &= ~CCIFG;
}

#pragma vector = USCI_A1_VECTOR             // USCI ISR
__interrupt void USCI_A1_ISR(void)
{
    switch(__even_in_range(UCA1IV,0x08))
    {
    case 0:break;                             // Vector 0 - no interrupt
    case 2:                                   // Vector 2 - RXIFG
          if(end==50){
              end = 0;
          }
          buffer[end] = UCA1RXBUF; // grab byte from RX buffer
          end = end + 1;
          size = size + 1;
      break;
    case 4:break;                             // Vector 4 - TXIFG
    default: break;
    }
}