#include <msp430.h>
#include <circular_buffer.h>

circularBuffer queue;
volatile int totalCount = 0;
volatile int referenceCount = 0;
volatile int newCommand = 0;
volatile int servoPWM = 50;
volatile float prevPWM = 0;
volatile int preverror = 0;
volatile float PWM = 0;
volatile float error = 0;


/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;   // stop watchdog timer

    /*
     * clock set up
     */
    CSCTL0_H = 0xA5;
    CSCTL1 |= DCORSEL + DCOFSEL_0;    // Set max. DCO setting =16MHz
    CSCTL2 = SELA_3 + SELS_3 + SELM_3; // set ACLK, SMCLK -> DCO, MCLK -> XT1 (24MHz)
    CSCTL3 = DIVA_1 + DIVS_3 + DIVM_0; // set ACLK/2, SMCLK/2, MCLK/1
    CSCTL4 = XTS + XT2OFF + XT1DRIVE_2; //HF mode, XT2 off
    CSCTL4 &= ~XT1OFF;

    /*
     * Queue set up
     */
    queue.stopIndex=0;
    queue.startIndex=0;
    queue.isFull=0;

    /*
     * UART
     */
    // Select P2.5 P2.6 for UART TX, RX
    P2SEL1 |= BIT5 + BIT6;
    P2SEL0 &= ~(BIT5 + BIT6);
    // Configure UART 0
    UCA1CTLW0 = UCSSEL_1 + UCSWRST; // Set ACLK = 24MHz as UCBRCLK, set to reset mode
    // For Baudrate = 9600
//    UCA1BRW = 52;
//    UCA1MCTLW = 0x4900 + UCBRF_1 +UCOS16;
//    UCA1CTLW0 &= ~UCSWRST;                     // release from reset
    UCA1BR0 = 1250 & 0xFF;                // Set baud rate based on 12Mhz BCLK pg425
    UCA1BR1 = 1250 >> 8;

    UCA1IE |= UCRXIE;                         // Enable RX interrupt

    /*
     * 20ms delay to send UART using TB2.0
     */
    //set delay to 20ms
    TB2CTL = TBSSEL_2 + MC_1 + ID_3; // Select SMCLK SOURCE /8, UP mode
    //1000kHz
    TB2CCR0 = 20000-1; // clkfreq*time
    TB2CCTL0 = CCIE; //enable interrupt

    /*
     * Encoder counter
     */
    //set input clocks for TA0CLK and TA1CLK P1.1 P1.2
    P1SEL1 |= BIT1 + BIT2;
    TA0CTL = TASSEL_0 + MC_2 + TACLR; //TA0CLK Source, Cont. mode
    TA1CTL = TASSEL_0 + MC_2 + TACLR; //TA1CLK source, cont. mode

    /*
     * Motor driver
     */
    P1DIR |= BIT4 + BIT5;
    P1SEL0 |= BIT4 + BIT5;

    /*
    * PWM generator using TB0
    */
    P1DIR |= BIT0;       // P1.0 output for TA0.1
    P1SEL0 |= BIT0;
    TB0CCR0 = 65535-1;
    TB0CCR1 = 0;     // CCR1 PWM duty cycle
    TB0CCTL1 = OUTMOD_7;         // CCR1 reset/set
    TB0CCR2 = 0;
    TB0CCTL2 = OUTMOD_7;
    TB0CTL = TASSEL_1 + MC_2;    // use ACLK, cont. mode,

    _EINT();

    //testing
    /*
    TB0CCR1 = 65533;
    __delay_cycles(6000000);
    TB0CCR1 = 0;*/

    unsigned int test;
    while(1) {
        if (bufferSize(&queue)>=3) {
        //if(bufferSize(&queue)>=2){

        int MSByte = 0;
           int LSByte = 0;
           int startingbyte = 0;
           //int commandByte = 0;
           //int endByte = 0;

           //commandByte = tryDequeue(&queue);
           while(startingbyte != 255){
               startingbyte = tryDequeue(&queue);
           }
           MSByte = tryDequeue(&queue);
           LSByte = tryDequeue(&queue);
           //servoPWM = tryDequeue(&queue);
           //endByte = tryDequeue(&queue);

           TB1CCR2 = servoPWM*100;
           test = TB1CCR2;
           referenceCount = ((MSByte & 0xFF) << 8) | (LSByte & 0xFF);

           newCommand = 0;
           //totalCount = 0;
        }
        while (newCommand == 0) {

            float Kp;
            preverror = error;

            error = referenceCount - totalCount;
            float scale = 10;
            Kp = 200;
           // PWM = prevPWM + 38.5*(error - preverror);
            //PWM = error*Kp;
            PWM = error/scale * Kp;
            //saturation limits
            if(PWM>6553){
                PWM = 65533;
            }
            else if(PWM<-6553){
                PWM = -65533;
            }
            else{
                PWM = PWM*10;
            }
//            if (PWM > 65535) { PWM = 65534; }
//            else if ( PWM < -65535) { PWM = -65534; }

           if (error > 0) {
               TB0CCR1 = PWM;
               TB0CCR2 = 0;
           }
           else if (error < 0) {
               PWM = -PWM;
               TB0CCR2 = PWM;
               TB0CCR1 = 0;
           }
           else {
               TB0CCR2 = 0;
               TB0CCR1 = 0;
           }
           prevPWM = PWM;
        }

    }
}


//Transmit Encoder count
#pragma vector = TIMER2_B0_VECTOR
__interrupt void Timer2_B0_ISR(void)
//void __attribute__ ((interrupt(TIMER2_B0_VECTOR))) Timer2_B0_ISR (void)
{
    int differenceCount;
    differenceCount = TA0R - TA1R;
    totalCount += differenceCount;
    TA0CTL = TASSEL_0 + MC_2 + TACLR;
    TA1CTL = TASSEL_0 + MC_2 + TACLR;
    int dataByteM, dataByteL;

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

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
//void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
{
  switch(__even_in_range(UCA1IV,0x08))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                   // Vector 2 - RXIFG
    while(!(UCA1IFG&UCTXIFG));             // USCI_A1 TX buffer ready
    enqueue(&queue,UCA1RXBUF);
    newCommand = 1;
    break;
  case 4:break;                             // Vector 4 - TXIFG
  default: break;
  }
}

void junkFunction (void) {
    int i;
    i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;
    i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;
    i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;i+=1;
}

