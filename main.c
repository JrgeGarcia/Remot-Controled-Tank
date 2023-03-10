#include "msp.h"
#define PERIOD1 1000;
void sendString(char *str);
void sendChar(char s);
void main(void)
{
      WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD; // stop watchdog timer
    // Error LED
    P1->DIR |= BIT0;    // set BIT0 as OUTPUT
    P1->OUT &= ~(BIT0); // set BIT0 as LOW
    // clear secondary functions
    P1->SEL0 &= ~(BIT0);
    P1->SEL1 &= ~(BIT0);
    // RGB LEDs
    P2->DIR |= BIT0 | BIT1 | BIT2;    // set BIT0,1,2 as OUTPUTs
    P2->OUT &= ~(BIT0 | BIT1 | BIT2); // set BIT0,1,2 as LOW
    // clear secondary functions
    P2->SEL0 &= ~(BIT0 | BIT1 | BIT2);
    P2->SEL1 &= ~(BIT0 | BIT1 | BIT2);
    // Enable UART0 Pins
    // P3.2->RX
    // P3.3->TX
    P3->SEL0 |= BIT2 | BIT3;
    P3->SEL1 &= ~(BIT2 | BIT3);
    /*
     * PWM INIT MOTOR DRIVER TIMER A1
     * PINS Motor A : P5.6| Motor B: P5.7
     */
    P5->DIR |= (BIT6 | BIT7);
    P5->SEL0 |= (BIT6 | BIT7);
    P5->SEL1 &= ~(BIT6 | BIT7);
    TIMER_A2->CCR[0] = PERIOD1 -1;
    TIMER_A2->CCTL [1] = TIMER_A_CCTLN_OUTMOD_7; //clear & reset
    TIMER_A2->CCR[1] = 900;   //Wheel A
    TIMER_A2->CCTL [2] = TIMER_A_CCTLN_OUTMOD_7; //clear & reset
    TIMER_A2->CCR[2] = 900  //Wheel B
    TIMER_A2->CTL = TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_MC__UP |
            TIMER_A_CTL_CLR;

    /*
        * Port-4 controls the H-Bridge Motor Driver
        * Wheel A-IN 1&2 Wheel B-IN 3&4
        * Forward IN 1 & 3 [1010] *activating pins  from on and off
        * Left IN 2 & 3 [0110]
        * Right In 1 & 4 [1001]
        * Stop [0000]
        * CCR1 - Wheel A | CCR2 - Wheel B * For future work
        */
       P4->DIR |= (BIT0|BIT1|BIT2|BIT3);
       P4->OUT &= ~(BIT0|BIT1|BIT2|BIT3);
       P4->REN |= (BIT0|BIT1|BIT2|BIT3);
    // UART0 Configuration
    // Enhanced Universal Serial Control Interface = EUSCI
    EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST;                               // Clear previous configuration of UART
    EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SSEL__SMCLK;                        // Select SMClock, no parity, 1 stop bit, 8 bits, LSB
    EUSCI_A2->BRW = 19;                                                  // Baudrate width, SMClock/16/DR -> 3000000/16/9600 = 19.53125
    EUSCI_A2->MCTLW = (9 << EUSCI_A_MCTLW_BRF_OFS | EUSCI_A_MCTLW_OS16); // // 19.53125 - 19 = 0.53125 * 16 = 8.5, round up to 9
    EUSCI_A2->CTLW0 &= ~(EUSCI_A_CTLW0_SWRST);                           // clear reset bit
    // enable UART interrupt
    EUSCI_A2->IE |= EUSCI_A_IE_RXIE;     // enable receiver interrupt
    EUSCI_A2->IFG &= ~(EUSCI_A_IE_RXIE); // clear interrupt flag
    // enable NVIC for UART0
    NVIC->ISER[0] = 1 << (EUSCIA2_IRQn & 31);
    // enable global interrupts
    __enable_irq();
  
    while (1);
}

void EUSCIA2_IRQHandler(void)
{
    if (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG) // recieve interrupt
    {
        char c = EUSCI_A2->RXBUF; // store data into character buffer, and clear flag
        sendChar(c);              // display character through the SERIAL port
        sendString("\r\n");       // send carriage return and new line
        // check the recieved character
        if (c == 'b'){  ///Forward
        P4->OUT |= BIT0;
        P4->OUT &= ~BIT1;
        P4->OUT |= BIT2;
        P4->OUT &= ~BIT3;
        TIMER_A2->CCR[1] = 4500;
        TIMER_A2->CCR[2] = 4500;
        }
        if (c == 'f'){  //Backwards
        P4->OUT &= ~BIT0;
        P4->OUT |= BIT1;
        P4->OUT &= ~BIT2;
        P4->OUT |= BIT3;
        TIMER_A2->CCR[1] = 5000;
        TIMER_A2->CCR[2] = 5000;
        }
        if (c == 'l'){ //Left
        P4->OUT &= ~BIT0;
        P4->OUT |= BIT1;
        P4->OUT |= BIT2;
        P4->OUT &= ~BIT3;
        TIMER_A2->CCR[1] = 4500;
        TIMER_A2->CCR[2] = 3000;
        }
        if (c == 'r'){ //Right
        P4->OUT |= BIT0;
        P4->OUT &= ~BIT1;
        P4->OUT &= ~BIT2;
        P4->OUT |= BIT3;
        TIMER_A2->CCR[1] = 30000;
        TIMER_A2->CCR[2] = 45000;
        }
        if (c== 's'){   //Neutral Stop
        P4->OUT &= ~BIT0;
        P4->OUT &= ~BIT1;
        P4->OUT &= ~BIT2;
        P4->OUT &= ~BIT3;
        TIMER_A2->CCR[0] = 0;
        TIMER_A2->CCR[1] = 0;
        }
        else
        {
            int i = 0; // create iterator variable
            for (; i < 4; i++)
            {                            // toggle twice
                P1->OUT ^= BIT0;         // toggle the led
                __delay_cycles(1000000); // small delay
            }
            P1->OUT &= ~(BIT0); // clear the led
        }
    }
}

void sendString(char *str)
{
    int i;                           // create variable
    for (i = 0; str[i] != '\0'; i++) // iterate over the end of the string
    {
        while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG)); // wait until is ready to transmit
        EUSCI_A2->TXBUF = str[i]; // send character through buffer
    }
}

void sendChar(char s)
{
    while (!(EUSCI_A2->IFG & EUSCI_A_IFG_TXIFG)); // wait until is ready to transmit
    EUSCI_A2->TXBUF = s; // send character through buffer
}
