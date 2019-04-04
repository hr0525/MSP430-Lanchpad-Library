/***************************************************************************************
 * _msp430fr6989_lanchpad.c
 *
 *  Created on	: 2018-02-24
 *  Author			:	HR052
 * 	Decribe			:	msp430fr6989_lanchpad自定义代码，主要针对低功耗、引脚配置和部分功能实现
 ***************************************************************************************/
#include"_msp430fr6989_lanchpad.h"

/*******************************************************************************************
 * name			:	_MCU_LPM_NP_
 * function	: _no_operation
 * input		: none
 * output		: none
 *******************************************************************************************/
void _MCU_LPM_NP_()
{
	__no_operation();
}

/*******************************************************************************************
 * name			:	_MCU_LPM_Set_
 * function	: set all IO port to low output
 * input		: void function -> special IO set operation 
 * 													-> _MCU_LPM_NP_ means has no special operation
 * output		: void
 *******************************************************************************************/
// low power set
void _MCU_LPMSet_(void(*ptr)())
{
	P1OUT = 0x00;
    P2OUT = 0x00;
    P3OUT = 0x00;
    P4OUT = 0x00;
    P5OUT = 0x00;
    P6OUT = 0x00;
    P7OUT = 0x00;
    P8OUT = 0x00;
    P9OUT = 0x00;
    P10OUT = 0x00;
    PJOUT = 0x00;

    P1DIR = ~P1DIR;
    P2DIR = ~P2DIR;
    P3DIR = ~P3DIR;
    P4DIR = ~P4DIR;
    P5DIR = ~P5DIR;
    P6DIR = ~P6DIR;
    P7DIR = ~P6DIR;
    P8DIR = ~P8DIR;
    P9DIR = ~P9DIR;
    P10DIR = ~P10DIR;
    PJDIR = ~PJDIR;

    (*ptr)();

    PM5CTL0 &= ~LOCKLPM5;												// Disable the GPIO power-on default high-impedance mode
                         												// to activate previously configured port settings
}

/*******************************************************************************************
 * name			:	_MCU_Px_InterruptSet_
 * function	: IO iterrupt set
 * input		: uchar portName		->	port select
							uint portBit			->	port bit select
							uint portIES			->	edge select
							void(*interruptFunction)()	->	interrupt function select
 * output		: void
 *******************************************************************************************/
// Port Interrupt function point
void (*iF)();
// port interrupt set
void _MCU_Px_InterruptSet_(
		uchar portName,
		uint portBit,
		uint portIES,
		void(*interruptFunction)())
{
	iF = interruptFunction;
	switch(portName)
	{
	case 1:
		P1OUT |= portBit;           								// Pull-up resistor on P1x
		P1REN |= portBit;           								// Select pull-up mode for P1x
		P1DIR &= ~portBit;   												// Set all but P1x to output direction
		P1IES |= portIES;          									// P1x Hi/Lo edge
		P1IFG = 0;         													// Clear all P1 interrupt flags
		P1IE |= portBit;           									// P1x interrupt enabled
		break;
	case 2:
		P2OUT |= portBit;
		P2REN |= portBit;
		P2DIR &= ~portBit;
		P2IES |= portIES;
		P2IFG = 0;
		P2IE |= portBit;
		break;
	case 3:
		P3OUT |= portBit;
		P3REN |= portBit;
		P3DIR &= ~portBit;
		P3IES |= portIES;
		P3IFG = 0;
		P3IE = portBit;
		break;
	case 4:
		P4OUT |= portBit;
		P4REN |= portBit;
		P4DIR &= ~portBit;
		P4IES |= portIES;
		P4IFG = 0;
		P4IE |= portBit;
		break;
	default:
		break;
	}
	__bis_SR_register(GIE);
}

// Port Interrupt vector
#pragma vector=PORT1_VECTOR
__interrupt void Port_1()
{
	(*iF)();
}
#pragma vector=PORT2_VECTOR
__interrupt void Port_2()
{
	(*iF)();
}
#pragma vector=PORT3_VECTOR
__interrupt void Port_3()
{
	(*iF)();
}
#pragma vector=PORT4_VECTOR
__interrupt void Port_4()
{
	(*iF)();
}

/*******************************************************************************************
 * name			:	_MCU_T0_Delay_
 * function	: use T0 as delay clock
 * input		: uint time	->	delay time
 * 											->	1/ACLK = 1/32768s = 0.0000 30517s
 * output		: void
 *******************************************************************************************/
void _MCU_T0Delay_(uint time)
{
	TA0CCTL0 = CCIE;                         		// TACCR0 interrupt enabled
	TA0CCR0 = time;
  	TA0CTL = TASSEL__ACLK | MC__UP;          	// SMCLK, UP mode
  	__bis_SR_register(LPM3_bits+GIE);
}

#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
{
	TA0CCTL0 &= ~CCIE;													// delay operation is done; close T0
	__bic_SR_register_on_exit(LPM3_bits + GIE);
}

/*******************************************************************************************
 * UCA0 UART operation
 *******************************************************************************************/
char *pTxData0;																// Tx data buff
char *pRxData0;																// Rx data Buff
uchar readFlag0,readLength0;									// Rx indicator variable

/*******************************************************************************************
 * name			:	_UCA0_UART_Init_
 * function	: UCA0 UART Init 115200 bps
 * input		: void
 * output		: void
 *******************************************************************************************/
void _UCA0_UART_Init_()
{
	_UCA0_UART_PORTINIT;												// UCA0 port initial

	// Configure USCI_A1 for UART mode
	UCA0CTLW0 = UCSWRST;               					// Put eUSCI in reset
	UCA0CTLW0 |= UCSSEL__SMCLK;         				// CLK = ACLK
	UCA0BR0 = 8;                          			// 115200 baud
	UCA0MCTLW |= 0xD600;
	UCA0BR1 = 0;
	UCA0CTL1 &= ~UCSWRST;           						// Initialize eUSCI
}

/*******************************************************************************************
 * name			:	_UCA0_UART_Tx_
 * function	: UCA0 UART Tx
 * input		: char* txBuff	-> Tx buff address
 * output		: void
 *******************************************************************************************/
void _UCA0_UART_Tx_(char* txBuff)
{
	UCA0IE |= UCTXIE;														// enable the TXIE

	pTxData0 = txBuff;													// Tx buff
	__bis_SR_register(LPM1_bits + GIE);

	while (UCA0IE & UCTXIE);
}

/*******************************************************************************************
 * name			:	_UCA0_UART_Rx_
 * function	: UCA0 UART Rx
 * input		: char* rxBuff	-> Rx buff address
 * output		: void
 *******************************************************************************************/
void _UCA0_UARTRead_(char* rxBuff)
{
	readFlag0 = 0;
	readLength0 = 0;

	UCA0IE |= UCRXIE;
	pRxData0 = rxBuff;

	__bis_SR_register(LPM1_bits + GIE);

	while (UCA0IE & UCRXIE);
}

/*******************************************************************************************
 * function			:	UCA0 UART interrupt function
 *******************************************************************************************/
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
  switch(__even_in_range(UCA0IV,4))
  {
  case 0:break;                             	// Vector 0 - no interrupt
  case 2:																			// Vector 2 - RXIFG
	  *pRxData0 = UCA0RXBUF;

  	  if(*pRxData0 == '#' | readFlag0 == 1)		// start?
  	  {
  		  readFlag0 = 1;
  		  if(*pRxData0 == '$' | readLength0 >= uartReadMaxLength) 			// out of range?
  		  {
  			  UCA0IE &= ~UCRXIE;
  			  __bic_SR_register_on_exit(LPM1_bits + GIE);									// Exit to active CPU
  		  }
  		  else
  		  {
  			  pRxData0++;
  			  readLength0++;
  		  }
  	  }
	  break;

  case 4:																			// Vector 4 - TXIFG
	  UCA0TXBUF = *(pTxData0++);
	  if (*pTxData0 == '$')
	  {
		  UCA0IE &= ~UCTXIE;
	  	  __bic_SR_register_on_exit(LPM1_bits + GIE);										// Exit active CPU
	  }
	  break;
  default: break;
  }
}

/*******************************************************************************************
 * UCA1 UART operation
 *******************************************************************************************/
char *pTxData1;																// Tx data buff
char *pRxData1;	// Rx data Buff
uchar readFlag1,readLength1;									// Rx indicator variable

/*******************************************************************************************
 * name			:	_UCA1_UART_Init_
 * function	: UCA1 UART Init 115200 bps
 * input		: void
 * output		: void
 *******************************************************************************************/
void _UCA1_UART_Init_()
{
	_UCA1_UART_PORTINIT;												// UCA1 port initial

	// Configure USCI_A1 for UART mode
	UCA1CTLW0 = UCSWRST;               					// Put eUSCI in reset
	UCA1CTLW0 |= UCSSEL__SMCLK;         				// CLK = ACLK
	UCA1BR0 = 8;                         			 	// 115200 baud
	UCA1MCTLW |= 0xD600;
	UCA1BR1 = 0;
	UCA1CTL1 &= ~UCSWRST;           						// Initialize eUSCI
}

/*******************************************************************************************
 * name			:	_UCA1_UART_Tx_
 * function	: UCA1 UART Tx
 * input		: char* txBuff	-> Tx buff address
 * output		: void
 *******************************************************************************************/
void _UCA1_UART_Tx_(char* txBuff)
{
	UCA1IE |= UCTXIE;														// enable the TXIE

	pTxData1 = txBuff;													// write buff
	__bis_SR_register(LPM1_bits + GIE);

	while (UCA1IE & UCTXIE);
}

/*******************************************************************************************
 * name			:	_UCA1_UART_Rx_
 * function	: UCA1 UART Rx
 * input		: char* txBuff	-> Rx buff address
 * output		: void
 *******************************************************************************************/
void _UCA1_UART_Rx_(char* rxBuff)
{
	readFlag1 = 0;
	readLength1 = 0;

	UCA1IE |= UCRXIE;
	pRxData1 = rxBuff;

	__bis_SR_register(LPM1_bits + GIE);

	while (UCA1IE & UCRXIE);
}

/*******************************************************************************************
 * function			:	UCA1 UART interrupt function
 *******************************************************************************************/
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
{
  switch(__even_in_range(UCA1IV,4))
  {
  case 0:break;                             	// Vector 0 - no interrupt
  case 2:																			// Vector 2 - RXIFG
	  *pRxData1 = UCA1RXBUF;

  	  if(*pRxData1 == '#' | readFlag1 == 1)		// start?
  	  {
  		  readFlag1 = 1;
  		  if(*pRxData1 == '$' | readLength1 >= uartReadMaxLength) 				// out of range?
  		  {
  			  UCA1IE &= ~UCRXIE;
  			  __bic_SR_register_on_exit(LPM1_bits + GIE);										// Exit to active CPU
  		  }
  		  else
  		  {
  			  pRxData1++;
  			  readLength1++;
  		  }
  	  }
	  break;

  case 4:																			// Vector 4 - TXIFG
	  UCA1TXBUF = *(pTxData1++);
	  if (*pTxData1 == '$')
	  {
		  UCA1IE &= ~UCTXIE;
	  	  __bic_SR_register_on_exit(LPM1_bits + GIE);											// Exit active CPU
	  }


	  break;
  default: break;
  }
}

/*******************************************************************************************
 * UCB0 I2C operation
 *******************************************************************************************/
uint RxByteCtr,TxByteCtr;
uchar *ptxData,*prxData;

/*******************************************************************************************
 * name			:	_UCB0_I2C_Init_
 * function	: UCA0 I2C Init
 * input		: void
 * output		: void
 *******************************************************************************************/
void _UCB0_I2C_Init_()
{
	_UCB0_I2C_PORTINIT;													// Assign I2C pins to USCI_B0
	P1REN |= (BIT6 + BIT7);
	P1OUT |= (BIT6 + BIT7);                     //pull-pu

	UCB0CTLW0  |= UCSWRST;                 			// Enable SW reset
	UCB0CTLW0  |= UCMODE_3 | UCMST | UCSSEL__SMCLK; 				// I2C, Master, SMCLK
	UCB0BRW = 0x08;                        			// baudrate = SMCLK / 4
	UCB0CTL1 &= ~UCSWRST;                   		// Clear SW reset, resume operation
}

/*******************************************************************************************
 * name			:	_UCB0_I2C_AddSet_
 * function	: UCB0 I2C address set
 * input		: uchar address	->	7-bit address
 * output		: void
 *******************************************************************************************/
void _UCB0_I2C_AddSet_(uchar address)
{		
	UCB0I2CSA = address;												// set slave device address
}	

/*******************************************************************************************
 * name			:	_UCB0_I2C_Tx_
 * function	: UCB0 I2C Tx
 * input		: uchar* txData	->	tx data buff address
 * 						uint dataL		->	tx data length
 * output		: void
 *******************************************************************************************/
void _UCB0_I2C_Tx_(uchar* txData, uint dataL)
{
	UCB0IE |= UCTXIE0 + UCNACKIE;								// Enable TX interrupt

	ptxData = (uchar *)txData;      						// TX array start address
	                                            // Place breakpoint here to see each
	                                            // transmit operation.
	TxByteCtr = dataL;              						// Load TX byte counter

	UCB0CTLW0  |= UCTR + UCTXSTT;             	// I2C TX, start condition

	__bis_SR_register(LPM0_bits + GIE);     		// Enter LPM0, enable interrupts
																							// Remain in LPM0 until all data is Tx'd

	while (UCB0CTLW0  & UCTXSTP);             	// Ensure stop condition got sent
	UCB0IE &= ~(UCTXIE0 + UCNACKIE);						// Disable TX interrupt
}

/*******************************************************************************************
 * name			:	_UCB0_I2C_Rx_
 * function	: UCB0 I2C Rx
 * input		: uchar* rxData	->	Rx data buff address
 * 						uint dataL		->	Rx data length
 * output		: void
 *******************************************************************************************/
void _UCB0_I2C_Rx_(uchar* rxData, uint dataL)
{
	UCB0IE |= UCRXIE0;													// Enable RX interrupt

	prxData = rxData;    										// Start of RX buffer
	RxByteCtr = dataL;                 		  	// Load RX byte counter

	while (UCB0CTLW0  & UCTXSTP);								// Ensure stop condition got sent

	UCB0CTLW0  &= ~UCTR;												// I2C Rx
	UCB0CTLW0  |= UCTXSTT;                		  // I2C start condition
	while (UCB0CTLW0  & UCTXSTT);

	if(dataL == 1)
		UCB0CTLW0  |= UCTXSTP;            				// Generate I2C stop condition

	__bis_SR_register(LPM0_bits + GIE);   		  // Enter LPM0, enable interrupts
	__no_operation();														// Remain in LPM0 until all data is Rx'd

	UCB0IE &= ~UCRXIE0;													// Disable RX interrupt
}

/*******************************************************************************************
 * function			:	UCB0 I2C interrupt function
 *******************************************************************************************/
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
	switch(__even_in_range(UCB0IV, USCI_I2C_UCBIT9IFG))
	{
		case USCI_NONE:          break;         	// Vector 0: No interrupts
	    case USCI_I2C_UCALIFG:   break;         // Vector 2: ALIFG

	    case USCI_I2C_UCNACKIFG:                // Vector 4: NACKIFG
	      UCB0CTLW0 |= UCTXSTT;                 // resend start if NACK
	      break;

	    case USCI_I2C_UCSTTIFG:  break;         // Vector 6: STTIFG
	    case USCI_I2C_UCSTPIFG:  break;         // Vector 8: STPIFG
	    case USCI_I2C_UCRXIFG3:  break;         // Vector 10: RXIFG3
	    case USCI_I2C_UCTXIFG3:  break;         // Vector 12: TXIFG3
	    case USCI_I2C_UCRXIFG2:  break;         // Vector 14: RXIFG2
	    case USCI_I2C_UCTXIFG2:  break;         // Vector 16: TXIFG2
	    case USCI_I2C_UCRXIFG1:  break;         // Vector 18: RXIFG1
	    case USCI_I2C_UCTXIFG1:  break;         // Vector 20: TXIFG1

	    case USCI_I2C_UCRXIFG0:									// Vector 22: RXIFG0
	    	RxByteCtr--;
	    	*prxData = UCB0RXBUF;           	 	// Move final RX data to pRxData1		// Decrement RX byte counter
	    	if (RxByteCtr)
	    	{
	    		prxData++;               					// Move RX data to address pRxData1
	    		if (RxByteCtr == 1)        				// Only one byte left?
	    			UCB0CTLW0 |= UCTXSTP;      				// Generate I2C stop condition
	    	}
	    	else
	    		__bic_SR_register_on_exit(LPM0_bits + GIE);			// Exit active CPU
	    	break;

	    case USCI_I2C_UCTXIFG0:                 // Vector 24: TXIFG0
			if (TxByteCtr)              					// Check TX byte counter
			{
				UCB0TXBUF = *ptxData++;      		 	// Load TX buffer
				TxByteCtr--;                				// Decrement TX byte counter
			}
			else
			{
				UCB0CTLW0 |= UCTXSTP;          			 // I2C stop condition
				UCB0IFG &= ~UCTXIFG;           			 // Clear USCI_B0 TX int flag
				__bic_SR_register_on_exit(LPM0_bits + GIE); 			// Exit LPM0
			}
			break;

	    default: break;
	}
}

/*******************************************************************************************
 * ADC12 operation
 *******************************************************************************************/
unsigned int ADC12CTL0_0,ADC12CTL0_1;
unsigned int ADC12CTL1_0,ADC12CTL1_1;
unsigned int ADC12CTL2_0,ADC12CTL2_1;

/*******************************************************************************************
 * name			:	_ADC_ADC12_Init_
 * function	: ADC12 init
 * input		: void
 * output		: void
 *******************************************************************************************/
void _ADC12_AVCCInit_()
{
    // ADC12 port init
		_ADC12_ADC_PORTINIT;

		// ADC12 close record
    ADC12CTL0_0 = ADC12CTL0;
    ADC12CTL1_0 = ADC12CTL1;
    ADC12CTL2_0 = ADC12CTL2;

    // Configure ADC12
    ADC12CTL0_1 = ADC12SHT0_2 | ADC12ON;	// Sampling time, S&H=16, ADC12 on
    ADC12CTL1_1 = ADC12SHP;           		// Use sampling timer
    ADC12CTL2_1 |= ADC12RES_2;         		// 12-bit conversion results
    ADC12MCTL0 |= ADC12INCH_7;        	 	// A7 ADC input select; Vref=AVCC
}

/*******************************************************************************************
 * name			:	_ADC_ADC12_close_
 * function	: ADC12 close, set to low power
 * input		: void
 * output		: void
 *******************************************************************************************/
void _ADC_ADC12_close_()
{
    ADC12CTL0 &= ~(ADC12ENC + ADC12SC);
    ADC12CTL0 = ADC12CTL0_0;
    ADC12CTL1 = ADC12CTL1_0;
    ADC12CTL2 = ADC12CTL2_0;
}

/*******************************************************************************************
 * name			:	_ADC_ADC12_open_
 * function	: ADC12 open
 * input		: void
 * output		: void
 *******************************************************************************************/
void _ADC12_Open_()
{
    ADC12CTL0 = ADC12CTL0_1;
    ADC12CTL1 = ADC12CTL1_1;
    ADC12CTL2 = ADC12CTL2_1;
    ADC12CTL0 |= (ADC12ENC + ADC12SC);
}
