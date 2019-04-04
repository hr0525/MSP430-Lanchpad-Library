/***************************************************************************************
 * _msp430fr6989_lanchpad.h
 *
 *  Created on	: 2018-02-24
 *  Author			:	HR052
 * 	Decribe			:	msp430fr6989_lanchpad自定义代码，主要针对低功耗、引脚配置和部分功能实现
 ***************************************************************************************/

#ifndef MSP430FR6989_LANCHPAD_H_
#define MSP430FR6989_LANCHPAD_H_

#include"msp430.h"
#include <stdio.h>

#define uchar unsigned char
#define uint unsigned int

// CS
struct systemClk_kHz {
    unsigned int MCLK;
    unsigned int SMCLK;
    unsigned int ACLK;
};

// UART
#define uartReadMaxLength 20

// Lanchpad
// LED port operation
#define _LP_LEDR_INIT 	P9DIR |= BIT7;
#define _LP_LEDR_on 	P9OUT |= BIT7;
#define _LP_LEDR_off 	P9OUT &= ~BIT7;

#define _LP_LEDL_INIT 	P1DIR |= BIT0;
#define _LP_LEDL_on 	P1OUT |= BIT0;
#define _LP_LEDL_off 	P1OUT &= ~BIT0;

// function port init
#define _UCB0_I2C_PORTINIT\
    P1SEL1 &= ~(BIT6 | BIT7);\
	P1SEL0 |= BIT6 | BIT7;		// Configure P1.6/P1.7 for I2C
#define _ADC12_ADC_PORTINIT\
	P8SEL1 |= BIT4;\
	P8SEL0 |= BIT4;				// Configure P1.1 for ADC
#define _UCA1_UART_PORTINIT\
	P3SEL0 |= BIT4 | BIT5;\
	P3SEL1 &= ~(BIT4 | BIT5);	// Configure P3.4/P3.5 for USCI_A1 UART operation
#define _UCA0_UART_PORTINIT\
	P2SEL0 |= BIT0 | BIT1;\
	P2SEL1 &= ~(BIT0 | BIT1);	// Configure P2.0/P2.1 for USCI_A0 UART operation

// function
// MCU basic operation
/*******************************************************************************************
 * MCU basic operation
 *******************************************************************************************/
void _MCU_LPM_NP_();
void _MCU_LPMSet_(void(*ptr)());
void _MCU_Px_InterruptSet_(uchar portName,uint portBit,uint portIES,void(*interruptFunction)());
void _MCU_T0Delay_(uint time);
/*******************************************************************************************
 * UCA0 UART operation
 *******************************************************************************************/
void _UCA0_UART_Init_();
void _UCA0_UART_Tx_(char* txBuff);
void _UCA0_UARTRead_(char* rxBuff);
/*******************************************************************************************
 * UCA1 UART operation
 *******************************************************************************************/
void _UCA1_UART_Init_();
void _UCA1_UART_Tx_(char* txBuff);
void _UCA1_UART_Rx_(char* rxBuff);
/*******************************************************************************************
 * UCB0 I2C operation
 *******************************************************************************************/
void _UCB0_I2C_Init_();
void _UCB0_I2C_AddSet_(uchar address);
void _UCB0_I2C_Tx_(uchar* txData, uint dataL);
void _UCB0_I2C_Rx_(uchar* rxData, uint dataL);
/*******************************************************************************************
 * ADC12 operation
 *******************************************************************************************/
void _ADC12_AVCCInit_();
void _ADC_ADC12_close_();
void _ADC12_Open_();


#endif /* MSP430FR6989_LANCHPAD_H_ */
