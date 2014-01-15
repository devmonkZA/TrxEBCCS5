/*
 * interrupts.c
 *
 * A LIST OF INTERRUPT VECTORS FOR THIS MSP430 FAMILY
 * THIS PREVENTS THE COMPILER FROM COMPLAINING ABOUT MISSING HANDLERS
 * CHANGE "#if 1" TO "#if 0" FOR HANDLERS DECLARED WITHIN YOUR CODE
 *
 */
#include "msp430.h"

#if defined (__MSP430F5438A__)
#if 0
#pragma vector=PORT1_VECTOR
__interrupt void port1_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/
#if 0
#pragma vector=PORT2_VECTOR
__interrupt void port2_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/
#if 1
#pragma vector=TIMER0_A1_VECTOR
__interrupt void timer0_a1_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/
#if 1
#pragma vector=TIMER0_A0_VECTOR
__interrupt void timer0_a0_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/
#if 1
#pragma vector=TIMER0_B1_VECTOR
__interrupt void timer0_b1_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/
#if 0
#pragma vector=TIMER0_B0_VECTOR
__interrupt void timer0_b0_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/

#if 1
#pragma vector=TIMER1_A1_VECTOR
__interrupt void timer1_a1_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/
#if 1
#pragma vector=TIMER1_A0_VECTOR
__interrupt void timer1_a0_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/
#if 0
#pragma vector=TIMER2_A0_VECTOR
__interrupt void timer2_a0_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/
#if 0
#pragma vector=TIMER2_A1_VECTOR
__interrupt void timer2_a1_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/


#if 1
#pragma vector=ADC12_VECTOR
__interrupt void adc10_interrupt(void)
{
	_no_operation();
}
#endif

/********************************************/
#if 0
#pragma vector=USCIAB0TX_VECTOR
__interrupt void usciab0_tx_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/
#if 0
#pragma vector=USCIAB0RX_VECTOR
__interrupt void usciab0_rx_interrupt(void)
{
	_no_operation();
}
#endif

/********************************************/
#if 1
#pragma vector=USCI_B2_VECTOR
__interrupt void usci_b2_interrupt(void)
{
	_no_operation();
}
#endif

/********************************************/
#if 1
#pragma vector=USCI_A2_VECTOR
__interrupt void usci_a2_interrupt(void)
{
	_no_operation();
}
#endif

#if 1
#pragma vector=USCI_B3_VECTOR
__interrupt void usci_b3_interrupt(void)
{
	_no_operation();
}
#endif

#if 1
#pragma vector=USCI_A1_VECTOR
__interrupt void usci_a1_interrupt(void)
{
	_no_operation();
}
#endif

#if 1
#pragma vector=RTC_VECTOR
__interrupt void rtc_interrupt(void)
{
	_no_operation();
}
#endif


/********************************************/
#if 1
#pragma vector=USCI_A3_VECTOR
__interrupt void usci_a3_interrupt(void)
{
	_no_operation();
}
#endif

/********************************************/
#if 1
#pragma vector=USCI_B0_VECTOR
__interrupt void usci_b0_interrupt(void)
{
	_no_operation();
}
#endif

/********************************************/
#if 1
#pragma vector=USCI_A0_VECTOR
__interrupt void usci_a0_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/
#if 1
#pragma vector=USCI_B1_VECTOR
__interrupt void usci_b1_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/
#if 0
#pragma vector=WDT_VECTOR
__interrupt void watchDog_interrupt(void)
{
	_no_operation();
}
#endif

/********************************************/
#if 0
#pragma vector=COMP_B_VECTOR
__interrupt void comp_b_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/
#if 1
#pragma vector=DMA_VECTOR
__interrupt void dma_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/
#if 0
#pragma vector=USB_UBM_VECTOR
__interrupt void usb_ubm_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/

#if 1
#pragma vector=UNMI_VECTOR
__interrupt void unmi_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/
#if 1
#pragma vector=SYSNMI_VECTOR
__interrupt void sysnmi_interrupt(void)
{
	_no_operation();
}
#endif
/********************************************/


#else
#error "DEVICE NOT SUPPORTED"
#endif
