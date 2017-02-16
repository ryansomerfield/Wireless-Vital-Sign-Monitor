/*
*Wireless Vital Sign Monitor
*
*Ryan Somerfield
*
*I'll do this legit later
*/

/*

TODO:
~P1.0 should be an input pin that triggers the interrupt function for DRDY.
~Write the slave reset function to be done through SPI Write commands
~SPI read/Write function should be written to make for easier SPI transmission
~AFEInit function needs to be written
~Check the clock speed to be used







*/


#include <msp430.h>

int main(void)
{
  volatile unsigned int i;

  WDTCTL = WDTPW+WDTHOLD;                   // Stop watchdog timer
  
  /*
  Here would not be a bad place to initialize
  the DRDY Pin. This hardware interrupt will
  need to be configured
  */
  
  
  P3SEL |= BIT3+BIT4;                       // P3.3,4 option select
  P2SEL |= BIT7;                            // P2.7 option select
  
  UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
  UCA0CTL0 |= UCMST+UCSYNC+UCCKPL+UCMSB;    // 3-pin, 8-bit SPI master
                                            // Clock polarity high, MSB
  UCA0CTL1 |= UCSSEL_2;                     // SMCLK
  UCA0BR0 = 0x02;                           // /2
  UCA0BR1 = 0;                              //
  UCA0MCTL = 0;                             // No modulation
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

  
  /*This slave reset can be done using 
  software?
  //P1OUT &= ~0x02;                           // Now with SPI signals initialized,
  //P1OUT |= 0x02;                            // reset slave
  */
  
  
  for(i=50;i>0;i--);                        // Wait for slave to initialize

											// Initialize data values
/*
Create the AFEInit() function to initialize
the defaults for the AFE4400 chip. The call
for AFEInit() will got here.
											
*/											
	__bis_SR_register(GIE);                   // enable interrupts		
											  //
  while(1){
	/*
	  This is the loop that will run continuously.
	  Place loop code here.
	*/
	
  /*These should be placed in the
  AFERead / AFE Write functions that will be created below
  while (~(UCA0IFG&UCTXIFG));                // USCI_A0 TX buffer ready?
  UCA0TXBUF = MST_Data;                     // Transmit first character
*/
  }
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
  state = HIGH;
 /*
 This is the blink function. Set the state to high here
 */
}



void AFE4490Init(void){
	//Create the AFEInit() function to initialize
    //the defaults for the AFE4400 chip.
}

void AFE4490Write(uint8_t address, uint32_t data){
	//while (~(UCA0IFG&UCTXIFG));                // USCI_A0 TX buffer ready?-Unsure if this is needed.
	//Write the slave select
	//to be low
	UCA0TXBUF = address;					   // send address to device
    UCA0TXBUF =((data >> 16) & 0xFF); 		   // write top 8 bits
    UCA0TXBUF =((data >> 8) & 0xFF); 		   // write middle 8 bits
    UCA0TXBUF =(data & 0xFF); 				   // write bottom 8 bits    
    //Write the slave select
	//to be low
}

unsigned long AFE4490Read (uint8_t address)
{       
    /*
	Read the datasheet to learn how the
	SPI Read can be done
	*/
}




/*
*******************************************************************************************************************

/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 * 
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--
//******************************************************************************
//   MSP430F552x Demo - USCI_A0, SPI 3-Wire Master Incremented Data
//
//   Description: SPI master talks to SPI slave using 3-wire mode. Incrementing
//   data is sent by the master starting at 0x01. Received data is expected to
//   be same as the previous transmission.  USCI RX ISR is used to handle
//   communication with the CPU, normally in LPM0. If high, P1.0 indicates
//   valid data reception.  Because all execution after LPM0 is in ISRs,
//   initialization waits for DCO to stabilize against ACLK.
//   ACLK = ~32.768kHz, MCLK = SMCLK = DCO ~ 1048kHz.  BRCLK = SMCLK/2
//
//   Use with SPI Slave Data Echo code example.  If slave is in debug mode, P1.1
//   slave reset signal conflicts with slave's JTAG; to work around, use IAR's
//   "Release JTAG on Go" on slave device.  If breakpoints are set in
//   slave RX ISR, master must stopped also to avoid overrunning slave
//   RXBUF.
//
//                   MSP430F552x
//                 -----------------
//             /|\|                 |
//              | |                 |
//              --|RST          P1.0|-> Interrup Pin
//                |                 |
//                |             P3.3|-> Data Out (UCA0SIMO)
//                |                 |
//                |             P3.4|<- Data In (UCA0SOMI)
//                |                 |
//  Slave reset <-|P1.1         P2.7|-> Serial Clock Out (UCA0CLK)
//
//
//   Bhargavi Nisarga
//   Texas Instruments Inc.
//   April 2009
//   Built with CCSv4 and IAR Embedded Workbench Version: 4.21
//******************************************************************************
*/


