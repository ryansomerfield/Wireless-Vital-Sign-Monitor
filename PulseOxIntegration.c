/*
 *Wireless Vital Sign Monitor
 *
 *ISE Senior Design Group 9
 */

/*
TODO:
~P1.2 should be an input pin that triggers the interrupt function for DRDY.
~Write the slave reset function to be done through SPI Write commandss
~SPI read/Write function should be written to make for easier SPI transmission
~AFEInit function needs to be written
~Check the clock speed to be used
 */

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
#include <stdio.h>
#include <stdint.h>
#include <msp430.h>




#define CONTROL0    0x00
#define LED2STC     0x01
#define LED2ENDC    0x02
#define LED2LEDSTC    0x03
#define LED2LEDENDC   0x04
#define ALED2STC    0x05
#define ALED2ENDC   0x06
#define LED1STC     0x07
#define LED1ENDC    0x08
#define LED1LEDSTC    0x09
#define LED1LEDENDC   0x0a
#define ALED1STC    0x0b
#define ALED1ENDC   0x0c
#define LED2CONVST    0x0d
#define LED2CONVEND   0x0e
#define ALED2CONVST   0x0f
#define ALED2CONVEND  0x10
#define LED1CONVST    0x11
#define LED1CONVEND   0x12
#define ALED1CONVST   0x13
#define ALED1CONVEND  0x14
#define ADCRSTCNT0    0x15
#define ADCRSTENDCT0  0x16
#define ADCRSTCNT1    0x17
#define ADCRSTENDCT1  0x18
#define ADCRSTCNT2    0x19
#define ADCRSTENDCT2  0x1a
#define ADCRSTCNT3    0x1b
#define ADCRSTENDCT3  0x1c
#define PRPCOUNT    0x1d
#define CONTROL1    0x1e
#define SPARE1      0x1f
#define TIAGAIN     0x20
#define TIA_AMB_GAIN  0x21
#define LEDCNTRL    0x22
#define CONTROL2    0x23
#define SPARE2      0x24
#define SPARE3      0x25
#define SPARE4      0x26
#define SPARE4      0x26
#define RESERVED1   0x27
#define RESERVED2   0x28
#define ALARM     0x29
#define LED2VAL     0x2a
#define ALED2VAL    0x2b
#define LED1VAL     0x2c
#define ALED1VAL    0x2d
#define LED2ABSVAL    0x2e
#define LED1ABSVAL    0x2f
#define DIAG      0x30


void AFE4490Init (void);
void AFE4490Write (unsigned char address, unsigned long data);
unsigned long AFE4490Read (unsigned char address);




int DRDY_flag = 0;

volatile char DataPacketHeader[16];
volatile unsigned int buff=0;
volatile int Responsebyte = 0;
unsigned long IRtemp;
unsigned long REDtemp;
unsigned long Pkt_Counter=0;
signed long seegtemp=0;
signed long seegtemp2=0;
volatile int i;





int main(void)
{

   	WDTCTL = WDTPW+WDTHOLD;                   // Stop watchdog timer



   P4SEL |= BIT1+BIT2+BIT3;  			// Set SPI peripheral bits
   P4DIR |= BIT0+BIT1+BIT3;			// STE, SCLK, and DOUT as output
   P4DIR &= ~BIT2;                         	// Din as input
   P4OUT |=BIT0;				// Set STE high
   UCB1CTL1 |= UCSWRST;               		// Enable SW reset
   UCB1CTL0 |= UCMSB+UCCKPH+UCMST+UCSYNC;	// [b0]   1 -  Synchronous mode
											   // [b2-1] 00-  3-pin SPI
											   // [b3]   1 -  Master mode
											   // [b4]   0 - 8-bit data
											   // [b5]   1 - MSB first
											   // [b6]   0 - Clock polarity high.
											   // [b7]   1 - Clock phase - Data is captured on the first UCLK edge and changed on the following edge.

   UCB1CTL1 |= UCSSEL_2;               	// SMCLK
   UCB1BR0 = 0x01;                             // 8 MHz
   UCB1BR1 = 0;                                //
   UCB1CTL1 &= ~UCSWRST;              		// Clear SW reset, resume operation
   UCB1IE =0x0;




   AFE4490Write(CONTROL0,0x000008);
   volatile unsigned short Init_i, j;
   for (j = 0; j < 10; j++)
   {
	 for ( Init_i =0; Init_i < 20000; Init_i++);
	 for ( Init_i =0; Init_i < 20000; Init_i++);
	 for ( Init_i =0; Init_i < 20000; Init_i++);
   }

  P2SEL &= ~BIT3; 								//Sets pins 2.3 to I/O function
  P2DIR &= ~BIT3;							//Clear 2.3 to make it an input
  P2REN |= BIT3;							//Enable the 2.3 resistor
  P2OUT |= BIT3;							//Clear 2.3's output
  P2IE |= BIT3;								//Enable 2.3's Interrupt
  P2IFG &= ~BIT3;							//Clear the Interrupt Flag


	// Initialize data values
	AFE4490Init();

	__bis_SR_register(GIE);                   // enable interrupts
	//
	while(1){
		if (DRDY_flag == 1){
			P2IE &= ~BIT3;								//Disable 2.3's Interrupt
			AFE4490Write(CONTROL0,0x000001);
			IRtemp = AFE4490Read(LED1VAL);
			AFE4490Write(CONTROL0,0x000001);
			REDtemp = AFE4490Read(LED2VAL);
			Responsebyte = 1;
		}
		if(Responsebyte == 1){
			  buff = 0;
			  Pkt_Counter++;


			  IRtemp = (unsigned long) (IRtemp<<10);
			  seegtemp = (signed long) (IRtemp);
			  seegtemp = (signed long) (seegtemp>>10);

			  REDtemp = (unsigned long) (REDtemp<<10);
			  seegtemp2 = (signed long) (REDtemp);
			  seegtemp2 = (signed long) (seegtemp2>>10);

			  DataPacketHeader[0] = 0x0A;
			  DataPacketHeader[1] = 0xFA;
			  DataPacketHeader[2] = 0x08;
			  DataPacketHeader[3] = 0;
			  DataPacketHeader[4] = 0x02;


			  DataPacketHeader[5] = seegtemp;
			  DataPacketHeader[6] = seegtemp>>8;
			  DataPacketHeader[7] = seegtemp>>16;
			  DataPacketHeader[8] = seegtemp>>24;

			  DataPacketHeader[9] = seegtemp2;
			  DataPacketHeader[10] = seegtemp2>>8;
			  DataPacketHeader[11] = seegtemp2>>16;
			  DataPacketHeader[12] = seegtemp2>>24;

			  DataPacketHeader[13] = 0x00;
			  DataPacketHeader[14] = 0x0b;

			  for(i=0; i<15; i++){ // transmit the data
				  printf("%c",DataPacketHeader[i]);
			  }
			  Responsebyte = 0;
			  DRDY_flag = 0;
			  P2IE |= BIT3;								//Enable 2.3's Interrupt
		  }
	}
}

#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{}

#pragma vector=PORT2_VECTOR
__interrupt void PORT2(void)
{
	if((P2IFG&0x08)==0x08){
		DRDY_flag = 1;
		P2IFG &= ~BIT3;							//Clear the Interrupt Flag
	}
	else{
		P2IFG &= 0x00;							//Clear the whole interrupt flag
	}
}




void AFE4490Write (unsigned char address, unsigned long data) {
	 unsigned char dummy;

	P4OUT&= ~0x01;

	UCB1TXBUF = address;
	while ( (UCB1STAT & UCBUSY) );
	dummy = UCB1RXBUF;

	UCB1TXBUF = (unsigned char)(data >>16);
	while ( (UCB1STAT & UCBUSY) );
	dummy = UCB1RXBUF;

	UCB1TXBUF = (unsigned char)(((data & 0x00FFFF) >>8));
	while ( (UCB1STAT & UCBUSY) );
	dummy = UCB1RXBUF;

	UCB1TXBUF = (unsigned char)(((data & 0x0000FF)));
	while ( (UCB1STAT & UCBUSY) );
	dummy = UCB1RXBUF;

	P4OUT|=0x01;
}

unsigned long AFE4490Read (unsigned char address)
{
	unsigned char dummy;
	unsigned long data, read1, read2, read3;
	data = 0;

	P4OUT&= ~0x01;

	UCB1TXBUF = address;                   //address transmission
	while ( (UCB1STAT & UCBUSY) );
	dummy = UCB1RXBUF;

	UCB1TXBUF = 0;                         //largest 8 bits recieve
	while ( (UCB1STAT & UCBUSY) );
	read1 = UCB1RXBUF;

	UCB1TXBUF = 0;                         //middle 8 bits recieve
	while ( (UCB1STAT & UCBUSY) );
	read2 = UCB1RXBUF;

	UCB1TXBUF = 0;                        //smallest 8 bits recieve
	while ( (UCB1STAT & UCBUSY) );
	read3 = UCB1RXBUF;

	P4OUT|=0x01;

	data = (read1<<16)|(read2<<8)|(read3);
	return 	data;
}

void AFE4490Init(void) {
	//These initializations were taken from the Open Source Arduino project.
	AFE4490Write(CONTROL0,0x000000);

	AFE4490Write(CONTROL0,0x000008);

	AFE4490Write(TIAGAIN,0x000000); // CF = 5pF, RF = 500kR
	AFE4490Write(TIA_AMB_GAIN,0x000001);

	AFE4490Write(LEDCNTRL,0x001414);
	AFE4490Write(CONTROL2,0x000000); // LED_RANGE=100mA, LED=50mA
	AFE4490Write(CONTROL1,0x010707); // Timers ON, average 3 samples

	AFE4490Write(PRPCOUNT, 0X001F3F);

	AFE4490Write(LED2STC, 0X001770); //timer control
	AFE4490Write(LED2ENDC,0X001F3E); //timer control
	AFE4490Write(LED2LEDSTC,0X001770); //timer control
	AFE4490Write(LED2LEDENDC,0X001F3F); //timer control
	AFE4490Write(ALED2STC, 0X000000); //timer control
	AFE4490Write(ALED2ENDC, 0X0007CE); //timer control
	AFE4490Write(LED2CONVST,0X000002); //timer control
	AFE4490Write(LED2CONVEND, 0X0007CF); //timer control
	AFE4490Write(ALED2CONVST, 0X0007D2); //timer control
	AFE4490Write(ALED2CONVEND,0X000F9F); //timer control

	AFE4490Write(LED1STC, 0X0007D0); //timer control
	AFE4490Write(LED1ENDC, 0X000F9E); //timer control
	AFE4490Write(LED1LEDSTC, 0X0007D0); //timer control
	AFE4490Write(LED1LEDENDC, 0X000F9F); //timer control
	AFE4490Write(ALED1STC, 0X000FA0); //timer control
	AFE4490Write(ALED1ENDC, 0X00176E); //timer control
	AFE4490Write(LED1CONVST, 0X000FA2); //timer control
	AFE4490Write(LED1CONVEND, 0X00176F); //timer control
	AFE4490Write(ALED1CONVST, 0X001772); //timer control
	AFE4490Write(ALED1CONVEND, 0X001F3F); //timer control

	AFE4490Write(ADCRSTCNT0, 0X000000); //timer control
	AFE4490Write(ADCRSTENDCT0,0X000000); //timer control
	AFE4490Write(ADCRSTCNT1, 0X0007D0); //timer control
	AFE4490Write(ADCRSTENDCT1, 0X0007D0); //timer control
	AFE4490Write(ADCRSTCNT2, 0X000FA0); //timer control
	AFE4490Write(ADCRSTENDCT2, 0X000FA0); //timer control
	AFE4490Write(ADCRSTCNT3, 0X001770); //timer control
	AFE4490Write(ADCRSTENDCT3, 0X001770);
}
