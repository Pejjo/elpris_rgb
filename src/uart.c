/*
    \file   main.c
    \brief  Main file of the project.
    (c) 2020 Microchip Technology Inc. and its subsidiaries.
    Subject to your compliance with these terms, you may use Microchip software and any
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party
    license terms applicable to your use of third party software (including open source software) that
    may accompany Microchip software.
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
    FOR A PARTICULAR PURPOSE.
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
    SOFTWARE.
*/

#define USART3_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <uart.h>
#include <avr/interrupt.h>
#include "include/atmel_start_pins.h"
#include "include/LightweightRingBuff.h"

void USART3_init(void);
void USART3_sendChar(char c);
void USART3_sendString(char *str);
char USART3_readChar(void);

RingBuff_t inBuffer;
RingBuff_t outBuffer;

ISR(USART3_DRE_vect)
{
     // I need to handle this interrupt.
    
} 

ISR(USART3_RXC_vect)
{
    char x;
    x=USART3.RXDATAL;
    
    RingBuffer_Insert(&inBuffer, x);
    TCB1.CNT=0; // Reset for every byte
} 


void USART3_init(void)
{
    USART3.BAUD = (uint16_t)USART3_BAUD_RATE(BAUD_RATE);

    USART3.CTRLA = 0 << USART_ABEIE_bp    /* Auto-baud Error Interrupt Enable: disabled */
                   | 0 << USART_DREIE_bp  /* Data Register Empty Interrupt Enable: disabled */
                   | 0 << USART_LBME_bp   /* Loop-back Mode Enable: disabled */
                   | USART_RS485_OFF_gc   /* RS485 Mode disabled */
                   | 1 << USART_RXCIE_bp  /* Receive Complete Interrupt Enable: enabled */
                   | 0 << USART_RXSIE_bp  /* Receiver Start Frame Interrupt Enable: disabled */
                   | 0 << USART_TXCIE_bp; /* Transmit Complete Interrupt Enable: disabled */

    USART3.CTRLB = 0 << USART_MPCM_bp       /* Multi-processor Communication Mode: disabled */
                   | 0 << USART_ODME_bp     /* Open Drain Mode Enable: disabled */
                   | 1 << USART_RXEN_bp     /* Reciever enable: disabled */
                   | USART_RXMODE_NORMAL_gc /* Normal mode */
                   | 0 << USART_SFDEN_bp    /* Start Frame Detection Enable: disabled */
                   | 1 << USART_TXEN_bp;    /* Transmitter Enable: enabled */

    USART3.CTRLC = USART_CMODE_ASYNCHRONOUS_gc /* Asynchronous Mode */
             | USART_CHSIZE_8BIT_gc /* Character size: 8 bit */
             | USART_PMODE_DISABLED_gc /* No Parity */
             | USART_SBMODE_1BIT_gc; /* 1 stop bit */
	
	PORTB.DIR |= PIN0_bm;
	PORTB.DIR &= ~PIN1_bm;
    
    
	PORTA.DIR |= PIN2_bm;
    PORTB.DIR |= PIN2_bm;
    
    EVSYS.CHANNEL0=EVSYS_GENERATOR_PORT1_PIN1_gc; // Connect PB1 (RXD) to EV0
    EVSYS.USERTCB1=EVSYS_CHANNEL_CHANNEL0_gc;   // Connect EV0 to TCB1
    EVSYS.USEREVOUTB=EVSYS_CHANNEL_CHANNEL0_gc; // Event out on PA2
    TCB1.CTRLA=TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm | TCB_ASYNC_bm;
    TCB1.CTRLB=(1 << TCB_CCMPEN_bp) | TCB_CNTMODE_SINGLE_gc; 
    TCB1.EVCTRL=TCB_FILTER_bm  | (1 << TCB_EDGE_bp) | TCB_CAPTEI_bm;
    TCB1.CCMP=0x8000;
//    TCB1.INTCTRL=TCB_CAPT_bm;
    
}

void USART3_sendChar(char c)
{
    while (!(USART3.STATUS & USART_DREIF_bm))
    {
        ;    
    }
    USART3.TXDATAL = c;
}

void USART3_sendString(char *str)
{
    for(size_t i = 0; i < strlen(str); i++)
    {
        USART3_sendChar(str[i]);
    }
}


void executeCommand(char *command)
{
    if(strcmp(command, "ON") == 0)
    {
        LED_on();
        USART3_sendString("OK, LED ON.\r\n");
    }
    else if (strcmp(command, "OFF") == 0)
    {
        LED_off();
        USART3_sendString("OK, LED OFF.\r\n");
    } 
    else 
    {
        USART3_sendString("Type ON/OFF to control the LED.\r\n");
    }
}

int mmain(void)
{
    char command[MAX_COMMAND_LEN];
    uint8_t index = 0;
    char c;
	
	/* This delay invalidates the initial noise on the TX line, after device reset. */
    _delay_ms(10);
	
    USART3_init();
    LED_init();
	
    USART3_sendString("Type ON/OFF to control the LED.\r\n");
    
    while (1)
    {
        c = USART3_readChar();
        if(c != '\n' && c != '\r')
        {
            command[index++] = c;
            if(index > MAX_COMMAND_LEN)
            {
                index = 0;
            }
        }
        
        if(c == '\r')
        {
            command[index] = '\0';
            index = 0;
            executeCommand(command);
        }
    }
}
