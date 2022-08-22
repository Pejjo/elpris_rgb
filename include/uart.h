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

#ifndef UART_H
#define UART_H

#define F_CPU 10000000
#define BAUD_RATE 115200
#define MAX_COMMAND_LEN		8
#define INIT_DELAY			10	/* Delay to invalidate the after-reset noise on the PC0 pin (TX) */

#include <avr/io.h>
#include "include/LightweightRingBuff.h"

extern RingBuff_t inBuffer;
extern RingBuff_t outBuffer;

void USART3_init(void);
void USART3_sendChar(char c);
void USART3_sendString(char *str);
char USART3_readChar(void);

#endif
