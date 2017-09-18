/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <xc.h>
#include "board.h"

/********************************************************************/
// CPU

void cpu_init(void) {
    // set oscillator to operate at 40MHz
    // Fosc = Fin*M/(N1*N2), Fcy = Fosc/2
    // Fosc = 7.37M*40/(2*2) = 80Mhz for 7.37M input clock
    PLLFBD = 41;            // M=39
    CLKDIVbits.PLLPOST = 0; // N1=2
    CLKDIVbits.PLLPRE = 0;  // N2=2
    OSCTUN = 0;

    // initiate clock switch to FRC with PLL
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(0x01);

    // wait for clock switch to occur
    while (OSCCONbits.COSC != 0x01) {
    }
    while (!OSCCONbits.LOCK) {
    }
}

/********************************************************************/
// LEDs

#define LED_TRIS TRISBbits.TRISB11


#define LED LATBbits.LATB11

#define LED_ON (0)
#define LED_OFF (1)

void led_init(void) {
    // set led GPIO as outputs
    LED_TRIS = 0;

    // turn off the LEDs
    LED = LED_ON;

}

void led_state(int state) {
    int val = state ? LED_ON : LED_OFF;
    LED = val;
}

void led_toggle(void) {
    LED ^= 1;
}

/********************************************************************/
// switches

#define SWITCH_S1_TRIS TRISAbits.TRISA2
#define SWITCH_S2_TRIS TRISAbits.TRISA3

#define SWITCH_S1 PORTAbits.RA2
#define SWITCH_S2 PORTAbits.RA3

void switch_init(void) {
    // set switch GPIO as inputs
    SWITCH_S1_TRIS = 1;
    SWITCH_S2_TRIS = 1;
}

int switch_get(int sw) {
    int val = 1;
    switch (sw) {
        case 1: val = SWITCH_S1; break;
        case 2: val = SWITCH_S2; break;
    }
    return val == 0;
}

/********************************************************************/
// UART

/*
// TODO need an irq
void uart_rx_irq(void) {
    if (c == interrupt_char) {
        MP_STATE_VM(mp_pending_exception) = MP_STATE_PORT(keyboard_interrupt_obj);
    }
}
*/

void uart_init(void) {
    // connect remappable pins:
    TRISBbits.TRISB4 = 1; // Make this an input
    RPINR18 = 36; // U1RX -> RP36
    RPOR0 = 1; // U1TX -> RP20
    // baudrate = F_CY / 16 (uxbrg + 1)
    // F_CY = 40MHz for us
    UART1.uxbrg = 64; // 38400 baud
    UART1.uxmode = 1 << 15; // UARTEN
    UART1.uxsta = 1 << 10;  // UTXEN
}

int uart_rx_any(void) {
    return UART1.uxsta & 1; // URXDA
}

int uart_rx_char(void) {
    return UART1.uxrxreg;
}

void uart_tx_char(int chr) {
    while (UART1.uxsta & (1 << 9)) {
        // tx fifo is full
    }
    UART1.uxtxreg = chr;
}
