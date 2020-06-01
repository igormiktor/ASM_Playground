; ***********************************************************************************
;
;    Blink LED PB0 on an ATmega328p at 1 Hz using timer-triggered interrupts
;    to toggle the LED.  Assume ATmega328p runs at 16 MHz.
;
;    The MIT License (MIT)
;
;    Copyright (c) 2020 Igor Mikolic-Torreira
;
;    Permission is hereby granted, free of charge, to any person obtaining a copy
;    of this software and associated documentation files (the "Software"), to deal
;    in the Software without restriction, including without limitation the rights
;    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
;    copies of the Software, and to permit persons to whom the Software is
;    furnished to do so, subject to the following conditions:
;
;    The above copyright notice and this permission notice shall be included in all
;    copies or substantial portions of the Software.
;
;    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
;    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
;    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
;    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
;    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
;    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
;    SOFTWARE.
;
; ***********************************************************************************



.device "ATmega328p"



; ***************************************
;  R E G I S T E R  P O L I C Y
; ***************************************

.def rTmp   = r16                               ; Define multipurpose register



; **********************************
;  M A C R O S
; **********************************

.macro initializeStack                          ; Takes 1 argument, @0 = register to use

    .ifdef SPH                                  ; if SPH is defined
        ldi @0, High( RAMEND )
        out SPH, @0                             ; Upper byte of stack pointer (always load high-byte first)
    .endif

    ldi @0, Low( RAMEND )
    out SPL, @0                                 ; Lower byte of stack pointer

.endm



; **********************************
;  C O D E  S E G M E N T
; **********************************

.cseg
.org 0



; ************************************
;  I N T E R R U P T  V E C T O R S
; ************************************

.org 0x00
	rjmp main                  ; Reset vector
.org 0x02
	reti                       ; INT0
.org 0x04
	reti                       ; INT1
.org 0x06
	reti                       ; PCI0
.org 0x08
	reti                       ; PCI1
.org 0x0A
	reti                       ; PCI2
.org 0x0C
	reti                       ; WDT
.org 0x0E
	reti                       ; OC2A
.org 0x10
	reti                       ; OC2B
.org 0x12
	reti                       ; OVF2
.org 0x14
	reti                       ; ICP1
.org 0x16
	rjmp hdlrTIM1_COMPA        ; OC1A
.org 0x18
	reti                       ; OC1B
.org 0x1A
	reti                       ; OVF1
.org 0x1C
	reti                       ; OC0A
.org 0x1E
	reti                       ; OC0B
.org 0x20
	reti                       ; OVF0
.org 0x22
	reti                       ; SPI
.org 0x24
	reti                       ; URXC
.org 0x26
	reti                       ; UDRE
.org 0x28
	reti                       ; UTXC
.org 0x2A
	reti                       ; ADCC
.org 0x2C
	reti                       ; ERDY
.org 0x2E
	reti                       ; ACI
.org 0x30
	reti                       ; TWI
.org 0x32
	reti                       ; SPMR
.org 0x34



; ***************************************
;  I N T E R R U P T  H A N D L E R S
; ***************************************

hdlrTIM1_COMPA:

    ; Nothing in this interrupt affects SREG, so no need to save it

    ; Toggle pins.  Note that PORTx bits can be toggled by writing a 1 to the corresponding PINx bit.

    sbi PINB, PINB0                             ; Toggle the LED

    reti



; **********************************
;  M A I N   P R O G R A M
; **********************************

main:

    .equ kTimer1Top     = 62449                 ; "Top" counter value for 1Hz output with prescalar of 256 using Timer1

    initializeStack rTmp                        ; Set up the stack

    sbi DDRB, DDB0                              ; Set pin connected to LED to output mode

    sbi PORTB, PORTB0                           ; Set LED high

    ; Set up Timer1 (CTC mode, prescalar=256, CompA interrupt on)

    ldi rTmp, ( 1 << WGM12 ) | ( 1 << CS12 )    ; Select CTC mode with prescalar = 256
    sts TCCR1B, rTmp;

    ; Load the CompA "top" counter value, 16-bit value must be loaded high-byte first

    ldi rTmp, High( kTimer1Top )                ; Always load high byte first
    sts OCR1AH, rTmp;
    ldi rTmp, Low( kTimer1Top )                 ; And load low byte second
    sts OCR1AL, rTmp;

    ; Enable the CompA interrupt for Timer1

    ldi rTmp, ( 1 << OCIE1A )                   ; Enable CompA interrupt
    sts TIMSK1, rTmp;

    sei                                         ; Enable interrupts


    loopMain:
        rjmp loopMain                           ; infinite loop
