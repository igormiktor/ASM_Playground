; ***********************************************************************************
;
;    Blink two LEDs (PB0 and PB1) on an ATmega328p at 1 Hz on opposite duty cycles
;    using timer-triggered interrupts to toggle the LEDs.
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
;  P I N - O U T
; ***************************************
;
; Package = 28-pin-PDIP
; 1: Reset
; 2: PD0 PCINT16 RXD0
; 3: PD1 TXD0 PCINT17
; 4: PD2 INT0 PCINT18
; 5: PD3 INT1 OC2B PCINT19
; 6: PD4 T0 XCK0 PCINT20
; 7: Vcc
; 8: Gnd
; 9: PB6 TOSC1 XTAL1 PCINT6
; 10: PB7 TOSC2 XTAL2 PCINT7
; 11: PD5 T1 OC0B PCINT21
; 12: PD6 AIN0 OC0A PCINT22
; 13: PD7 AIN1 PCINT23
; 14: PB0 ICP1 CLKO PCINT0
; 15: PB1 OC1A PCINT1
; 16: PB2 /SS OC1B PCINT2
; 17: PB3 MOSI OC2A PCINT3
; 18: PB4 MISO PCINT4
; 19: PB5 USCK PCINT5
; 20: AVCC
; 21: AREF
; 22: Gnd
; 23: PC0 ADC0 PCINT8
; 24: PC1 ADC1 PCINT9
; 25: PC2 ADC2 PCINT10
; 26: PC3 ADC3 PCINT11
; 27: PC4 ADC4 SDA PCINT12
; 28: PC5 ADC5 SCL PCINT13;



; **********************************
;  P O R T S   A N D   P I N S
; **********************************

; Red LED pin
.equ pRedLedDirD                    = DDRB
.equ pRedLedDirDBit                 = DDB0
.equ pRedLedPort                    = PORTB
.equ pRedLedPortBit                 = PORTB0
.equ pRedLedPin                     = PINB
.equ pRedLedPinBit                  = PINB0

; Green LED pin
.equ pGreenLedDirD                  = DDRB
.equ pGreenLedDirDBit               = DDB1
.equ pGreenLedPort                  = PORTB
.equ pGreenLedPortBit               = PORTB1
.equ pGreenLedPin                   = PINB
.equ pGreenLedPinBit                = PINB1



; ***************************************
;  R E G I S T E R  P O L I C Y
; ***************************************

; .def rSreg  = r15                             ; Save/Restore status port

.def rTmp           = r16                       ; Define multipurpose register
.def rArg           = r24                       ; Register for argument passing



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

    sbi pGreenLedPin, pGreenLedPinBit           ; Toggle green LED
    sbi pRedLedPin, pRedLedPinBit               ; Toggle red LED

    reti



; **********************************
;  M A I N   P R O G R A M
; **********************************

main:

    .equ kPauseTime     = 20                    ; Tenths of Seconds
    .equ kTimer1Top     = 62449                 ; "Top" counter value for 1Hz output with prescalar of 256 using Timer1

    initializeStack rTmp                        ; Set up the stack


    ; Set up and flash the LEDs before we start the synchronized blinking

    sbi pGreenLedDirD, pGreenLedDirDBit         ; Set pin connected to Green LED to output mode
    sbi pRedLedDirD, pRedLedDirDBit             ; Set pin connected to Red LED to output mode

    sbi pGreenLedPort, pGreenLedPortBit         ; Green LED high
    sbi pRedLedPort, pRedLedPortBit             ; Red LED high

    ldi rArg, kPauseTime
    rcall delayTenthsOfSeconds                  ; Pause (see both LEDs working)

    cbi pGreenLedPort, pGreenLedPortBit         ; Green LED low
    cbi pRedLedPort, pRedLedDirDBit             ; Red LED low

    ldi rArg, kPauseTime
    rcall delayTenthsOfSeconds                  ; Pause before we start blinking them

    sbi pRedLedPort, pRedLedPortBit             ; Red (PB0) LED high


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



; **********************************
;  S U B R O U T I N E
; **********************************

delayTenthsOfSeconds:

    ; Register r24 (tenthOfSecCounter) is passed as parameter
    ; r24 = number of tenths-of-seconds to count (comes in as argument)
    ;     = number of times to execute the outer+inner loops combined
    ; r25 = outer loop counter byte
    ; r26 = low byte of inner loop counter word
    ; r27 = high byte of inner loop counter word

    .def r10ths         = r24                   ; r24 = number of tenths-of-seconds to count (comes in as argument)
                                                ;     = number of times to execute the outer+inner loops combined
    .def rOuter         = r25                   ; r25 = outer loop counter byte
    .def rInnerL        = r26                   ; r26 = low byte of inner loop counter word
    .def rInnerH        = r27                   ; r27 = high byte of inner loop counter word

    ; Executing the following combination of inner and outer loop cycles takes almost precisely 0.1 seconds at 16 Mhz
    .equ kOuterCount    = 7
    .equ kInnerCount    = 57142



    ; Top of loop for number of tenths-of-seconds
    Loop1:
        ; Initialize outer loop (uses a byte counter and counts down)
        ldi rOuter, kOuterCount

        ; Top of outer loop
        Loop2:
            ; Initialze inner loop (uses a word counter and counts down)
            ldi rInnerL, Low( kInnerCount )
            ldi rInnerH, High( kInnerCount )

            ; Top of inner loop
            Loop3:
                ; Decrement and test inner loop
                sbiw rInnerL, 1
                brne Loop3
                ; Done with inner loop

            ; Decrement and test outer loop
            dec rOuter
            brne Loop2
            ; Done with outer loop

        ; Decrement and test tenth-of-second loop
        dec r10ths
        brne Loop1
        ; Done with the requested number of tenths-of-seconds

    ret
