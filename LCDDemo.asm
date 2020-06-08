; ***********************************************************************************
;
;    Operate an LCD
;    Display a greeting message on an LCD and maintain a 1-second counter display.
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

; Green LED pin
.equ pGreenLedDirD                  = DDRB
.equ pGreenLedDirDBit               = DDB1
.equ pGreenLedPort                  = PORTB
.equ pGreenLedPortBit               = PORTB1
.equ pGreenLedPin                   = PINB
.equ pGreenLedPinBit                = PINB1

; Red LED pin
.equ pRedLedDirD                    = DDRB
.equ pRedLedDirDBit                 = DDB0
.equ pRedLedPort                    = PORTB
.equ pRedLedPortBit                 = PORTB0
.equ pRedLedPin                     = PINB
.equ pRedLedPinBit                  = PINB0

; LCD enable pin
.equ pLcdEnableDirD                 = DDRD
.equ pLcdEnableDirDBit              = DDD2
.equ pLcdEnablePort                 = PORTD
.equ pLcdEnablePortBit              = PORTD2

; LCD data (register) select pin
.equ pLcdDataSelectDirD             = DDRD
.equ pLcdDataSelectDirDBit          = DDD3
.equ pLcdDataSelectPort             = PORTD
.equ pLcdDataSelectPortBit          = PORTD3

; LCD data 7 pin
.equ pLcdD7DirD                     = DDRC
.equ pLcdD7DirDBit                  = DDC3
.equ pLcdD7Port                     = PORTC
.equ pLcdD7PortBit                  = PORTC3

; LCD data 6 pin
.equ pLcdD6DirD                     = DDRC
.equ pLcdD6DirDBit                  = DDC2
.equ pLcdD6Port                     = PORTC
.equ pLcdD6PortBit                  = PORTC2

; LCD data 5 pin
.equ pLcdD5DirD                     = DDRC
.equ pLcdD5DirDBit                  = DDC1
.equ pLcdD5Port                     = PORTC
.equ pLcdD5PortBit                  = PORTC1

; LCD data 4 pin
.equ pLcdD4DirD                     = DDRC
.equ pLcdD4DirDBit                  = DDC0
.equ pLcdD4Port                     = PORTC
.equ pLcdD4PortBit                  = PORTC0



; **********************************
;  C O N S T A N T S
; **********************************

; String-related constants
.equ kDecimalDigits                 = 5
.equ kHexDigits                     = 6         ; Includes '0x' prefix

; LCD commands
.equ kLcdClearDisplay               = 0x01
.equ kLcdReturnHome                 = 0x02
.equ kLcdEntryModeSet               = 0x04
.equ kLcdDisplayControl             = 0x08
.equ kLcdCursorShift                = 0x10
.equ kLcdFunctionSet                = 0x20
.equ kLcdSetCgramAddr               = 0x40
.equ kLcdSetDdramAddr               = 0x80

; LCD flags for display entry mode
.equ kLcdEntryRight                 = 0x00
.equ kLcdEntryLeft                  = 0x02
.equ kLcdEntryShiftIncrement        = 0x01
.equ kLcdEntryShiftDecrement        = 0x00

; LCD flags for display on/off control
.equ kLcdDisplayOn                  = 0x04
.equ kLcdDisplayOff                 = 0x00
.equ kLcdCursorOn                   = 0x02
.equ kLcdCursorOff                  = 0x00
.equ kLcdBlinkOn                    = 0x01
.equ kLcdBlinkOff                   = 0x00

; LCD flags for display/cursor shift
.equ kLcdDisplayMove                = 0x08
.equ kLcdCursorMove                 = 0x00
.equ kLcdMoveRight                  = 0x04
.equ kLcdMoveLeft                   = 0x00

; LCD flags for function set
.equ kLcd8BitMode                   = 0x10
.equ kLcd4BitMode                   = 0x00
.equ kLcd2Line                      = 0x08
.equ kLcd1Line                      = 0x00
.equ kLcd5x10Dots                   = 0x04
.equ kLcd5x8Dots                    = 0x00

; Second row addressing offset
.equ kLcdSecondRowOffset            = 0x40



; ***************************************
;  R E G I S T E R  P O L I C Y
; ***************************************

.def rScratch1      = r2                        ; Scratch (low) register
.def rScratch2      = r3                        ; Scratch (low) register

.def rBinWordL      = r4                        ; Argument for ASCII conversion
.def rBinWordH      = r5                        ; Argument for ASCII conversion

.def rLoop1         = r14                       ; Loop counter

.def rSREG          = r15                       ; Save/Restore status port

.def rTmp1          = r16                       ; Multipurpose registera
.def rTmp2          = r17

.def rArgByte0      = r24                       ; For now using C register conventions for function calls
.def rArgByte1      = r25                       ; Second byte arg, or high byte of word arg

.def rCounterLSB    = r26                       ; LSB of word (16-bit) counter (XL)
.def rCounterMSB    = r27                       ; MSB of word (16-bit) counter (XH)



; **********************************
;  M A C R O
; **********************************

; Arguments:  @0 = tmp reg to use (upper half)
.macro initializeStack

    .ifdef SPH
        ldi @0, High( RAMEND )
        out SPH, @0                             ; Upper byte of stack pointer (always load high-byte first)
    .endif
    ldi @0, Low( RAMEND )
    out SPL, @0                                 ; Lower byte of stack pointer

.endm



; **********************************
;  M A C R O
; **********************************

; Arguments:  none
.macro clearLcd

    ldi rArgByte0, kLcdClearDisplay
    call sendCmdToLcd

.endm



; **********************************
;  D A T A   S E G M E N T
;        ( S R A M )
; **********************************
;
.dseg
.org SRAM_START

sDecNbrStr:
    .byte 5                                     ; Reserve for the output of convertBinWordToAscStr
sDecNbrStrEnd:
.equ ksDecNbrStrLen = sDecNbrStrEnd - sDecNbrStr

sHexNbrStr:
    .byte 2                                     ; Reserve for leading '0x'
sHexOutputStr:
    .byte 4                                     ; Reserve for outpout of convertBinWordToHexStr'
sHexNbrStrEnd:
.equ ksHexNbrStrEnd = sHexNbrStrEnd - sHexNbrStr

sGreetingStr:
    .byte 12                                    ; Reserve for greeting to display at start up
sGreetingStrEnd:
.equ ksGreetingStrEd = sGreetingStrEnd - sGreetingStr


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
;  D A T A   I N   C O D E S E G
; ***************************************

dGreeting:
    .db 'H', 'e', 'l', 'l', 'o', ' '
    .db 'W', 'o', 'r', 'l', 'd', '!'
dGreetingEnd:
.equ kdGreetingLen = 2 * ( dGreetingEnd - dGreeting )

d0xPrefix:
    .db '0', 'x'
d0xPrefixEnd:
.equ kd0xPrefixLen = 2 * ( d0xPrefixEnd - d0xPrefix )


; ***************************************
;  I N T E R R U P T  H A N D L E R
; ***************************************

; Timer1 Compare A (OCR1A) interrupt handler

hdlrTIM1_COMPA:

    in rSREG, SREG
    adiw rCounterMSB:rCounterLSB, 1             ; Increment 16-bit counter
    out SREG, rSREG
    set                                         ; Set T flag
    reti



; **********************************
;  M A I N   P R O G R A M
; **********************************

main:

    .equ kPauseTime     = 20                    ; Seconds
    .equ kCompATop      = 62449                 ; "Top" counter value for 1Hz output with prescalar of 256 using Timer1
    .equ kCompBTop      = 62449                 ; "Top" counter value for 1Hz output with prescalar of 256 using Timer1

    initializeStack rTmp1                       ; Set up the stack

    call initStaticData                         ; Move static data from PROGMEM to SRAM

    sbi pGreenLedDirD, pGreenLedDirDBit         ; Set pin connected to Green LED to output mode
    sbi pRedLedDirD, pRedLedDirDBit             ; Set pin connected to Red LED to output mode

    ; Configure the LCD pins for output
    sbi pLcdD4DirD, pLcdD4DirDBit
    sbi pLcdD5DirD, pLcdD5DirDBit
    sbi pLcdD6DirD, pLcdD6DirDBit
    sbi pLcdD7DirD, pLcdD7DirDBit
    sbi pLcdEnableDirD, pLcdEnableDirDBit
    sbi pLcdDataSelectDirD, pLcdDataSelectDirDBit

    ; Set LCD pins LOW
    cbi pLcdD4Port, pLcdD4PortBit
    cbi pLcdD5Port, pLcdD5PortBit
    cbi pLcdD6Port, pLcdD6PortBit
    cbi pLcdD7Port, pLcdD7PortBit
    cbi pLcdEnablePort, pLcdEnablePortBit
    cbi pLcdDataSelectPort, pLcdDataSelectPortBit

    ; Start the LCD display
    rcall initializeLcd

    sbi pGreenLedPort, pGreenLedPortBit         ; Green LED on
    sbi pRedLedPort, pRedLedPortBit             ; Red LED on

    ; Display the greeting on the screen
    clr rArgByte0
    clr rArgByte1
    call setLcdRowCol
    ldi ZH, High( sGreetingStr )
    ldi ZL, Low( sGreetingStr )                 ; Read the greeting out of SRAM
    ldi rTmp1, ( kdGreetingLen - 1 )
    mov rLoop1, rTmp1
displayGreetingLoop:
        ld rArgByte0, Z+
        rcall sendDataToLcd
        dec rLoop1
        brne displayGreetingLoop

    ; Pause (see both LEDs working)
    ldi rArgByte0, kPauseTime
    rcall delayTenthsOfSeconds

    cbi pGreenLedPort, pGreenLedPortBit         ; Green LED off
    cbi pRedLedPort, pRedLedPortBit             ; Red LED off

    ldi rArgByte0, kPauseTime
    rcall delayTenthsOfSeconds                  ; Pause before we start blinking them

    sbi pRedLedPort, pRedLedPortBit             ; Red (PB0) LED on

    ; Load the CompA/CompB "top" counter values, 16-bit value must be loaded high-byte first
    ldi rTmp1, High( kCompATop )                 ; Always load high byte first
    sts OCR1AH, rTmp1;
    ldi rTmp1, Low( kCompATop )                  ; And load low byte second
    sts OCR1AL, rTmp1;

    ; Set up Timer1 (CTC mode, prescalar=256, CompA = Top)
    ldi rTmp1, ( 1 << WGM12 ) | ( 1 << CS12 )    ; Select CTC mode with prescalar = 256
    sts TCCR1B, rTmp1

    ; Enable the CompA interrupt for Timer1 to generate a 1 sec heart beat
    ldi rTmp1, ( 1 << OCIE1A )                   ; Enable CompA interrupt
    sts TIMSK1, rTmp1;

    ; Ready to run...
    clt                                         ; Clear T flag
    sei                                         ; Enable interrupts

    mainLoop:
        brtc mainLoop

        ; Get here only if T flag set
        sbi pGreenLedPin, pGreenLedPinBit       ; Toggle green LED
        sbi pRedLedPin, pRedLedPinBit           ; Toggle red LED

        cli                                     ; Pause interrupts
        clt                                     ; Clear T flag

        ; Read the counter value to display on the LCD...
        mov rBinWordL, rCounterLSB
        mov rBinWordH, rCounterMSB

        sei                                     ; Restore interrupts

        ; Convert the value to an ASCII string and display
        ldi ZH, HIGH( sDecNbrStr )
        ldi ZL, LOW( sDecNbrStr )
        call convertBinWordToAscStr             ; Dec ASCII version in SRAM
        ldi rArgByte0, 1
        ldi rArgByte1, 1
        rcall setLcdRowCol
        ldi rTmp1, kDecimalDigits
        mov rLoop1, rTmp1
    displayDecNbrLoop:
            ld rArgByte0, Z+                    ; Read the value out of SRAM
            rcall sendDataToLcd
            dec rLoop1
            brne displayDecNbrLoop

        ; Convert the value to a hex string and display
        ldi ZH, HIGH( sHexOutputStr )
        ldi ZL, LOW( sHexOutputStr )
        call convertBinWordToHexStr             ; Hex ASCII version in SRAM
        ldi rArgByte0, 1
        ldi rArgByte1, 0x09
        rcall setLcdRowCol
        ldi ZH, HIGH( sHexNbrStr )              ; Display the string with prefix
        ldi ZL, LOW( sHexNbrStr )
        ldi rTmp1, kHexDigits
        mov rLoop1, rTmp1
    displayHexNbrLoop:
            ld rArgByte0, Z+                    ; Read the value out of SRAM
            rcall sendDataToLcd
            dec rLoop1
            brne displayHexNbrLoop

        rjmp mainLoop                           ; Go back to top of main loop



; **********************************
;  S U B R O U T I N E
; **********************************

initStaticData:

    ; Copy the static strings into SRAM

    ; Z             = pointer to program memory
    ; Y             = pinter to SRAM
    ; rTmp1         = counter
    ; rScratch1     = transfer register

    ; Copy greeting string
    ; Set up pointers to read from PROGMEM to SRAM
    ldi rTmp1, kdGreetingLen
    ldi ZH, HIGH( dGreeting << 1 )
    ldi ZL, LOW( dGreeting << 1 )
    ldi XH, HIGH( sGreetingStr )
    ldi XL, LOW( sGreetingStr )
initStaticData_1:                               ; Actual transfer loop from PROGMEM to SRAM
        lpm rScratch1, Z+
        st X+, rScratch1
        dec rTmp1
        brne initStaticData_1

    ; Copy hex prefix
    ; Set up pointers to read from PROGMEM to SRAM
    ldi rTmp1, kd0xPrefixLen
    ldi ZH, HIGH( d0xPrefix << 1 )
    ldi ZL, LOW( d0xPrefix << 1 )
    ldi XH, HIGH( sHexNbrStr )
    ldi XL, LOW( sHexNbrStr )
    initStaticData_2:                               ; Actual transfer loop from PROGMEM to SRAM
            lpm rScratch1, Z+
            st X+, rScratch1
            dec rTmp1
            brne initStaticData_2

    ret



; **********************************
;  S U B R O U T I N E
; **********************************

initializeLcd:

    ; Wait 50 milliseconds to ensure full voltage rise
    clr rArgByte1
    ldi rArgByte0, 50
    rcall delayMilliSeconds

    cbi pLcdDataSelectPort, pLcdDataSelectPortBit   ; Pull DS pin Low (sending commands)
    cbi pLcdEnablePort, pLcdEnablePortBit           ; Pull E pin Low

    ; Need to send 0x03 three times

    ; First time
    ldi rArgByte0, 0x03
    rcall write4BitsToLcd

    ; Wait > 4.1 ms
    ldi rArgByte1, HIGH( 4500 )
    ldi rArgByte0, LOW( 4500 )
    call delayMicroSeconds

    ; Second time
    ldi rArgByte0, 0x03
    rcall write4BitsToLcd

    ; Wait > 4.1 ms
    ldi rArgByte1, HIGH( 4500 )
    ldi rArgByte0, LOW( 4500 )
    call delayMicroSeconds

    ; Third try and go...
    ldi rArgByte0, 0x03
    rcall write4BitsToLcd

    ; Wait >150 us
    ldi rArgByte1, High( 200 )
    ldi rArgByte0, Low( 200 )
    call delayMicroSeconds

    ; This actually sets the 4-bit interface
    ldi rArgByte0, 0x02
    rcall write4BitsToLcd

    ; Set nbr of lines and font
    ldi rArgByte0, ( kLcdFunctionSet | kLcd2Line | kLcd5x8Dots )
    rcall sendCmdToLcd

    ; Turn on display, with cursor off and blinking off
    ldi rArgByte0, kLcdDisplayControl | kLcdDisplayOn | kLcdCursorOff | kLcdBlinkOff
    rcall sendCmdToLcd

    ; Set text entry more (L to R)
    ldi rArgByte0, kLcdEntryModeSet | kLcdEntryLeft
    rcall sendCmdToLcd

    ; Clear display
    ldi rArgByte0, kLcdClearDisplay
    rcall sendCmdToLcd
    ldi rArgByte1, High( 2000 )
    ldi rArgByte0, Low( 2000 )
    call delayMicroSeconds           ; Clear cmd takes a long time...

    ret



; **********************************
;  S U B R O U T I N E
; **********************************

write4BitsToLcd:

    ; Register rArgByte0 is passed as parameter (the 4-bits to write)

    ; rArgByte0 = the 4-bits to write to the LCD in lower nibble (modified)
    ; rTmp1 used as a temporary register

    ; First write the pins with the 4-bit value;
    ; The 4 pins are on the lower nibble of a single PORT
    andi rArgByte0, 0x0F                        ; Mask out just the lower nibble
    in rTmp1, pLcdD4Port
    andi rTmp1, 0xF0                            ; Save just the upper nibble of PORTC in @1
    or rArgByte0, rTmp1                         ; Combine upper nibble of PORTC with lower nibble value
    out pLcdD4Port, rArgByte0

    ; Now pulse the enable pin to have the LCD read the value
    cbi pLcdEnablePort, pLcdEnablePortBit       ; Enable pin LOW
    clr rArgByte1
    ldi rArgByte0, 2
    call delayMicroSeconds
    sbi pLcdEnablePort, pLcdEnablePortBit       ; Enable pin HIGH (actual enable pulse)
    ldi rArgByte0, 2
    call delayMicroSeconds                      ; Enable pulse must be > 450ns
    cbi pLcdEnablePort, pLcdEnablePortBit       ; Enable pin LOW
    ldi rArgByte1, HIGH( 100 )
    ldi rArgByte0, LOW( 100 )                   ; Seems like a lot but didn't work with 70us
    call delayMicroSeconds                      ; Command needs > 37us to settle

    ret



; **********************************
;  S U B R O U T I N E
; **********************************

sendDataToLcd:

    ; Register rArgByte0 is passed as parameter (the 4-bits to write)

    ; rArgByte0 = the 4-bits to write to the LCD in lower nibble (modified)
    ; rScratch1 used as a temporary register
    ; (rTmp1 used as a temporary by write4BitsToLcd)


    sbi pLcdDataSelectPort, pLcdDataSelectPortBit   ; Pin on to send data
    rjmp send8BitsToLcd

sendCmdToLcd:

    cbi pLcdDataSelectPort, pLcdDataSelectPortBit   ; Pin off to send command
    ; Intentional fall through

send8BitsToLcd:
    mov rScratch1, rArgByte0                        ; Save the value
    swap rArgByte0
    rcall write4BitsToLcd                           ; Send the upper nibble
    mov rArgByte0, rScratch1                        ; Restore the value
    rcall write4BitsToLcd                           ; Send the lower nibble

    ret



; **********************************
;  S U B R O U T I N E
; **********************************

setLcdRowCol:

    ; rArgByte0 and rArgByte1 passed as parameters (row, col)

    ; rArgByte0 = LCD row (0-1)
    ; rArgByte1 = LCD col (0-15)

    cpi rArgByte0, 0                            ; Compare row to 0
    breq NoOffsetRequired                       ; If row == 0, skip offset
    subi rArgByte1, -kLcdSecondRowOffset        ; Add the offset for second row to column
NoOffsetRequired:
    ori rArgByte1, kLcdSetDdramAddr             ; Incorporate the command itself
    mov rArgByte0, rArgByte1                    ; Move the cmd to rArgByte0
    rcall sendCmdToLcd

    ret



; **********************************
;  S U B R O U T I N E
; **********************************

delayMicroSeconds:

    ; Register r25:24 is passed as parameter (the number of microseconds to delay)

    ; r24 = LSB microseconds to delay
    ; r25 = MSB microseconds to delay

    .def rDelayUsL    = r24
    .def rDelayUsH    = r25

    ; 1 microsecond = 16 cycles.
    ; Call/return overhead takes 7-8 cycles (depending on rcall or call).

    ; So burn up 8 more cycles (not counting the ret) to make a whole microsecond, including
    ; a check to see if we are done (i.e., the request was a 1us delay).
    ; Then do a loop that burns 16 cycles each time

    nop                                 ; 1 cycle
    nop                                 ; 1 cycle
    nop                                 ; 1 cycle
    nop                                 ; 1 cycle
    sbiw rDelayUsH:rDelayUsL, 1         ; 2 cycles
    breq delayMicroseconds_Ret          ; 1 cycle if false/continue, 2 cycles (8 total) if true/branch
    nop                                 ; 1 cycle (8 total)

    delayMicroseconds_Loop:
        nop                             ; 1 cycle
        nop                             ; 1 cycle
        nop                             ; 1 cycle
        nop                             ; 1 cycle

        nop                             ; 1 cycle
        nop                             ; 1 cycle
        nop                             ; 1 cycle
        nop                             ; 1 cycle

        nop                             ; 1 cycle
        nop                             ; 1 cycle
        nop                             ; 1 cycle
        nop                             ; 1 cycle

        sbiw rDelayUsH:rDelayUsL, 1     ; 2 cycles
        brne delayMicroseconds_Loop     ; 2 cycles (16 total) on true/loop, 1 cycle on false/exit_loop
    nop                                 ; 1 cycle (so total 16 on exit from last loop)

delayMicroseconds_Ret:
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

delayMilliSeconds:

    ; Register r25:r24 (milliSecCounter) is passed as parameter

    ; r24 = number of milliseconds to count (comes in as argument)
    ;     = number of times to execute the outer+inner loops combined
    ; r25 = outer loop counter byte
    ; r26 = low byte of inner loop counter word
    ; r27 = high byte of inner loop counter word

    .def rMillisL           = r24
    .def rMillisH           = r25

    .def rDWMSOuter         = r16       ; ( = rTmp1 )
    .def rDWMSInnerL        = r26
    .def rDWMSInnerH        = r27

    ; Executing the following combination of inner and outer loop cycles takes almost precisely 1 millisecond at 16 MHz
    .equ kDWMSOuterCount    = 2
    .equ kDWMSInnerCount    = 1997

    ; Top of loop for number of milliseconds
    DWMS_Loop1:

        ; Initialize outer loop (uses a byte counter and counts down)
        ldi rDWMSOuter, kDWMSOuterCount

        ; Top of outer loop
        DWMS_Loop2:

            ; Initialze inner loop (uses a word counter and counts down)
            ldi rDWMSInnerL, Low( kDWMSInnerCount )
            ldi rDWMSInnerH, High( kDWMSInnerCount )

            ; Top of inner loop
            DWMS_Loop3:

                ; Decrement and test inner loop
                sbiw rDWMSInnerL, 1
                brne DWMS_Loop3
                ; Done with inner loop

            ; Decrement and test outer loop
            dec rDWMSOuter
            brne DWMS_Loop2
            ; Done with outer loop

        ; Decrement and test millisecond loop
        sbiw rMillisH:rMillisL, 1
        brne DWMS_Loop1
        ; Done with the requested number of milliseconds

    ret



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

    .def r10ths             = r24                   ; r24 = number of tenths-of-seconds to count (comes in as argument)
                                                    ;     = number of times to execute the outer+inner loops combined
    .def rDTSOuter          = r25                   ; r25 = outer loop counter byte
    .def rDTSInnerL         = r26                   ; r26 = low byte of inner loop counter word
    .def rDTSInnerH         = r27                   ; r27 = high byte of inner loop counter word

    ; Executing the following combination of inner and outer loop cycles takes almost precisely 0.1 seconds at 16 Mhz
    .equ kDTSOuterCount     = 7
    .equ kDTSInnerCount     = 57142

    ; Top of loop for number of tenths-of-seconds
    DTS_Loop1:
        ; Initialize outer loop (uses a byte counter and counts down)
        ldi rDTSOuter, kDTSOuterCount

        ; Top of outer loop
        DTS_Loop2:
            ; Initialze inner loop (uses a word counter and counts down)
            ldi rDTSInnerL, Low( kDTSInnerCount )
            ldi rDTSInnerH, High( kDTSInnerCount )

            ; Top of inner loop
            DTS_Loop3:
                ; Decrement and test inner loop
                sbiw rDTSInnerL, 1
                brne DTS_Loop3
                ; Done with inner loop

            ; Decrement and test outer loop
            dec rDTSOuter
            brne DTS_Loop2
            ; Done with outer loop

        ; Decrement and test tenth-of-second loop
        dec r10ths
        brne DTS_Loop1
        ; Done with the requested number of tenths-of-seconds

    ret



; **********************************
;  S U B R O U T I N E
; **********************************

convertBinWordToAscStr:

    ; Convert a 16-bit-binary to a 5 char decimal ASCII string

    ; Registers rBinWordH:rBinWordL and Z passed in as arguments
    ; Result returned in 5-bytes starting where Z points

    ; rBinWordH:rBinWordL   = 16-bit quantity to convert (not changed)
    ; Z                     = pointer to first (highest) digit of ASCII result
    ;                         (1 digit per byte, 5 bytes total, with leading spaces)
    ; rScratch2:rScratch1   = scratch registers sometimes used as a 16-bit quantity (changed)

	rcall convertBinWordToBcdArray     ; convert binary to BCD
	ldi rTmp1, 4                       ; Counter is 4 leading digits
	mov rScratch1, rTmp1

convertBinWordToAscStr1:
	ld rTmp1, Z                        ; read a BCD digit
	tst rTmp1                          ; check if leading zero
	brne convertBinWordToAscStr2       ; No, found digit >0
	ldi rTmp1, ' '                     ; overwrite with blank
	st Z+, rTmp1                       ; store and set to next position
	dec rScratch1                      ; decrement counter
	brne convertBinWordToAscStr1       ; further leading blanks
	ld rTmp1, Z                        ; Read the last BCD

convertBinWordToAscStr2:
	inc rScratch1                      ; one more char

convertBinWordToAscStr3:
	subi rTmp1, -'0'                   ; Add '0'
	st Z+, rTmp1                       ; store and inc pointer
	ld rTmp1, Z                        ; read next char
	dec rScratch1                      ; more chars?
	brne convertBinWordToAscStr3       ; yes, go on
	sbiw ZL, 5                         ; Pointer to beginning of the BCD
	ret



; **********************************
;  S U B R O U T I N E
; **********************************

convertBinWordToBcdArray:

    ; Convert 16-bit-binary to a 5-digit-BCD array

    ; Registers rBinWordH:rBinWordL and Z passed in as arguments
    ; Result returned in 5-bytes starting where Z points

    ; rBinWordH:rBinWordL   = 16-bit quantity to convert (not changed)
    ; Z                     = pointer to first (highest) digit of BCD result
    ;                         (1 digit per byte, 5 bytes total, with leading zeros)
    ; rScratch2:rScratch1   = scratch registers sometimes used as a 16-bit quantity (changed)

	push rBinWordH                     ; Save number
	push rBinWordL

	ldi rTmp1, HIGH( 10000 )           ; Start with ten thousands
	mov rScratch2, rTmp1
	ldi rTmp1, LOW( 10000 )
	mov rScratch1, rTmp1
	rcall getOneBinWordDecDigit        ; Calculate digit

	ldi rTmp1, HIGH( 1000 )            ; Next with thousands
	mov rScratch2, rTmp1
	ldi rTmp1, LOW( 1000 )
	mov rScratch1, rTmp1
	rcall getOneBinWordDecDigit        ; Calculate digit

	ldi rTmp1, HIGH( 100 )             ; Next with hundreds
	mov rScratch2, rTmp1
	ldi rTmp1, LOW( 100 )
	mov rScratch1, rTmp1
	rcall getOneBinWordDecDigit        ; Calculate digit

	ldi rTmp1, HIGH( 10 )              ; Next with tens
	mov rScratch2, rTmp1
	ldi rTmp1, LOW( 10 )
	mov rScratch1, rTmp1
	rcall getOneBinWordDecDigit        ; Calculate digit

	st Z,rBinWordL                     ; Remainder are ones
	sbiw ZL, 4                         ; Set pointer to first BCD

	pop rBinWordL                      ; Restore original binary
	pop rBinWordH
	ret



; **********************************
;  S U B R O U T I N E
; **********************************

getOneBinWordDecDigit:

    ; Determine one decimal digit by continued subtraction of a binary decimal value

    ; Registers rBinWordH:rBinWordL, rScratch2:rScratch1, and Z passed in as arguments
    ; Result returned where Z points; Z incremented, rBinWordH:rBinWordL contains remainder

    ; rBinWordH:rBinWordL   = 16-bit quantity to be decimated (changed)
    ; Z                     = pointer to store resulting BCD digit  (changed)
    ; rScratch2:rScratch1   = 16-bit binary decimal value (unchanged)

	clr rTmp1                          ; digit count is zero

getOneBinWordDecDigit1:
	cp rBinWordH, rScratch2            ; Number bigger than decimal?
	brcs getOneBinWordDecDigit3        ; MSB smaller than decimal -> done (digit = 0)
	brne getOneBinWordDecDigit2        ; MSB bigger than decimal
	cp rBinWordL, rScratch1            ; LSB bigger or equal decimal
	brcs getOneBinWordDecDigit3        ; LSB smaller than decimal -> done (digit = 0)

getOneBinWordDecDigit2:
	sub rBinWordL, rScratch1           ; Subtract LSB decimal
	sbc rBinWordH, rScratch2           ; Subtract MSB decimal
	inc rTmp1                          ; Increment digit count
	rjmp getOneBinWordDecDigit1        ; Next loop -> try to subtract again

getOneBinWordDecDigit3:
	st Z+, rTmp1                       ; Save digit and increment
	ret





; **********************************
;  S U B R O U T I N E
; **********************************

convertBinWordToHexStr:

; Convert a 16-bit-binary to a 4 char hex ASCII string

; Registers rBinWordH:rBinWordL and Z passed in as arguments
; Result returned in 4 bytes starting where Z points

; rBinWordH:rBinWordL   = 16-bit quantity to convert (not changed)
; Z                     = pointer to first (highest) digit of ASCII result
;                         (1 digit per byte, 4 bytes total, with leading zeros)
; rTmp1                 = temp (upper) register (changed)

	mov rTmp1, rBinWordH                           ; Load MSB
	rcall Bin1ToHex2                               ; Convert byte
	mov rTmp1, rBinWordL                           ; Repeat on LSB
	rcall Bin1ToHex2
	sbiw ZL, 4                                     ; Reset Z to start

	ret



; **********************************
;  S U B R O U T I N E
; **********************************

Bin1ToHex2:

; Convert an 8-bit-binary to 2-char uppercase hex string

; Registers rBinWordH:rBinWordL and Z passed in as arguments
; Result returned in 4 bytes starting where Z points

; rBinWordH:rBinWordL   = 16-bit quantity to convert (not changed)
; Z                     = pointer to first (highest) digit of ASCII result
;                         (1 digit per byte, 4 bytes total, with leading zeros)
; rTmp1                 = temp (upper) register (changed)

	push rTmp1                                     ; Save byte
	swap rTmp1                                     ; Move Upper to lower nibble
	rcall Bin1ToHex1                               ; Convenience to make this a function all
	pop rTmp1                                      ; Restore byte and fall through (instead of another rcall)

Bin1ToHex1:
	andi rTmp1, 0x0F                               ; Mask upper nibble
	subi rTmp1, -'0'                               ; Add 0 to convert to ASCII 0-9
	cpi rTmp1, '9' + 1                             ; Is it A..F?
	brcs Bin1ToHex1a
	subi rTmp1, -7                                 ; Add 7 for A..F
Bin1ToHex1a:
	st z+, rTmp1                                   ; Store a hex digit

	ret
