; ***********************************************************************************
;
;    Operate an 4x4 KeyPad
;    Detect key hit and flash LEDs accordingly.
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



; **********************************
;  P O R T S   A N D   P I N S
; **********************************

; Green LED pin
.equ pGreenLedDirD                  = DDRC
.equ pGreenLedDirDBit               = DDC3
.equ pGreenLedPort                  = PORTC
.equ pGreenLedPortBit               = PORTC3
.equ pGreenLedPin                   = PINC
.equ pGreenLedPinBit                = PINC3

; Red LED pin
.equ pRedLedDirD                    = DDRC
.equ pRedLedDirDBit                 = DDC2
.equ pRedLedPort                    = PORTC
.equ pRedLedPortBit                 = PORTC2
.equ pRedLedPin                     = PINC
.equ pRedLedPinBit                  = PINC2

; Interrupt pin
.equ pInt0DirD                      = DDRD
.equ pInt0DirDBit                   = DDD2
.equ pInt0Port                      = PORTD
.equ pInt0PortBit                   = PORTD2
.equ pInt0Pin                       = PIND
.equ pInt0PinBit                    = PIND2


; Keypad uses D4-D7 (columns) and B0-3 (rows) an D2 for INT0

; Keypad row pins are Port B pins 0-3
.equ pRowDirD                       = DDRB
.equ pRowPort                       = PORTB
.equ pRowPin                        = PINB
.equ kRowBitsOnes                   = 0x0F
.equ kRowBitsZeros                  = 0xF0
; Keypad row pin bits
.equ kRow1                          = 3
.equ kRow2                          = 2
.equ kRow3                          = 1
.equ kRow4                          = 0

; Keypad column pins are Port D pins 4-7
.equ pColDirD                       = DDRD
.equ pColPort                       = PORTD
.equ pColPin                        = PIND
.equ kColBitsOnes                   = 0xF0
.equ kColBitsZeros                  = 0x0F
; Keypad columns pin bits
.equ kCol1                          = 7
.equ kCol2                          = 6
.equ kCol3                          = 5
.equ kCol4                          = 4




; **********************************
;  C O N S T A N T S
; **********************************




; ***************************************
;  R E G I S T E R  P O L I C Y
; ***************************************

.def rScratch1                      = r2        ; Scratch (low) register
.def rScratch2                      = r3        ; Scratch (low) register

.def rBinWordL                      = r4        ; Argument for ASCII conversion
.def rBinWordH                      = r5        ; Argument for ASCII conversion

.def rLoop1                         = r14       ; Loop counter

.def rSREG                          = r15       ; Save/Restore status port

.def rTmp1                          = r16       ; Multipurpose register
.def rTmp2                          = r17       ; Multipurpose register
.def rDWMSOuter                     = r16       ; Subroutine delayMilliSeconds

.def rKey                           = r18       ; Index of key hit, used to look value in Key Table

.def rArgByte0                      = r24       ; For now using C register conventions for function calls
.def rArgByte1                      = r25       ; Second byte arg, or high byte of word arg

.def rDelayUsL                      = r24       ; Subroutine delayMicroSeconds
.def rDelayUsH                      = r25       ; Subroutine delayMicroSeconds
.def rMillisL                       = r24       ; Subroutine delayMilliSeconds
.def rMillisH                       = r25       ; Subroutine delayMilliSeconds
.def r10ths                         = r24       ; Subroutine delayTenthsOfSeconds
.def rDTSOuter                      = r25       ; Subroutine delayTenthsOfSeconds

.def rDWMSInnerL                    = r26       ; Subroutine delayMilliSeconds
.def rDWMSInnerH                    = r27       ; Subroutine delayMilliSeconds
.def rDTSInnerL                     = r26       ; Subroutine delayTenthsOfSeconds
.def rDTSInnerH                     = r27       ; Subroutine delayTenthsOfSeconds



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

; Arguments:  @0 = register base name, @1 = 16-bit constant
.macro ldiw

   ldi @0H, High( @1 )
   ldi @0L, Low( @1 )

.endm



; **********************************
;  M A C R O
; **********************************

; Arguments:  @0 = number of microseconds to delay (16-bit word)
.macro delayMicroSecondsM
    ldi rArgByte1, High( @0 )
    ldi rArgByte0, Low( @0 )
    call delayMicroSeconds
.endm



; **********************************
;  M A C R O
; **********************************

; Arguments:  @0 = number of milliseconds to delay (word value)
.macro delayMilliSecondsM
    ldi rArgByte1, High( @0 )
    ldi rArgByte0, Low( @0 )
    call delayMilliSeconds
.endm



; **********************************
;  M A C R O
; **********************************

; Arguments:  @0 = number of tenths of seconds to delay (byte value)
.macro delayTenthsOfSecondsM
    ldi rArgByte0, Low( @0 )
    call delayTenthsOfSeconds
.endm




; **********************************
;  D A T A   S E G M E N T
;        ( S R A M )
; **********************************

.dseg
.org SRAM_START


sStaticDataBegin:

    sKeyPadTable:
        .byte 16

sStaticDataEnd:




; **********************************
;  C O D E  S E G M E N T
; **********************************

.cseg
.org 0x00




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
	reti                       ; OC1A
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

; Rem: data in codeseg stored and addressed by words (not bytes)

dStaticDataBegin:

; Look up table for key conversion
.db 1, 2, 3, 15, 4, 5, 6, 14, 7, 8, 9, 13, 10, 0, 11, 12

dStaticDataEnd:

.equ kdStaticDataLen = 2 * ( dStaticDataEnd - dStaticDataBegin )



; ***************************************
;  I N T E R R U P T   H A N D L E R S
; ***************************************







; ***************************************
;  M A I N   ( R E S E T )
; ***************************************

main:

    initializeStack rTmp1

    rcall initStaticData                        ; Move static data from PROGMEM to SRAM

    ; Initialize LEDs
    sbi pGreenLedDirD, pGreenLedDirDBit
    cbi pGreenLedPort, pGreenLedPortBit
    sbi pRedLedDirD, pRedLedDirDBit
    cbi pRedLedPort, pRedLedPortBit

    ; Flash the LEDs
    sbi pGreenLedPort, pGreenLedPortBit
    sbi pRedLedPort, pRedLedPortBit
    delayTenthsOfSecondsM 20
    cbi pGreenLedPort, pGreenLedPortBit
    cbi pRedLedPort, pRedLedPortBit

    ; Configure the keypad to accept inputs
    rcall doConfigureKeypad

    mainLoop:
        ; Look for rows to go low
        in rTmp1, pRowPin
        andi rTmp1, kRowBitsOnes
        cpi rTmp1, kRowBitsOnes
        breq mainLoop
            rcall doKeyHit
            rjmp mainLoop



; **********************************
;  S U B R O U T I N E
; **********************************

doKeyHit:

    rcall doScanKeyPad
    rcall doFlashLeds
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doFlashLeds:
    ldiw Z, sKeyPadTable                        ; Read number corresponding to key from SRAM
    add ZL, rKey
    clr rTmp2                                    ; Doesn't affect carry flag
    adc ZH, rTmp2
    ld rTmp2, Z                                 ; rTmp2 holds the value of the key
    tst rTmp2                                   ; Is it zero?
    breq flashZero                                   ; ...then flash the red LED

    flashGreenLed:                              ; Flash green LED 'rTmp2' times
        sbi pGreenLedPort, pGreenLedPortBit
        delayMilliSecondsM 250
        cbi pGreenLedPort, pGreenLedPortBit
        delayMilliSecondsM 300
        dec rTmp2
        brne flashGreenLed
    rjmp flashExit

flashZero:
    sbi pRedLedPort, pRedLedPortBit             ; "0" is a single long flash of red LED
    delayMilliSecondsM 2000
    cbi pRedLedPort, pRedLedPortBit

flashExit:
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doScanKeyPad:
    sbis pRowPin, kRow1                         ; Find row of keypress
    ldi rKey, 0                                 ; Set Row pointer
    sbis pRowPin, kRow2
    ldi rKey, 4
    sbis pRowPin, kRow3
    ldi rKey, 8
    sbis pRowPin, kRow4
    ldi rKey, 12

    ; To read the column value need to flip the configuration of rows & columns
    ; Reconfigure rows
    in rTmp1, pRowDirD                          ; Change Rows to outputs
    ori rTmp1, kRowBitsOnes
    out pRowDirD, rTmp1
    in rTmp1, pColDirD                          ; Change Columns to inputs
    andi rTmp1, kColBitsZeros
    out pColDirD, rTmp1
    ; Reconfigure columns
    in rTmp1, pRowPort                          ; Set Rows low
    andi rTmp1, kRowBitsZeros
    out pRowPort, rTmp1
    in rTmp1, pColPort                          ; Set pull-up resistors on Cols
    ori rTmp1, kColBitsOnes
    out pColPort, rTmp1

    delayMicroSecondsM 200                      ; Allow time for port to settle

    sbis pColPin, kCol1                         ; Find column of keypress
    ldi rTmp1, 0
    sbis pColPin, kCol2
    ldi rTmp1, 1
    sbis pColPin, kCol3
    ldi rTmp1, 2
    sbis pColPin, kCol4
    ldi rTmp1, 3

    add rKey, rTmp1                             ; Combine ROW and COL for pointer

    ; Re-initialize columns and rows
    rcall doConfigureKeypad

    ret



; **********************************
;  S U B R O U T I N E
; **********************************

initStaticData:

    ; Copy the static strings into SRAM

    ; Z             = pointer to program memory
    ; X             = pointer to SRAM
    ; rTmp1         = counter
    ; rScratch1     = transfer register

    ; Set up pointers to read from PROGMEM to SRAM
    ldi rTmp1, kdStaticDataLen
    ldiw Z, dStaticDataBegin << 1
    ldiw X, sStaticDataBegin
initStaticData_Loop:                               ; Actual transfer loop from PROGMEM to SRAM
        lpm rScratch1, Z+
        st X+, rScratch1
        dec rTmp1
        brne initStaticData_Loop

    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doConfigureKeypad:

    ; Configure the keybad to accept inputs

    ; rTmp1     = used as a scratch register

    ; Configure keypad column pins
    in rTmp1, pColDirD                          ; Set PD4-PD7, columns, as output (others unchanged)
    ori rTmp1, kColBitsOnes
    out pColDirD, rTmp1
    in rTmp1, pColPort                          ; Set PD4-PD7 as low
    andi rTmp1, kColBitsZeros
    out pColPort, rTmp1

    ; Configure keypad row pins
    in rTmp1, pRowDirD                          ; Set PB0-PB3, rows, as input
    andi rTmp1, kRowBitsZeros
    out pRowDirD, rTmp1
    in rTmp1, pRowPort                          ; Enable pull ups on PB0-PB3
    ori rTmp1, kRowBitsOnes
    out pRowPort, rTmp1

    delayMicroSecondsM 200                      ; Allow time for port to settle

    ret



; **********************************
;  S U B R O U T I N E
; **********************************

delayMicroSeconds:

    ; Register r25:24 is passed as parameter (the number of microseconds to delay)

    ; r24 = LSB microseconds to delay
    ; r25 = MSB microseconds to delay

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



;; **********************************
;  S U B R O U T I N E
; **********************************

delayMilliSeconds:

    ; Register r25:r24 (milliSecCounter) is passed as parameter

    ; r24 = number of milliseconds to count (comes in as argument)
    ;     = number of times to execute the outer+inner loops combined
    ; r25 = outer loop counter byte
    ; r26 = low byte of inner loop counter word
    ; r27 = high byte of inner loop counter word

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
            ldiw rDWMSInner, kDWMSInnerCount

            ; Top of inner loop
            DWMS_Loop3:

                ; Decrement and test inner loop
                sbiw rDWMSInnerL:rDWMSInnerL, 1
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
            ldiw rDTSInner, kDTSInnerCount

            ; Top of inner loop
            DTS_Loop3:
                ; Decrement and test inner loop
                sbiw rDTSInnerH:rDTSInnerL, 1
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
