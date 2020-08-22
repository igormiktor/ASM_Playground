; ***********************************************************************************
;
;    Operate an 4x4 KeyPad and display on LCD
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
; 2: PD0 RXD0 PCINT16
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
.equ pRedLedDirD                    = DDRD
.equ pRedLedDirDBit                 = DDD2
.equ pRedLedPort                    = PORTD
.equ pRedLedPortBit                 = PORTD2
.equ pRedLedPin                     = PIND
.equ pRedLedPinBit                  = PIND2

; Green LED pin
.equ pGreenLedDirD                  = DDRD
.equ pGreenLedDirDBit               = DDD3
.equ pGreenLedPort                  = PORTD
.equ pGreenLedPortBit               = PORTD3
.equ pGreenLedPin                   = PIND
.equ pGreenLedPinBit                = PIND3


; LCD uses pins PC0-PC5

; LCD data (register) select pin
.equ pLcdDataSelectDirD             = DDRC
.equ pLcdDataSelectDirDBit          = DDC5
.equ pLcdDataSelectPort             = PORTC
.equ pLcdDataSelectPortBit          = PORTC5

; LCD enable pin
.equ pLcdEnableDirD                 = DDRC
.equ pLcdEnableDirDBit              = DDC4
.equ pLcdEnablePort                 = PORTC
.equ pLcdEnablePortBit              = PORTC4

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

; Message and display related lengths
.equ kDisplayMsgLen                 = 16
.equ kDecimalDigitsLen              = 5

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



;; ***************************************
;  R E G I S T E R  P O L I C Y
; ***************************************

.def rScratch1                      = r2        ; Scratch (low) register
.def rScratch2                      = r3        ; Scratch (low) register

.def rBinWordL                      = r4        ; Argument for ASCII conversion
.def rBinWordH                      = r5        ; Argument for ASCII conversion

.def rWorkingNbrL                   = r12       ; Assemble a 16 bit number from keypad entry here (low byte)
.def rWorkingNbrH                   = r13       ; Assemble a 16 bit number from keypad entry here (high byte)

.def rLoop1                         = r14       ; Loop counter

.def rSREG                          = r15       ; Save/Restore status port

.def rDWMSOuter                     = r16       ; Subroutine delayMilliSeconds
.def rTmp1                          = r16       ; Multipurpose register
.def rTmp2                          = r17       ; Multipurpose register

.def rKey                           = r18       ; Index of key hit, used to look-up value in Key Table

.def rState                         = r19       ; State of operation
.equ kDigitEntryBit                 = 0x01      ; Bit 0 set = digit entry mode; clear = not digit entry mode
.equ kDigitEntryBitNbr              = 0         ; Bit number 0
.equ kNbrSignBit                    = 0x02      ; Bit 1 set = number has negative sign; clear = no negative sign
.equ kNbrSignBitNbr                 = 1         ; Bit number 1


.def rArgByte0                      = r24       ; First byte arg, or low byte of word arg
.def rArgByte1                      = r25       ; Second byte arg, or high byte of word arg

.def rDelayUsL                      = r24       ; Subroutine delayMicroSeconds
.def rDelayUsH                      = r25       ; Subroutine delayMicroSeconds
.def rMillisL                       = r24       ; Subroutine delayMilliSeconds
.def rMillisH                       = r25       ; Subroutine delayMilliSeconds
.def r10ths                         = r24       ; Subroutine delayTenthsOfSeconds
.def rDTSOuter                      = r25       ; Subroutine delayTenthsOfSeconds

.def rArgByte2                      = r26       ; Third byte of dword (32 bit) argument
.def rArgByte3                      = r27       ; Fourth byte of dword (32 bit) argument
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
;  M A C R O
; **********************************

; Arguments:  none
.macro clearLcd

    ldi rArgByte0, kLcdClearDisplay
    rcall sendCmdToLcd

.endm



; **********************************
;  M A C R O
; **********************************

; Arguments:  @0 = row  (byte), @1 = column (byte)
.macro setLcdRowColM

    ldi rArgByte0, @0
    ldi rArgByte1, @1
    rcall setLcdRowCol

.endm



; **********************************
;  M A C R O
; **********************************

; Arguments:  @0 = LCD command (byte)
.macro sendCmdToLcdM

    ldi rArgByte0, @0
    rcall sendCmdToLcd

.endm



; **********************************
;  M A C R O
; **********************************

; Arguments:  @0 = Data to display (byte)
.macro sendDataToLcdM

    ldi rArgByte0, @0
    rcall sendDataToLcd

.endm



; **********************************
;  M A C R O
; **********************************

; Arguments:  @0 = Data to display (byte)
.macro sendDataToLcdMR

    mov rArgByte0, @0
    rcall sendDataToLcd

.endm



; **********************************
;  M A C R O
; **********************************

; Arguments:  @0 = address of 16 bit message to display on LCD
.macro displayMsgOnLcdM

    ldiw Z, @0
    call displayMsgOnLcd

.endm




; **********************************
;  D A T A   S E G M E N T
;        ( S R A M )
; **********************************

.dseg
.org SRAM_START


sStaticDataBegin:

    sJumpTable:
        .byte 16*2

    sLcdGreeting:
        .byte 16

    sBankLine:
        .byte 16

    sOverflowMsg:
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

; Jump table
.dw  doKey0,  doKey1,  doKey2,  doKey3
.dw  doKey4,  doKey5,  doKey6,  doKey7
.dw  doKey8,  doKey9,  doKey10, doKey11
.dw  doKey12, doKey13, doKey14, doKey15

; LCD Message
.db 'K', 'e', 'y', ' ', 'h', 'i', 't', ':', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '

; Blank Line
.db ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '

; Overflow msg
.db 'O', 'v', 'e', 'r', 'f', 'l', 'o', 'w', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '

dStaticDataEnd:

.equ kdStaticDataLen = 2 * ( dStaticDataEnd - dStaticDataBegin )



; ***************************************
;  I N T E R R U P T   H A N D L E R S
; ***************************************


; None




; ***************************************
;  M A I N   ( R E S E T )
; ***************************************

main:

    initializeStack rTmp1

    rcall initStaticData                        ; Move static data from PROGMEM to SRAM

    ; Start the LCD display
    rcall initializeLcd

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

    ; Prepare the LCD display
    setLcdRowColM 0, 0
    displayMsgOnLcdM sLcdGreeting

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

doKey0:
    ldi rTmp2, '1'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey1:
    ldi rTmp2, '2'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey2:
    ldi rTmp2, '3'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey3:
    ldi rTmp2, '/'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey4:
    ldi rTmp2, '4'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey5:
    ldi rTmp2, '5'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey6:
    ldi rTmp2, '6'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey7:
    ldi rTmp2, 'x'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey8:
    ldi rTmp2, '7'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey9:
    ldi rTmp2, '8'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey10:
    ldi rTmp2, '9'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey11:
    ldi rTmp2, '-'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey12:
    ldi rTmp2, 's'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey13:
    ldi rTmp2, '0'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey14:
    ldi rTmp2, 'E'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKey15:
    ldi rTmp2, '+'
    setLcdRowColM 1, 14                         ; Display the key hit second row, second-to-last column
    sendDataToLcdMR rTmp2
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doKeyHit:

    rcall doScanKeyPad
    rcall doDisplayKey
    delayMilliSecondsM 200                      ; Delay for button de-bounce
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doDisplayKey:
    ldiw X, sJumpTable                          ; Read number corresponding to key from SRAM
    lsl rKey                                    ; Multiply by 2 (jump addresses are words)
    add XL, rKey                                ; Add the offset (possible carry required)
    clr rTmp2                                   ; Doesn't affect carry flag
    adc XH, rTmp2                               ; X now points to the low byte in SRAM of the jump address
    ld ZL, X+
    ld ZH, X                                    ; Z now contains the address to call
    icall
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

; Returns the number of the key hit in register rKey
; Numbered as:
;   0   1   2   3
;   4   5   6   7
;   8   9   10  11
;   12  13  14  15

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

doNumberKey:

    ; rKey = (inbound) the numerical value of number key

    sbrs rState, kDigitEntryBitNbr
    rjmp doNumberKey_Continuing
                                                ; Just started entering a number
    ; TODO Need to roll the stack up!!!
    clr rWorkingNbrH                            ; Clear the registers we accumulate the number in
    clr rWorkingNbrL
    cbi rState, kNbrSignBit                     ; Clear the sign indicator bit
    sbi rState, kDigitEntryBit                  ; Set that we are in number entry mode
    mov rWorkingNbrL, rKey
    ret

doNumberKey_Continuing:
    ; Multiply the existing number by 10 to incorporate a new digit
    add rWorkingNbrL, rKey
    clr rKey                                    ; Doesn't affect carry flag
    adc rWorkingNbrH, rKey

    brcs doNumberKey_Overflow

    ret

doNumberKey_Overflow:
    call doOverflow:
    ret




; **********************************
;  S U B R O U T I N E
; **********************************

doOverflow:
    ; Prepare the LCD display
    setLcdRowColM 1, 0
    displayMsgOnLcdM sOverflowMsg





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

initializeLcd:

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

    ; Wait 50 milliseconds to ensure full voltage rise
    delayMilliSecondsM 50

    cbi pLcdDataSelectPort, pLcdDataSelectPortBit   ; Pull DS pin Low (sending commands)
    cbi pLcdEnablePort, pLcdEnablePortBit           ; Pull E pin Low

    ; Need to send 0x03 three times

    ; First time
    ldi rArgByte0, 0x03
    rcall write4BitsToLcd

    ; Wait > 4.1 ms
    delayMicroSecondsM  4500

    ; Second time
    ldi rArgByte0, 0x03
    rcall write4BitsToLcd

    ; Wait > 4.1 ms
    delayMicroSecondsM  4500

    ; Third try and go...
    ldi rArgByte0, 0x03
    rcall write4BitsToLcd

    ; Wait >150 us
    delayMicroSecondsM  200

    ; This actually sets the 4-bit interface
    ldi rArgByte0, 0x02
    rcall write4BitsToLcd

    ; Set nbr of lines and font
    sendCmdToLcdM  ( kLcdFunctionSet | kLcd2Line | kLcd5x8Dots )

    ; Turn on display, with cursor off and blinking off
    sendCmdToLcdM ( kLcdDisplayControl | kLcdDisplayOn | kLcdCursorOff | kLcdBlinkOff )

    ; Set text entry more (L to R)
    sendCmdToLcdM  ( kLcdEntryModeSet | kLcdEntryLeft )

    ; Clear display
    sendCmdToLcdM kLcdClearDisplay
    delayMicroSecondsM 2000                     ; Clear cmd takes a long time...

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
    delayMicroSecondsM 2
    sbi pLcdEnablePort, pLcdEnablePortBit       ; Enable pin HIGH (actual enable pulse)
    delayMicroSecondsM 2                        ; Enable pulse must be > 450ns
    cbi pLcdEnablePort, pLcdEnablePortBit       ; Enable pin LOW
    delayMicroSecondsM 100                      ; Seems like a lot but didn't work with 70us... Command needs > 37us to settle

    ret



; **********************************
;  S U B R O U T I N E
; **********************************

sendDataToLcd:

    ; Register rArgByte0 is passed as parameter

    ; rArgByte0 = the byte to write to the LCD (modified)
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

displayMsgOnLcd:

    ; Z passed as parameter (pointer to message, modified)

    ; Z             = pointer to SRAM to 16 character (byte) message to display
    ; rTmp1         = temporary
    ; rLoop1        = loop counter
    ; rArgByte0     = Used
    ; rArgByte1     = Used

    ; Position display
    setLcdRowColM 0, 0

    ; Set up loop and display
    ldi rTmp1, kDisplayMsgLen
    mov rLoop1, rTmp1
displayMsgLoop:
        ld rArgByte0, Z+
        rcall sendDataToLcd
        dec rLoop1
        brne displayMsgLoop

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

    ; r25:r24 = number of milliseconds to count (comes in as argument)
    ;     = number of times to execute the outer+inner loops combined
    ; r16 = outer loop counter byte
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
                sbiw rDWMSInnerH:rDWMSInnerL, 1
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



; **********************************
;  S U B R O U T I N E
; **********************************

multU16by8:

    ; r25:r24 = 16-bit multiplicand
    ; r22 = 8-bit multiplicand
    ; r18:r19:r20 = result

#define rM16L       r24
#define rM16H       r25
#define rM8         r22
#define rProd0      r18
#define rProd1      r19
#define rProd2      r20

    mul rM16L, rM8      ; Multiply LSB
    movw rProd0, r0     ; Copy result

    mul rM16H, rM8      ; Multiply MSB
    mov rProd2, r1      ; Copy MSB result to result byte 3
    add rProd1, r0      ; Add LSB result to result byte 2
    brcc 1f             ; If no carry, done
    inc rProd2          ; Do the carry
multU16by8_1:

    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doWord2sComplement:

    com rArgByte1
    neg rArgByte0
    sbci rArgByte1, 0xff
    ret



; **********************************
;  S U B R O U T I N E
; **********************************

doDword2sComplement:

    com rArgByte3
    com rArgByte2
    com rArgByte1
    neg rArgByte0
    sbci rArgByte1, 0xff
    sbci rArgByte2, 0xff
    sbci rArgByte3, 0xff
    ret
