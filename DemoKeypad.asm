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

; Going for D4-D7 (columns) and B0-3 (rows) an D2 for INT0


; **********************************
;  C O N S T A N T S
; **********************************

;Port B pins
.equ pRowDirD                       = DDRB
.equ ROW1   = 3 ;keypad input rows
.equ ROW2   = 2
.equ ROW3   = 1
.equ ROW4   = 0

;Port D pins
.equ pColDirD                       = DDRD
.equ COL1   = 7 ;keypad output columns
.equ COL2   = 6
.equ COL3   = 5
.equ COL4   = 4

;Port C pins
.equ GREEN  = 3 ;green LED
.equ RED    = 2 ;red LED

;Port D pins
.equ INTR   = 2 ;interrupt input



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

.def rKey           = r18                       ; Index of key hit, used to look value in Key Table

.def rArgByte0      = r24                       ; For now using C register conventions for function calls
.def rArgByte1      = r25                       ; Second byte arg, or high byte of word arg

.def rCounterLSB    = r26                       ; LSB of word (16-bit) counter (XL)
.def rCounterMSB    = r27                       ; MSB of word (16-bit) counter (XH)

;


;***** Registers used by interrupt service routine
.def key =r17 ;key pointer for EEPROM




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



; ************************************
;  E E P R O M   S E G M E N T
; ************************************

;.eseg
;.org 0

; Look up table for key conversion
;.db 1, 2, 3, 15, 4, 5, 6, 14, 7, 8, 9, 13, 10, 0, 11, 12



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
.org 0




; ************************************
;  I N T E R R U P T  V E C T O R S
; ************************************

.org 0x00
	rjmp reset                 ; Reset vector
.org 0x02
	rjmp scanKeyPad            ; INT0
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
;  I N T E R R U P T  H A N D L E R S
; ***************************************

scanKeyPad:
    in rSREG, SREG                 ; Preserve status register
    sbis PINB, ROW1                 ; Find row of keypress
    ldi rKey, 0                      ; Set ROW pointer
    sbis PINB, ROW2
    ldi rKey, 4
    sbis PINB, ROW3
    ldi rKey, 8
    sbis PINB, ROW4
    ldi rKey, 12
    ldi rTmp1, 0x0F                  ; Change port B I/O to
    out DDRB, rTmp1                  ; find column press
    ldi rTmp1, 0xF0                  ; Enable pull ups and
    out PORTB, rTmp1                 ; Write 0s to rows
    rcall settle                    ; Allow time for port to settle
    sbis PINB, COL1                 ; Find column of keypress
    ldi rTmp1, 0                     ; And set COL pointer
    sbis PINB, COL2
    ldi rTmp1, 1
    sbis PINB, COL3
    ldi rTmp1, 2
    sbis PINB, COL4
    ldi rTmp1, 3
    add rKey, rTmp1                   ;merge ROW and COL for pointer
    ldi rTmp1, 0xF0                  ;reinitialise port B as I/O
    out DDRB, rTmp1                  ; 4 OUT 4 IN
    ldi rTmp1, 0x0F                  ;rKey columns all low and
    out PORTB, rTmp1                 ;active pull ups on rows enabled
    out SREG, status                ;restore status register
    ldi rTmp1, 0x00
    out GIMSK, rTmp1                 ; Disable external interrupt have to do this, because we're using a level-triggered interrupt
    reti








;*** Reset handler *************************************************
reset:
;    ldi rTmp1, 0xFB          ; Initialize port D as O/I
;    out DDRD, rTmp1          ; All OUT except PD2 ext.int.
;    ldi rTmp1, 0x30         ; Turn on sleep mode and power
;    out MCUCR, rTmp1        ; Down plus interrupt on low level.


    initializeStack rTmp1

    rcall initStaticData                        ; Move static data from PROGMEM to SRAM


    ; Initialize LEDs
    sbi pGreenLedDirD, pGreenLedDirDBit
    cbi pGreenLedPort, pGreenLedPortBit
    sbi pRedLedDirD, pGreenLedDirDBit
    cbi pRedLedLedPort, pRedLedPortBit

    ; INT0/PD2, external interrupt 0
    cbi pInt0DirD, pInt0DirDBit     ; Set as input
    sbi pInt0Port, pInt0PortBit     ; Enable pullup

    ldi rTmp1, 0x40          ; Enable external interrupts
    out GIMSK, rTmp1
;    sbi ACSR,ACD ;shut down comparator to save power

main:
    cli                             ; Disable interrupts

    ; Columns
    in rTmp1, pColDirD               ; Set PD4-PD7, columns, as output (others unchanged)
    ori rTmp1, 0xF0
    out pColDirD, rTmp1
    in rTmp1, pColDirD               ; Set PD4-PD7 as low
    andi rTmp1, 0x0F
    out pColDirD, rTmp1

    ; Rows
    in rTmp1, pRowDirD               ; Set PB0-PB3, rows, as input
    andi rTmp1, 0xF0
    out pRowDirD, rTmp1
    in rTmp1, pRowDirD               ; Enable pull ups on PB0-PB3
    ori rTmp1, 0x0F
    out pRowDirD, rTmp1

    ; LEDs off
    sbi pGreenLedDirD, pGreenLedDirDBit
    cbi pGreenLedPort, pGreenLedPortBit
    sbi pRedLedDirD, pGreenLedDirDBit
    cbi pRedLedLedPort, pRedLedPortBit

    sei                             ; Enable interrupts

    sleep

    rcall flash                     ; Flash LEDs

    ldi rTmp1,0x40                   ; Enable external interrupt
    out GIMSK,rTmp1
    rjmp main                       ; Loop




;***Example test program to flash LEDs using key press data***********

flash:
    out EEAR, rKey                   ; Address EEPROM
    sbi EECR, EERE                  ; Strobe EEPROM
    in rTmp1, EEDR                   ; Set number of flashes
    tst rTmp1                        ; Is it zero?
    breq zero                       ; Do RED LED

green_flash:
    cbi PORTD, GREEN                ; Flash green LED 'rTmp1' times
    rcall delay
    sbi PORTD, GREEN
    rcall delay
    dec rTmp1
    brne green_flash

exit:
    ret

zero:
    ldi rTmp1,10

flash_again:
    cbi PORTD, RED                  ; Flash red LED ten times
    rcall delay
    sbi PORTD,  RED
    rcall delay
    dec rTmp1
    brne flash_again
    rjmp exit


;****Time Delay Subroutine for LED flash*********************************
; Delay about 0.25 sec
delay:
    ldi coarse,5        ; triple nested FOR loop
cagain:
    ldi medium,255      ; giving about 1/4 second
magain:
    ldi fine,255        ; delay on 4 MHz clock
fagain:
    dec fine
    brne fagain
    dec medium
    brne magain
    dec coarse
    brne cagain
    ret


;***Settling time delay for port to stabilise****************************
; Delay about 192us at 4 MHz
settle:
    ldi rTmp1,255            ; 1 cycle
tagain:
    dec rTmp1                ; 1 cycle
    brne tagain             ; 2 if true, 1 if false
    ret                     ; 4 cycles

; Total:  1 + 4 + 253 * 3 + 2 = 766 cycles =



; **********************************
;  S U B R O U T I N E
; **********************************

initStaticData:

    ; Copy the static strings into SRAM

    ; Z             = pointer to program memory
    ; X             = pointer to SRAM
    ; rTmp1         = counter
    ; rScratch1     = transfer register

    ; Copy greeting string
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
