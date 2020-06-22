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

.def rArgByte0      = r24                       ; For now using C register conventions for function calls
.def rArgByte1      = r25                       ; Second byte arg, or high byte of word arg

.def rCounterLSB    = r26                       ; LSB of word (16-bit) counter (XL)
.def rCounterMSB    = r27                       ; MSB of word (16-bit) counter (XH)

;***** Register used by all programs
;******Global variable used by all routines
.def temp =r16 ;general scratch space



;***** Registers used by interrupt service routine
.def key =r17 ;key pointer for EEPROM
.def status =r21 ;preserve sreg here



; ************************************
;  E E P R O M   S E G M E N T
; ************************************

.eseg
.org 0

; Look up table for key conversion
.db 1, 2, 3, 15, 4, 5, 6, 14, 7, 8, 9, 13, 10, 0, 11, 12




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
;  I N T E R R U P T  H A N D L E R S
; ***************************************

scanKeyPad:
    in status, SREG                 ; Preserve status register
    sbis PINB, ROW1                 ; Find row of keypress
    ldi key, 0                      ; Set ROW pointer
    sbis PINB, ROW2
    ldi key, 4
    sbis PINB, ROW3
    ldi key, 8
    sbis PINB, ROW4
    ldi key, 12
    ldi temp, 0x0F                  ; Change port B I/O to
    out DDRB, temp                  ; find column press
    ldi temp, 0xF0                  ; Enable pull ups and
    out PORTB, temp                 ; Write 0s to rows
    rcall settle                    ; Allow time for port to settle
    sbis PINB, COL1                 ; Find column of keypress
    ldi temp, 0                     ; And set COL pointer
    sbis PINB, COL2
    ldi temp, 1
    sbis PINB, COL3
    ldi temp, 2
    sbis PINB, COL4
    ldi temp, 3
    add key, temp                   ;merge ROW and COL for pointer
    ldi temp, 0xF0                  ;reinitialise port B as I/O
    out DDRB, temp                  ; 4 OUT 4 IN
    ldi temp, 0x0F                  ;key columns all low and
    out PORTB, temp                 ;active pull ups on rows enabled
    out SREG, status                ;restore status register
    ldi temp, 0x00
    out GIMSK, temp                 ; Disable external interrupt have to do this, because we're using a level-triggered interrupt
    reti








;*** Reset handler *************************************************
reset:
;    ldi temp, 0xFB          ; Initialize port D as O/I
;    out DDRD, temp          ; All OUT except PD2 ext.int.
;    ldi temp, 0x30         ; Turn on sleep mode and power
;    out MCUCR, temp        ; Down plus interrupt on low level.

    ; Initialize LEDs
    sbi pGreenLedDirD, pGreenLedDirDBit
    cbi pGreenLedPort, pGreenLedPortBit
    sbi pRedLedDirD, pGreenLedDirDBit
    cbi pRedLedLedPort, pRedLedPortBit

    ; INT0/PD2, external interrupt 0
    cbi pInt0DirD, pInt0DirDBit     ; Set as input
    sbi pInt0Port, pInt0PortBit     ; Enable pullup

    ldi temp, 0x40          ; Enable external interrupts
    out GIMSK, temp
;    sbi ACSR,ACD ;shut down comparator to save power

main:
    cli                             ; Disable interrupts

    ; Columns
    in temp, pColDirD               ; Set PD4-PD7, columns, as output (others unchanged)
    ori temp, 0xF0
    out pColDirD, temp
    in temp, pColDirD               ; Set PD4-PD7 as low
    andi temp, 0x0F
    out pColDirD, temp

    ; Rows
    in temp, pRowDirD               ; Set PB0-PB3, rows, as input
    andi temp, 0xF0
    out pRowDirD, temp
    in temp, pRowDirD               ; Enable pull ups on PB0-PB3
    ori temp, 0x0F
    out pRowDirD, temp

    ; LEDs off
    sbi pGreenLedDirD, pGreenLedDirDBit
    cbi pGreenLedPort, pGreenLedPortBit
    sbi pRedLedDirD, pGreenLedDirDBit
    cbi pRedLedLedPort, pRedLedPortBit

    sei                             ; Enable interrupts

    sleep

    rcall flash                     ; Flash LEDs

    ldi temp,0x40                   ; Enable external interrupt
    out GIMSK,temp
    rjmp main                       ; Loop




;***Example test program to flash LEDs using key press data***********

flash:
    out EEAR, key                   ; Address EEPROM
    sbi EECR, EERE                  ; Strobe EEPROM
    in temp, EEDR                   ; Set number of flashes
    tst temp                        ; Is it zero?
    breq zero                       ; Do RED LED

green_flash:
    cbi PORTD, GREEN                ; Flash green LED 'temp' times
    rcall delay
    sbi PORTD, GREEN
    rcall delay
    dec temp
    brne green_flash

exit:
    ret

zero:
    ldi temp,10

flash_again:
    cbi PORTD, RED                  ; Flash red LED ten times
    rcall delay
    sbi PORTD,  RED
    rcall delay
    dec temp
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
    ldi temp,255            ; 1 cycle
tagain:
    dec temp                ; 1 cycle
    brne tagain             ; 2 if true, 1 if false
    ret                     ; 4 cycles

; Total:  1 + 4 + 253 * 3 + 2 = 766 cycles =
