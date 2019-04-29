;*******************************************************************************
;                                                                              *
;    Microchip licenses this software to you solely for use with Microchip     *
;    products. The software is owned by Microchip and/or its licensors, and is *
;    protected under applicable copyright laws.  All rights reserved.          *
;                                                                              *
;    This software and any accompanying information is for suggestion only.    *
;    It shall not be deemed to modify Microchip?s standard warranty for its    *
;    products.  It is your responsibility to ensure that this software meets   *
;    your requirements.                                                        *
;                                                                              *
;    SOFTWARE IS PROVIDED "AS IS".  MICROCHIP AND ITS LICENSORS EXPRESSLY      *
;    DISCLAIM ANY WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING  *
;    BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS    *
;    FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL          *
;    MICROCHIP OR ITS LICENSORS BE LIABLE FOR ANY INCIDENTAL, SPECIAL,         *
;    INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, HARM TO     *
;    YOUR EQUIPMENT, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR    *
;    SERVICES, ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY   *
;    DEFENSE THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER      *
;    SIMILAR COSTS.                                                            *
;                                                                              *
;    To the fullest extend allowed by law, Microchip and its licensors         *
;    liability shall not exceed the amount of fee, if any, that you have paid  *
;    directly to Microchip to use this software.                               *
;                                                                              *
;    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF    *
;    THESE TERMS.                                                              *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Filename:                                                                 *
;    Date:                                                                     *
;    File Version:                                                             *
;    Author:                                                                   *
;    Company:                                                                  *
;    Description:                                                              *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Notes: In the MPLAB X Help, refer to the MPASM Assembler documentation    *
;    for information on assembly instructions.                                 *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Known Issues: This template is designed for relocatable code.  As such,   *
;    build errors such as "Directive only allowed when generating an object    *
;    file" will result when the 'Build in Absolute Mode' checkbox is selected  *
;    in the project properties.  Designing code in absolute mode is            *
;    antiquated - use relocatable mode.                                        *
;                                                                              *
;*******************************************************************************
;                                                                              *
;    Revision History:                                                         *
;                                                                              *
;*******************************************************************************



;*******************************************************************************
; Processor Inclusion
;
; TODO Step #1 Open the task list under Window > Tasks.  Include your
; device .inc file - e.g. #include <device_name>.inc.  Available
; include files are in C:\Program Files\Microchip\MPLABX\mpasmx
; assuming the default installation path for MPLAB X.  You may manually find
; the appropriate include file for your device here and include it, or
; simply copy the include generated by the configuration bits
; generator (see Step #2).
;
;*******************************************************************************

; TODO INSERT INCLUDE CODE HERE
    #include "p16f15356.inc"

;*******************************************************************************
;
; TODO Step #2 - Configuration Word Setup
;
; The 'CONFIG' directive is used to embed the configuration word within the
; .asm file. MPLAB X requires users to embed their configuration words
; into source code.  See the device datasheet for additional information
; on configuration word settings.  Device configuration bits descriptions
; are in C:\Program Files\Microchip\MPLABX\mpasmx\P<device_name>.inc
; (may change depending on your MPLAB X installation directory).
;
; MPLAB X has a feature which generates configuration bits source code.  Go to
; Window > PIC Memory Views > Configuration Bits.  Configure each field as
; needed and select 'Generate Source Code to Output'.  The resulting code which
; appears in the 'Output Window' > 'Config Bits Source' tab may be copied
; below.
;
;*******************************************************************************

; TODO INSERT CONFIG HERE
; CONFIG1
; __config 0x1FFA
 __CONFIG _CONFIG1, _FEXTOSC_HS & _RSTOSC_EXT1X & _CLKOUTEN_OFF & _CSWEN_ON & _FCMEN_OFF
; CONFIG2
; __config 0x3FFF
 __CONFIG _CONFIG2, _MCLRE_ON & _PWRTE_OFF & _LPBOREN_OFF & _BOREN_ON & _BORV_LO & _ZCD_OFF & _PPS1WAY_ON & _STVREN_ON
; CONFIG3
; __config 0x3F9F
 __CONFIG _CONFIG3, _WDTCPS_WDTCPS_31 & _WDTE_OFF & _WDTCWS_WDTCWS_7 & _WDTCCS_SC
; CONFIG4
; __config 0x3FFF
 __CONFIG _CONFIG4, _BBSIZE_BB512 & _BBEN_OFF & _SAFEN_OFF & _WRTAPP_OFF & _WRTB_OFF & _WRTC_OFF & _WRTSAF_OFF & _LVP_ON
; CONFIG5
; __config 0x3FFF
 __CONFIG _CONFIG5, _CP_OFF




;*******************************************************************************
;
; TODO Step #3 - Variable Definitions
;
; Refer to datasheet for available data memory (RAM) organization assuming
; relocatible code organization (which is an option in project
; properties > mpasm (Global Options)).  Absolute mode generally should
; be used sparingly.
;
; Example of using GPR Uninitialized Data
;
;   GPR_VAR        UDATA
;   MYVAR1         RES        1      ; User variable linker places
;   MYVAR2         RES        1      ; User variable linker places
;   MYVAR3         RES        1      ; User variable linker places
;
;   ; Example of using Access Uninitialized Data Section (when available)
;   ; The variables for the context saving in the device datasheet may need
;   ; memory reserved here.
;   INT_VAR        UDATA_ACS
;   W_TEMP         RES        1      ; w register for context saving (ACCESS)
;   STATUS_TEMP    RES        1      ; status used for context saving
;   BSR_TEMP       RES        1      ; bank select used for ISR context saving
;
;*******************************************************************************

; TODO PLACE VARIABLE DEFINITIONS GO HERE
    CBLOCK	0x20	
	MYD1	
	MYD2	
    ENDC

;*******************************************************************************
; Reset Vector
;*******************************************************************************

RES_VECT  CODE    0x0000            ; processor reset vector
    GOTO    START                   ; go to beginning of program

;*******************************************************************************
; TODO Step #4 - Interrupt Service Routines
;
; There are a few different ways to structure interrupt routines in the 8
; bit device families.  On PIC18's the high priority and low priority
; interrupts are located at 0x0008 and 0x0018, respectively.  On PIC16's and
; lower the interrupt is at 0x0004.  Between device families there is subtle
; variation in the both the hardware supporting the ISR (for restoring
; interrupt context) as well as the software used to restore the context
; (without corrupting the STATUS bits).
;
; General formats are shown below in relocatible format.
;
;------------------------------PIC16's and below--------------------------------
;
; ISR       CODE    0x0004           ; interrupt vector location
;
;     <Search the device datasheet for 'context' and copy interrupt
;     context saving code here.  Older devices need context saving code,
;     but newer devices like the 16F#### don't need context saving code.>
;
;     RETFIE
;
;----------------------------------PIC18's--------------------------------------
;
; ISRHV     CODE    0x0008
;     GOTO    HIGH_ISR
; ISRLV     CODE    0x0018
;     GOTO    LOW_ISR
;
; ISRH      CODE                     ; let linker place high ISR routine
; HIGH_ISR
;     <Insert High Priority ISR Here - no SW context saving>
;     RETFIE  FAST
;
; ISRL      CODE                     ; let linker place low ISR routine
; LOW_ISR
;       <Search the device datasheet for 'context' and copy interrupt
;       context saving code here>
;     RETFIE
;
;*******************************************************************************

; TODO INSERT ISR HERE
ISR	CODE    0x0004		; Interrupt vector location

    banksel	PIR0		; Switch to PIR0 BANK14
    btfsc	PIR0, TMR0IF    ; is Timer interrupt overflowed?     
    goto	TMR_INT
    
; If timer didn't interrupt, the button push must have
BTN_INT
    banksel	PORTA		; Back to BANK0
    ;movf	PORTA, F	; Read PORTA to itself
    banksel	PORTB
    movlw	(1<<4)		; Fourth bit
    xorwf	LATB, F		; set the latch for LED
    ; Clear interrupts
    movlw	0xFF
    banksel	IOCAF
    xorwf	IOCAF, W	; clear the
    andwf	IOCAF, F	; interrupts
    
    retfie

TMR_INT
    
    ; Clear TIMER0 overflow bit
    banksel	PIR0
    bcf		PIR0, TMR0IF	; Clear the bit
    
    banksel	MYD1		; count holder
    incf	MYD1, F		; increase count, put in MYD1
    movf	MYD2, W		; move test value into W
    subwf	MYD1, W		; subtact MYD1 from MYD2 each time
    btfss	STATUS, Z	; skip next line if Z is 1 (1 means subtraction
				; of MYD1 from MYD2 resulted in a 0)
    retfie
    
    banksel	PORTB		; need to ensure we are in BANK0
    movlw	(1<<5)		; set 5th bit only
    xorwf	LATB, F		; xor to flip bit PORTB RB<5> - high/low
    
   ; Reset counters    
    banksel	MYD1		; Clear out MYD1 counter
    clrf	MYD1		; back to zero
    
    retfie
   
    
;*******************************************************************************
; MAIN PROGRAM
;*******************************************************************************

MAIN_PROG CODE			; let linker place main program

START
    ; Setup or pins for input/output/analog functionality
    ; PIN 26 on the PIC is RB5 - PORTB Register at B'000100000'
    ; Setup to digital, and data directionto out
    
    ; Port info in section 14.3 of datasheet

    banksel	PORTB		; This is BANK 0
    clrf	PORTB		; Init PORTB to zero - low
    
    banksel	LATB		; BANK 0 again
				; LATB is where PORTB data is written
    clrf	LATB		; Init LATB to zero
    
    banksel	ANSELB		; Analog Select for pins on PORTB (BANK 62) 
    clrf	ANSELB		; Init ANSELB to zero - digital pins
    
    banksel	TRISB		; Data direction for PORTB (also in BANK 0)
    clrw			; Clear WREG
    movlw	0xFF		; All 1's
    bcf		TRISB, RB5	; Clear bit 5 and 4
    bcf		TRISB, RB4	; Set RB<5:4> as output (0's)
				; and set RB<7:6> and RB<3:0> as inputs (1's)
    
; Setup PORTA, 5 as an input				
    banksel	PORTA		; Set PORTA is in BANK 0
    clrf	PORTA		; Set to zero - low
    
    banksel	LATA		; Data latch for PORTA in BANK0
				; LATA is where PORTB data is written
    clrf	LATA		; Init LATA to zero
    
    banksel	ANSELA		; Analog Select for pins on PORTA 
    clrf	ANSELA		; Init ANSELA to zero - digital pins
    
    banksel	TRISA		; Data direction for PORTA (also in BANK 0)
    clrw			; Clear WREG
    movlw	(1<<5)		; Set 5th bit to 1 - 00010000
    movwf	TRISA		; Set RA<5> as input (1's)
				; and set RA<7:6> and RA<4:0> as outputs (0's)
    
    ; Timer 0 Setup
    ; Timer 0 found in section 25 of datasheet
    
    banksel	T0CON0
    clrf	T0CON0
    bsf		T0CON0, T0EN	; Enable Timer 0
    bsf		T0CON0, T016BIT	; 16 bit counter
    
    ; Setup Timer 0 parameter		
    clrf	T0CON1
    clrw			; Clear w reg
    movlw	(0x02 << 5)	; FOSC/4 Clock source in bits <7:5>
    movwf	T0CON1		; Move into T0CON1 reg
    clrw
    movlw	0x05		; Values for bits <4:0>
				; Prescale rate 1:32
    iorwf	T0CON1, F	; OR with W to set remaining bits
				; bits <4:0>
    
    ; Timer 0 Needs a Counter using MYD1 and setting to 0
    banksel	MYD1
    clrf	MYD1
    movlw	0x02		; need value to subtract 
    movwf	MYD2		; from Timer counter
				; 0x02 is 2*524 mseconds
    
    ; Interrupt code setup
    ; Interrupt Registers are found in seciton 10.6 of datasheet
    
    ; Setup button Interrupt and Timer 0 Interrupt
    banksel	PIE0		; Set for PIN Change BANK14
    clrf	PIE0		; Clear to zeros
    bsf		PIE0, IOCIE	; Set IOCIE bit to allow interrupt on change
    bsf		PIE0, TMR0IE	; Enable Timer 0 interrupt
    
    ; Setup button PORTA RA5 for on change interrupt
    banksel	IOCAN		; Select BANK62 for PORTA pins
				; PORTA Negative Edge Register
    clrf	IOCAN		; Clear to zeros
    bsf		IOCAN, IOCAN5	; Set for Falling Edge on RA5 since our 
				; button will have a pull-up resistor
    
    ; After initial pin and Timer 0 setup, enable Global interrupts
    clrf	INTCON		; Clear to zeros
    bsf		INTCON, PEIE	; Enable peripheral interrupts (needed for Timer 0)
    bsf		INTCON, GIE	; Enable Global Interrupts
    
    
LOOP

    ; TODO Step #5 - Insert Your Program Here
    ; Nothing much in here - Interrupts are handling everything
    goto	LOOP
    
;--------------Subroutines--------------------
    

    
;-----------------End Program-----------------

		    END