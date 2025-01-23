;------------------------------------------------------------------------------
; Beau Coburn & Aaron Foster
; EELE 465 Project 2: I2C RTC
; 2/6/2025
;Description:
;------------------------------------------------------------------------------

;-------------------------------------------------------------------------------
; Include files
            .cdecls C,LIST,"msp430.h"  ; Include device header file
;-------------------------------------------------------------------------------

            .def    RESET                   ; Export program entry-point to
                                            ; make it known to linker.

            .global __STACK_END
            .sect   .stack                  ; Make stack linker segment ?known?

            .text                           ; Assemble to Flash memory
            .retain                         ; Ensure current section gets linked
            .retainrefs

RESET       mov.w   #__STACK_END,SP         ; Initialize stack pointer


init:
            ; stop watchdog timer
            mov.w   #WDTPW+WDTHOLD,&WDTCTL

            ; Set Pin 6.0 as output for SDA (Port6.SDA)
            bic.b   #BIT0, &P6OUT
            bis.b   #BIT0, &P6DIR

            ; Set Pin 6.1 as output for SCL (Port6.SCL)
            bic.b   #BIT1, &P6OUT
            bis.b   #BIT1, &P6DIR

            ;LED Initialization
            bic.b   #BIT0,&P1OUT            ; Clear P1.0 output
            bis.b   #BIT0,&P1DIR            ; Set P1.0 as output
;--------------------------------Timer Setup------------------------------------
;TimerB0
            bis.w	#TBCLR, &TB0CTL			;Clear timer and dividers
            bis.w 	#TBSSEL__ACLK, &TB0CTL	;Select ACLK as timer source
            bis.w	#MC__CONTINUOUS, &TB0CTL;choose continuous counting
            bis.w	#CNTL_1, &TB0CTL		;choose counter length=12 bits
            bis.w	#ID_3, &TB0CTL			;choose divider D1=4
            bis.w 	#TBIE, &TB0CTL			;Enable Overflow Interrupt
            bic.w 	#TBIFG, &TB0CTL			;Clear Interrupt flag

;----------------------------End Timer Setup------------------------------------

            ; Disable low-power mode
            bic.w   #LOCKLPM5,&PM5CTL0
            nop
            bis.w	#GIE, SR					;Enable global interrupt
            nop

main:

            nop 
            jmp main
            nop



;------------------------------------------------------------------------------
; Interrupt Service Routines
;------------------------------------------------------------------------------


;--------------------------Start TimerB1_2s-------------------------------------
TimerB0_1s:

	xor.b	#BIT0, &P1OUT		; Toggle P1.0(LED)
;    xor.b   #BIT0, &P6OUT      ; Testing I2C Pins
;    xor.b   #BIT1, &P6OUT
	bic.w	#TBIFG, &TB0CTL		;Clear interrupt flag

	reti
;-------------------------------End TimerB1_2s----------------------------------

;------------------------------------------------------------------------------
;           Interrupt Vectors
;------------------------------------------------------------------------------
            .sect   RESET_VECTOR            ; MSP430 RESET Vector
            .short  RESET                   ;

            .sect   ".int42"                ; TB0 vector
            .short  TimerB0_1s              ;
            .end
