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

            ; Set Pin 6.0 as output for SDA
            bic.w   #BIT0, &P6OUT
            bis.w   #BIT0, &P6DIR

            ; Set Pin 6.1 as output for SCL
            bic.w   #BIT1, &P6OUT
            bis.w   #BIT1, &P6DIR

            ; Disable low-power mode
            bic.w   #LOCKLPM5,&PM5CTL0

main:

            nop 
            jmp main
            nop



;------------------------------------------------------------------------------
;           Interrupt Vectors
;------------------------------------------------------------------------------
            .sect   RESET_VECTOR            ; MSP430 RESET Vector
            .short  RESET                   ;
