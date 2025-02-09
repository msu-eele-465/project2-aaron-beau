;-------------------------------------------------------------------------------
; Beau Coburn & Aaron Foster
; EELE 465 Project 2: I2C RTC
; 2/6/2025
;Description:
;-------------------------------------------------------------------------------

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

            
            ; Set Pin 6.1 as output for SCL (Port6.SCL)
            bis.b   #BIT1, &P6OUT
            bis.b   #BIT1, &P6DIR

            ; Set Pin6.0 as output for SDA (Port 6.0)
            bis.b   #BIT0, &P6OUT
            bis.b   #BIT0, &P6DIR

            ;LED Initialization
            bic.b   #BIT0,&P1OUT            ; Clear P1.0 output
            bis.b   #BIT0,&P1DIR            ; Set P1.0 as output
;--------------------------------Timer Setup-----------------------------------
;TimerB0
            bis.w	#TBCLR, &TB0CTL			;Clear timer and dividers
            bis.w 	#TBSSEL__ACLK, &TB0CTL	;Select ACLK as timer source
            bis.w	#MC__CONTINUOUS, &TB0CTL;choose continuous counting
            bis.w	#CNTL_1, &TB0CTL		;choose counter length=12 bits
            bis.w	#ID_3, &TB0CTL			;choose divider D1=4
            bis.w 	#TBIE, &TB0CTL			;Enable Overflow Interrupt
            bic.w 	#TBIFG, &TB0CTL			;Clear Interrupt flag

;----------------------------End Timer Setup-----------------------------------

            ; Disable low-power mode
            bic.w   #LOCKLPM5,&PM5CTL0
            nop
            bis.w	#GIE, SR					;Enable global interrupt
            nop
        

main:
    call #i2c_write

main_loop:
    call #i2c_read             ; call i2c_write

delay_1s:
    mov.w   #60000, R15       ; Load counter (~1 sec delay)
delay_loop:
    dec.w   R15               ; Decrement counter
    jnz     delay_loop        ; Loop until R15 reaches 0
           
    jmp main_loop
            


;------------------------------------------------------------------------------
; Subroutines
;------------------------------------------------------------------------------
;---------Start i2c_init Subroutine--------------------------------------------

;---------End i2c_init Subroutine----------------------------------------------


;---------Start i2c_start Subroutine-------------------------------------------
i2c_start:
    bic.b   #BIT0, &P6OUT       ; set SDA low          
    nop
    bic.b   #BIT1, &P6OUT       ; pull clock low
    ret
;---------End i2c_start Subroutine---------------------------------------------


;---------Start i2c_stop Subroutine--------------------------------------------
i2c_stop:
    bic.b   #BIT0, &P6OUT
    bis.b   #BIT1, &P6OUT        ; ensure SCL high
    nop
    bis.b   #BIT0, &P6OUT        ; Set SDA high
    ret
;---------End i2c_stop Subroutine----------------------------------------------

;---------Start i2c_tx_byte Subroutine-----------------------------------------
i2c_tx_byte:
    mov.b   R6, R7                 ; Copy byte to send
    mov.b   #8, R6                 ; 8 bits to transmit

send_byte_loop:
    rlc.b   R7                     ; Rotate left, MSB moves into carry
    jc      set_sda_high_tx         ; If carry is set, set SDA high
    bic.b   #BIT0, &P6OUT           ; Else, SDA low
    jmp     clk_pulse_byte

set_sda_high_tx:
    bis.b   #BIT0, &P6OUT           ; SDA high

clk_pulse_byte:
    bis.b   #BIT1, &P6OUT           ; SCL high
    call    #i2c_scl_delay
    bic.b   #BIT1, &P6OUT           ; SCL low
    call    #i2c_scl_delay

    dec.b   R6
    jnz     send_byte_loop          ; Loop until all bits sent

; Wait for ACK
    bic.b   #BIT0, &P6DIR           ; Set SDA as input (release line)
    nop
    bis.b   #BIT1, &P6OUT           ; SCL high
    call    #i2c_scl_delay

    bit.b   #BIT0, &P6IN            ; Check if SDA is high (no ACK)
    jnz     nack_handler_data       ; If no ACK, handle error

    bic.b   #BIT1, &P6OUT           ; SCL low
    call    #i2c_scl_delay
    bis.b   #BIT0, &P6DIR           ; Set SDA back as output
    ret

nack_handler_data:
    call    #i2c_stop               ; Stop if no ACK
    ret
;---------End i2c_tx_byte Subroutine-------------------------------------------

;---------Start i2c_rx_byte Subroutine-----------------------------------------
i2c_rx_byte:
    bic.b   #BIT0, &P6DIR       ; Set SDA as input (release line)
    mov.b   #8, R7              ; 8 bits to receive
    clr.b   R6                  ; Clear R6 (store received byte)

receive_loop:
    bis.b   #BIT1, &P6OUT       ; SCL high (slave puts bit on SDA)
    call    #i2c_scl_delay      ; Delay for clock high

    bit.b   #BIT0, &P6IN        ; Read SDA
    rlc.b   R6                  ; Shift left, storing bit in R6

    bic.b   #BIT1, &P6OUT       ; SCL low
    call    #i2c_scl_delay      ; Delay for clock low

    dec.b   R7
    jnz     receive_loop        ; Loop for 8 bits

; Send ACK/NACK
    bis.b   #BIT0, &P6DIR       ; Set SDA as output
    cmp.b   #1, R5              ; Is this the last byte?
    jne     send_ack            ; If not last byte, send ACK
    bis.b   #BIT0, &P6OUT       ; Else send NACK (SDA high)
    jmp     clk_pulse_ack

send_ack:
    bic.b   #BIT0, &P6OUT       ; SDA low (ACK)

clk_pulse_ack:
    bis.b   #BIT1, &P6OUT       ; SCL high (send ACK/NACK)
    call    #i2c_scl_delay
    bic.b   #BIT1, &P6OUT       ; SCL low
    call    #i2c_scl_delay
ret
;---------End i2c_rx_byte Subroutine-------------------------------------------


;---------Start i2c_sda_delay Subroutine---------------------------------------
i2c_sda_delay:
   nop
   nop
   nop
    ret
;---------End i2c_sda_delay Subroutine-----------------------------------------


;---------Start i2c_scl_delay Subroutine---------------------------------------
i2c_scl_delay:
    nop
    nop
    nop
    ret
;---------End i2c_scl_delay Subroutine-----------------------------------------


;---------Start i2c_send_address Subroutine------------------------------------
i2c_send_address:

    mov.b   #8, R6            ; 8 bits to transmit

send_address_loop:
    rlc.b   R5                ; Rotate left, MSB moves into carry
    jc      set_sda_high       ; If carry is set, set SDA high
    bic.b   #BIT0, &P6OUT      ; Else, SDA low
    jmp     clk_pulse

set_sda_high:
    bis.b   #BIT0, &P6OUT      ; SDA high

clk_pulse:
    bis.b   #BIT1, &P6OUT      ; SCL high
    call    #i2c_scl_delay
    bic.b   #BIT1, &P6OUT      ; SCL low
    call    #i2c_scl_delay

    dec.b   R6
    jnz     send_address_loop  ; Loop until all bits sent

; Check for ACK
    bic.b   #BIT0, &P6DIR      ; Set SDA as input (release line)
    bis.b   #BIT1, &P6OUT      ; SCL high
    call    #i2c_scl_delay

    bit.b   #BIT0, &P6IN       ; Check if SDA is high (no ACK)
    jnz     nack_handler       ; If no ACK, handle error

    bic.b   #BIT1, &P6OUT      ; SCL low
    call    #i2c_scl_delay
    bis.b   #BIT0, &P6DIR      ; Set SDA back as output

    ret                        ; Return to calling function

nack_handler:
    call    #i2c_stop          ; Send stop if no ACK received
    ret

;--------End i2c_send_address Subroutine--------------------------------------


;---------Start i2c_write Subroutine-------------------------------------------
i2c_write:
    mov.b   #0xD0, R5               ; Load RTC write address (0x68 << 1 | 0)
    call    #i2c_start             ; call i2c_start
    call    #i2c_send_address
    mov.w   #tx_data, R4           ; move memory 
    mov.b   #5, R5                 ; # of bytes to transmit 
    
send_data_loop:
    mov.b   @R4+, R6               ; Load byte from buffer
    call    #i2c_tx_byte           ; Send byte
    dec.b   R5
    jnz     send_data_loop         ; Loop until all bytes sent

    call    #i2c_stop              ; Send STOP condition
    ret
;---------End i2c_write Subroutine---------------------------------------------


;---------Start i2c_read Subroutine--------------------------------------------
i2c_read:
    call    #i2c_start              ; Start condition
    mov.b   #0xD0, R5               ; RTC write address (0x68 << 1 | 0)
    call    #i2c_send_address        ; Send write address

    mov.b   #0x00, R6               ; Register address to start reading (seconds register)
    call    #i2c_tx_byte             ; Send register address

    call    #i2c_stop                ; Stop to end write phase
    call    #i2c_start               ; Repeated start for read

    mov.b   #0xD1, R5               ; RTC read address (0x68 << 1 | 1)
    call    #i2c_send_address        ; Send read address

    mov.w   #rx_data, R4            ; Store data in rx_data buffer
    mov.b   #5, R5                  ; Number of bytes to read (seconds, minutes, hours, temp MSB, temp LSB)

read_data_loop:
    call    #i2c_rx_byte             ; Receive one byte
    mov.b   R6, 0(R4)                ; Store received byte
    inc.b   R4                       ; Move buffer pointer
    dec.b   R5                       ; Decrement count
    jnz     read_data_loop            ; Repeat until all bytes received

    call    #i2c_stop                 ; Send STOP condition
    ret
;---------End i2c_read Subroutine----------------------------------------------
 
;------------------------------------------------------------------------------
; Memory Allocation
;------------------------------------------------------------------------------
.data
.retain
tx_data: .byte 00h, 01h, 02h, 11h, 12h

rx_data: .space 5



;------------------------------------------------------------------------------
; Interrupt Service Routines
;------------------------------------------------------------------------------


;--------------------------Start TimerB1_2s------------------------------------
TimerB0_1s:

	xor.b	#BIT0, &P1OUT		; Toggle P1.0(LED)
	bic.w	#TBIFG, &TB0CTL		;Clear interrupt flag

	reti
;-------------------------------End TimerB1_2s---------------------------------

;------------------------------------------------------------------------------
;           Interrupt Vectors
;------------------------------------------------------------------------------
            .sect   RESET_VECTOR            ; MSP430 RESET Vector
            .short  RESET                   ;

            .sect   ".int42"                ; TB0 vector
            .short  TimerB0_1s              ;
            .end
