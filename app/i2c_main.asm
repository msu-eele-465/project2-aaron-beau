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

    mov.w   #60000, R15       ; Load counter (~1 sec delay)
delay_loop:
    dec.w   R15               ; Decrement counter
    jnz     delay_loop        ; Loop until R15 reaches 0
           
    jmp main_loop
            


;------------------------------------------------------------------------------
; Subroutines
;------------------------------------------------------------------------------

;---------Start i2c_start Subroutine-------------------------------------------
;Condition to start an I2C transaction.  Executed by setting SDA low then pulling
;the clock high while the data line is low indicating that an address is about to
;be sent
i2c_start:
    bic.b   #BIT0, &P6OUT       ; set SDA low          
    nop
    bic.b   #BIT1, &P6OUT       ; pull clock low
    ret
;---------End i2c_start Subroutine---------------------------------------------


;---------Start i2c_stop Subroutine--------------------------------------------
;Condition to end any transaction through I2C.  Executed by first ensuring SDA is low,
;pulling the clock high then while the clock is high, pulling the data line high.
i2c_stop:
    bic.b   #BIT0, &P6OUT        ; ensure SDA LoW
    bis.b   #BIT1, &P6OUT        ; ensure SCL high
    nop
    bis.b   #BIT0, &P6OUT        ; Set SDA high
    ret
;---------End i2c_stop Subroutine----------------------------------------------

;---------Start i2c_tx_byte Subroutine-----------------------------------------
;The information that was placed into a register in the write subroutine, is copied
;into another register.  R6 is loaded with 8 to represent the number of bits within a byte
;to ensure that each bit of the packet is read.  The byte is rotated through and the carry
;bit is checked.  If the carry is set then the data line is set high and the clock toggled.
;This is repeated until all 8 bits of the byte have been rotated through.  After the entire
;has been sent, then the dta line is set as an input to look for an ack from the slave. 
;A brief delay is executed to give the slave amble time to send this ack.  The state of the 
;data line is checked and if it is high, an ack has been sent the clock is set low, data line
;set as an input and the subroutine left.  If the data line is found to not be high, meaning nack,
;then a stop condition is sent to terminate the write and the subroutine left.
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
;The data line is set as an input to receive the bytes from the slave.  A register
;is set to a value of 8 representing the number of bits within each byte.  
;The clock is set high and a delay called to allow for the slave to pull SDA high.
;The data line state is checked and rotated through storing the bits within register 6.
;The clock is pulled low and the processes repeated for all 8 bits within a byte.  Once
;all 8 bits have been sent, an acknowledge or nack is sent.  If it is not the last byte to 
;to be received then an acknowledge will be sent.  If all 5 bytes have been sent then a nack will
;sent to end the read transaction and the subroutine will be left.
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
;simple delay subroutine to be implemented when a signal needs to be held for a 
;period for a transaction to execute
i2c_sda_delay:
   nop
   nop
   nop
    ret
;---------End i2c_sda_delay Subroutine-----------------------------------------


;---------Start i2c_scl_delay Subroutine---------------------------------------
;simple delay subroutine to be implemented when a signal needs to be held for a 
;period for a transaction to execute
i2c_scl_delay:
    nop
    nop
    nop
    ret
;---------End i2c_scl_delay Subroutine-----------------------------------------


;---------Start i2c_send_address Subroutine------------------------------------
i2c_send_address:
;Within one byte there are 8 bits to send so this value is placed within a register to be tracked.
;The address that was loaded in the write/read routine is rotated left so that thhat each bit can be read 
;and determine if it a read or write.  Depending on the type of transaction and state of carry, the data line 
;will be toggled. Each time a bit is read the clock with be toggled.  This will continue until all 8 bits are sent.
;An acknowlege statement will then be check for from the slave by setting the the data line as an input and setting the
;clock high.  A brief delay is set to allow the slave time to send this transaction.  The state of the data line is 
;then checked to see if an ack was sent.  If an ack was sent the subroutine will be left.  If no ack is detected
;then a stop condition will be sent and the subroutine left.
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
;The address of the RTC we are writing to is 68h.  Using the hex value of D0, which in binary is 68
;with the write bit.  This address is then sent to the send address routine.  Once the address is sent
;the bytes of data reserved in tx_data are moved to a register to be prepared to be sent.  The number of 
;bytes to be sent is set, and the data is sent by looking at the memory address of the register they are saved in.
;This is executed until all bytes have been sent then the transaction is terminated through a stop.
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
;A start condition is called followed by the write address.  The first register 
;to be read from the slave is then sent and a stop condition sent.  A repeated start
;is sent followed by the read address.  The number of bytes to be read is set and the 
;read subroutine called.  The data is then collected into a register and stored in the memory
;that was allocated by rx_data.  This is repeated until all five bytes of desired data have been read
;being seconds, minutes, hours, and both bytes of temperature data.  A stop condition is then called and the
;subroutine is left.
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
tx_data: .byte 00h, 01h, 02h, 11h, 12h      ;address to be written to (sec, min, hr, MSB temp, LSB temp)

rx_data: .space 5                           ; space for values to be saved



;------------------------------------------------------------------------------
; Interrupt Service Routines
;------------------------------------------------------------------------------


;--------------------------Start TimerB1_2s------------------------------------
;Interrupt gives visual that code is indeed operating
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
