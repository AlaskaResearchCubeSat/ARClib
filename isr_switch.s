#include <msp430.h>
        
        
  .vectors
  .keep
    org    USCI_B0_VECTOR
    dw    __I2C_ISR 

; Go to code section.
        .psect  "ISR"
        .keep

__I2C_ISR proc
  ADD &UCB0IV, PC            ; Add offset to jump table
  RETI                       ; Vector 0: No interrupt
  JMP _BUS_I2C_ALIFG         ; Vector 2: ALIFG
  JMP _BUS_I2C_NACKIFG       ; Vector 4: NACKIFG
  JMP _BUS_I2C_STTIFG        ; Vector 6: STTIFG
  JMP _BUS_I2C_STPIFG        ; Vector 8: STPIFG
  JMP _BUS_I2C_RXIFG3        ; Vector 10: RXIFG3
  JMP _BUS_I2C_TXIFG3        ; Vector 12  TXIFG3
  JMP _BUS_I2C_RXIFG2        ; Vector 14  RXIFG2
  JMP _BUS_I2C_TXIFG2        ; Vector 16  TXIFG2
  JMP _BUS_I2C_RXIFG1        ; Vector 18  RXIFG1
  JMP _BUS_I2C_TXIFG1        ; Vector 20  TXIFG1
  JMP _BUS_I2C_RXIFG0        ; Vector 22  RXIFG0
  JMP _BUS_I2C_TXIFG0        ; Vector 24  TXIFG0
  JMP _BUS_I2C_BCNTIFG       ; Vector 26  BCNTIFG
  JMP _BUS_I2C_CLTOIFG     ; Vector 28  UCCLTOIFG
  JMP _BUS_I2C_BCNTIFG     ; Vector 30  UCBCNTIFG
  ;code should not get here
  RETI ; Return
  endproc



