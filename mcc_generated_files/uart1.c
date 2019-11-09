/**
  UART1 Generated Driver File 

  @Company
    Microchip Technology Inc.

  @File Name
    uart1.c

  @Summary
    This is the generated source file for the UART1 driver using Foundation Services Library

  @Description
    This source file provides APIs for driver for UART1. 
    Generation Information : 
        Product Revision  :  Foundation Services Library - pic24-dspic-pic32mm : v1.26
        Device            :  PIC24FJ64GA002
    The generated drivers are tested against the following:
        Compiler          :  XC16 1.30
        MPLAB 	          :  MPLAB X 3.45
*/



/* Propose of this UART :
 * 
 * This UART is for the TPC/IP comunication, it is connected to se virtual serial port
 * 
 */

#include <stdbool.h>
#include <stdint.h>
#include "xc.h"
#include "uart1.h"
#include "CRC.h"
#include "pin_manager.h"
#include"uart2.h"
/**
  Section: Data Type Definitions
*/



extern INT_VAL dirIn;
extern INT_VAL NoIn;
extern INT_VAL Crc;
extern INT_VAL InputRegister[10][6];
extern INT_VAL OwnInputRegisters[10][6];
extern INT_VAL HoldingRegister[10][6];
extern INT_VAL CoilRegister[10][6];
extern INT_VAL DiscreteInputRegister[10][6];
extern  uint8_t j;
extern  uint8_t SlaveID;
extern bool WriteSuccess;

extern void (*state_table[120])(void);




extern  uint8_t * volatile rxTail;
//static uint8_t *rxHead;
//static uint8_t *txTail;
extern  uint8_t * volatile txHead;
extern  bool volatile rxOverflowed;
extern  uint8_t *pint;
extern uint8_t contTx;
extern ModbusEstados curr_state;
extern  uint8_t buffRx, buffTx,n,auxRx;


#define UART1_CONFIG_TX_BYTEQ_LENGTH 8
#define UART1_CONFIG_RX_BYTEQ_LENGTH 8



//static uint8_t uart1_txByteQ[UART1_CONFIG_TX_BYTEQ_LENGTH] ;
//static uint8_t uart1_rxByteQ[UART1_CONFIG_RX_BYTEQ_LENGTH] ;



void UART1_Initialize (void)
{
   // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; RXINV disabled; UEN TX_RX; 
   U1MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit   
   // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
   U1STA = 0x0000;
   // BaudRate = 9600; Frequency = 16000000 Hz; BRG 416; 
   U1BRG = 0x01A0;
    
   IEC0bits.U1RXIE = 1;


    //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
   U1MODEbits.UARTEN = 1;  // enabling UART ON bit
   U1STAbits.UTXEN = 1;
   
   

}

extern void Com_MODBUS_Read(uint8_t Slave, uint8_t Function, uint16_t StartingAddress, uint16_t Quantity);


/*
 * This function has the Write MODBUS functions
 */


extern void Com_MODBUS_Write1(uint8_t Slave, uint8_t Function, uint16_t Address, uint16_t Data);


extern void SLAVEADDRESS(void);

extern void FUNCTION(void);


extern void BYTECOUNT(void);


void DATA(void);




extern void CRC1Hi(void);


extern void CRC1Lo(void);



extern void COILADDRESS5HI(void);


extern void COILADDRESS5LO(void);

extern void  FORCEDATA5Hi(void);
    

extern void FORCEDATA5Lo(void);
	
extern void CRC5Hi(void);


extern void CRC5Lo(void);


extern void REGISTERADDRESS6HI(void);


extern void REGISTERADDRESS6LO(void);
       
extern void  WRITEDATA6Hi(void);


extern void WRITEDATA6Lo(void);


extern void CRC6Hi(void);

extern void CRC6Lo(void);

extern void ESPERASINCRONISMO(void);
/*
 *This functions starts the module of modbus by trying to comunicate with each 
 * slave asking for its Holding registers 
 */

extern void Com_MODBUS_Init(void);





/**
    Maintains the driver's transmitter state machine and implements its ISR
*/


void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1TXInterrupt ( void )
{
    
    PORTAbits.RA1=!PORTAbits.RA1;
    pint++;
    if(--contTx>0) U1TXREG = *pint; 
    IFS0bits.U1TXIF = false;
    
}




void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1RXInterrupt( void )
{
    TMR2 = 0x00;
    auxRx = U1RXREG;
    LED = 1;
    state_table[curr_state]();
    IFS0bits.U1RXIF = false;
}

