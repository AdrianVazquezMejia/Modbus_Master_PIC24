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



INT_VAL dirIn1;
INT_VAL NoIn1;
INT_VAL Crc1;
INT_VAL InputRegister1[10][6];
INT_VAL OwnInputRegisters1[10][6];
INT_VAL HoldingRegister1[10][6];
INT_VAL CoilRegister1[10][6];
INT_VAL DiscreteInputRegister1[10][6];
uint8_t j1=1;
uint8_t SlaveID1 = 0;
bool WriteSuccess1;

void (*state_table1[120])(void)={SLAVEADDRESS,FUNCTION,BYTECOUNT, DATA,
    CRC1Hi, CRC1Lo,COILADDRESS5HI, COILADDRESS5LO, FORCEDATA5Hi,
    FORCEDATA5Lo, CRC5Hi, CRC5Lo,REGISTERADDRESS6HI, REGISTERADDRESS6LO,
    WRITEDATA6Hi, WRITEDATA6Lo, CRC6Hi, CRC6Lo,ESPERASINCRONISMO};




static uint8_t * volatile rxTail1;
//static uint8_t *rxHead;
//static uint8_t *txTail;
static uint8_t * volatile txHead1;
static bool volatile rxOverflowed1;
uint8_t *pint1;
uint8_t contTx1;
ModbusEstados curr_state1=SlaveAddress;
uint8_t buffRx1[100], buffTx1[100],n1,auxRx1;


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

void Com_MODBUS_Read1(uint8_t Slave, uint8_t Function, uint16_t StartingAddress, uint16_t Quantity)
{
        INT_VAL auxStartingAddress1, auxQuantity1;
        auxStartingAddress1.Val = StartingAddress;
        auxQuantity1.Val=Quantity;
        
        buffTx1[0]=Slave;
        buffTx1[1]=Function;
        buffTx1[2]=auxStartingAddress1.byte.HB;
        buffTx1[3]=auxStartingAddress1.byte.LB;
        buffTx1[4]=auxQuantity1.byte.HB;
        buffTx1[5]=auxQuantity1.byte.LB;
        Crc1.Val=CRC16(buffTx1,6);
		buffTx1[6]=Crc1.byte.LB;
		buffTx1[7]=Crc1.byte.HB;
        contTx1=8;
		pint1=buffTx1;                
		U1TXREG = *pint1;
}

/*
 * This function has the Write MODBUS functions
 */


void Com_MODBUS_Write1(uint8_t Slave, uint8_t Function, uint16_t Address, uint16_t Data)
{
        INT_VAL auxAddress1, auxData1;
        auxAddress1.Val=Address;
        auxData1.Val=Data;
    
        buffTx1[0]=Slave;
        buffTx1[1] = Function;
        buffTx1[2] = auxAddress1.byte.HB;
        buffTx1[3] = auxAddress1.byte.LB;
        buffTx1[4] = auxData1.byte.HB;
        buffTx1[5] = auxData1.byte.LB;
        Crc1.Val = CRC16(buffTx1,6);
		buffTx1[6] = Crc1.byte.LB;
		buffTx1[7] = Crc1.byte.HB;
        
        contTx1 = 8;
		pint1 = buffTx1;
        SlaveID1 = Slave;
		U1TXREG = *pint1;
}

void SLAVEADDRESS1(void)
{
    if(auxRx1 == SlaveID1){
                n1=0;
                buffRx1[n1++]  = auxRx1;
                curr_state1 = Function;
                LED=0;
                //U1TXREG = auxRx; 
            }
    else {
        curr_state1 =  EsperaSincronismo;
       // U1TXREG = auxRx;
    }    
}

void FUNCTION1(void)
{
    buffRx1[n1++]  = auxRx1;
    switch(auxRx1)
            {  
                case 1:
                   curr_state1 = ByteCount;
                break;
                
                case 2:
                    curr_state1 = ByteCount;
                break;
                         
                case 3:
                    curr_state1 = ByteCount;
                break;
                
                case 4:
                    curr_state1 = ByteCount;
                break;
                
                case 5:
                    curr_state1 = CoilAddress5HI;
                break;
                
                case 6:
                    curr_state1 = RegisterAddress6HI;
                break;
                         
                case 15:
                    curr_state1 =  EsperaSincronismo;
                break;
                
                case 16:
                    curr_state1 =  EsperaSincronismo;
                break; 
                
                default:
                    curr_state1 =  EsperaSincronismo;
                break;    
            
            }
}

void BYTECOUNT1(void)
{
    buffRx1[n1++]  = auxRx1;
    curr_state1 =  Data;
    j1=1;
}

void DATA1(void)
{
    buffRx1[n1++]  = auxRx1;
     if (j1==buffRx1[2])
        curr_state1=Crc1Hi;
    j1++;
}



void CRC1Hi1(void)
{
buffRx1[n1++] = auxRx1;
curr_state1=Crc1Lo;
}

void CRC1Lo1(void)
{
	buffRx1[n1++]  = auxRx1;
	curr_state1 =  EsperaSincronismo;
	//CRC
	if(CRC16 (buffRx1, n1)==0){
		// datos buenos crear respuesta  
		
		LED = !LED; // LED0 cambia cada vez que pasa
        

        
        for (j1=1;j1==buffRx1[1];++j1)
        switch(buffRx1[1])
        {
            case ReadCoils:
            
                CoilRegister1[buffRx1[0]][j1].Val = buffRx1[2+j1];
            break;
            case ReadDiscreteInputs:
                DiscreteInputRegister1[buffRx1[0]][j1].Val=buffRx1[2+j1];
            break;   
            case ReadHoldingRegisters:
                HoldingRegister1[buffRx1[0]][j1].Val=buffRx1[2+j1];
            break;
            case ReadInputRegisters:
                HoldingRegister1[buffRx1[0]][j1].Val=buffRx1[2+j1];
            break;
        }
}
}   


void COILADDRESS5HI1(void)
{	
	buffRx1[n1++]  = auxRx1;
	dirIn1.byte.HB = auxRx1;
	curr_state1 = CoilAddress5LO;
}

void COILADDRESS5LO1(void)
{	
	buffRx1[n1++]  = auxRx1;
	dirIn1.byte.LB = auxRx1;
    curr_state1 = ForceData5Hi;       
}
void  FORCEDATA5Hi1(void)
{
	buffRx1[n1++]  = auxRx1;
	NoIn1.byte.HB = auxRx1;
	curr_state1 = ForceData5Lo;
}    

void FORCEDATA5Lo1(void){
	
	buffRx1[n1++]  = auxRx1;
	NoIn1.byte.LB = auxRx1;
	curr_state1 =  Crc5Hi;
}

void CRC5Hi1(void)
{	
	buffRx1[n1++]  = auxRx1;
	Crc1.byte.HB = auxRx1;
	curr_state1 =  Crc5Lo;	
}

void CRC5Lo1(void){
	
	buffRx1[n1++]  = auxRx1;
	Crc1.byte.LB = auxRx1;
	curr_state1 =  EsperaSincronismo;
	//CRC
	if(CRC16 (buffRx1, n1)==0)
	{
		
        WriteSuccess1=true;
       for (j1=0;j1==7;j1++)
           if (buffRx1[j1]!=buffTx1[j1])
               WriteSuccess1=false;
        
	}           
}

void REGISTERADDRESS6HI1(void)
{	
	buffRx1[n1++]  = auxRx1;
	curr_state1 = RegisterAddress6LO;
}

void REGISTERADDRESS6LO1(void)
{	
	buffRx1[n1++]  = auxRx1;
	curr_state1 = WriteData6Hi;       
}        
void  WRITEDATA6Hi1(void)
{
	buffRx1[n1++]  = auxRx1;
	curr_state1 = WriteData6Lo;
}

void WRITEDATA6Lo1(void)
{  
	buffRx1[n1++]  = auxRx1;
	curr_state1 =  Crc6Hi;
}

void CRC6Hi1(void)
{	
	buffRx1[n1++]  = auxRx1;
	curr_state1 =  Crc6Lo;
}

void CRC6Lo1(void)
{	
	buffRx1[n1++]  = auxRx1;
	curr_state1 =  EsperaSincronismo;
	//CRC
	if(CRC16 (buffRx1, n1)==0)
	{
        WriteSuccess1=true;
       for (j1=0;j1==7;j1++)
           if (buffRx1[j1]!=buffTx1[j1])
               WriteSuccess1=false;
        
	}           
}

void ESPERASINCRONISMO1(void)
{	// este estado no hace nada espera Timer 2 lo saque de aqui	
}
/*
 *This functions starts the module of modbus by trying to comunicate with each 
 * slave asking for its Holding registers 
 */

void Com_MODBUS_Init1(void)
{
   

}




/**
    Maintains the driver's transmitter state machine and implements its ISR
*/


void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1TXInterrupt ( void )
{
    
    PORTAbits.RA1=!PORTAbits.RA1;
    pint1++;
    if(--contTx1>0) U1TXREG = *pint1; 
    IFS0bits.U1TXIF = false;
    
}




void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1RXInterrupt( void )
{
    TMR2 = 0x00;
    auxRx1 = U1RXREG;
    LED = 1;
    state_table1[curr_state1]();
    IFS0bits.U1RXIF = false;
}

