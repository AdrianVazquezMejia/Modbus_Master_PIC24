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

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

/**
  Section: Included Files
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
/**
  Section: Data Type Definitions
*/

/** UART Driver Queue Status

  @Summary
    Defines the object required for the status of the queue.
*/



UART_BYTEQ_STATUS;

/** UART Driver Hardware Instance Object

  @Summary
    Defines the object required for the maintenance of the hardware instance.

*/


/** UART Driver Queue Length

  @Summary
    Defines the length of the Transmit and Receive Buffers

*/

#define UART1_CONFIG_TX_BYTEQ_LENGTH 8
#define UART1_CONFIG_RX_BYTEQ_LENGTH 8


/** UART Driver Queue

  @Summary
    Defines the Transmit and Receive Buffers

*/

static uint8_t uart1_txByteQ[UART1_CONFIG_TX_BYTEQ_LENGTH] ;
static uint8_t uart1_rxByteQ[UART1_CONFIG_RX_BYTEQ_LENGTH] ;



/**
  Section: Driver Interface
*/


void UART1_Initialize (void)
{
   // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; RXINV disabled; UEN TX_RX; 
   U1MODE = (0x8008 & ~(1<<15));  // disabling UARTEN bit   
   // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
   U1STA = 0x0000;
   // BaudRate = 9600; Frequency = 16000000 Hz; BRG 416; 
   U1BRG = 0x01A0;
    
   IEC0bits.U1RXIE = 1;
   UART1_SetRxInterruptHandler(UART1_Receive_ISR);
   UART1_SetTxInterruptHandler(UART1_Transmit_ISR);

    //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
   U1MODEbits.UARTEN = 1;  // enabling UART ON bit
   U1STAbits.UTXEN = 1;
   
   

}

void Com_MODBUS_Read(uint8_t Slave, uint8_t Function, uint16_t StartingAddress, uint16_t Quantity)
{
        INT_VAL auxStartingAddress, auxQuantity;
        auxStartingAddress.Val = StartingAddress;
        auxQuantity.Val=Quantity;
        
        buffTx[0]=Slave;
        buffTx[1]=Function;
        buffTx[2]=auxStartingAddress.byte.HB;
        buffTx[3]=auxStartingAddress.byte.LB;
        buffTx[4]=auxQuantity.byte.HB;
        buffTx[5]=auxQuantity.byte.LB;
        Crc.Val=CRC16(buffTx,6);
		buffTx[6]=Crc.byte.LB;
		buffTx[7]=Crc.byte.HB;
        
        contTx=8;
		pint=buffTx;                
		U2TXREG = *pint;
}

/*
 * This function has the Write MODBUS functions
 */


void Com_MODBUS_Write(uint8_t Slave, uint8_t Function, uint16_t Address, uint16_t Data)
{
        INT_VAL auxAddress, auxData;
        auxAddress.Val=Address;
        auxData.Val=Data;
    
        buffTx[0]=Slave;
        buffTx[1] = Function;
        buffTx[2] = auxAddress.byte.HB;
        buffTx[3] = auxAddress.byte.LB;
        buffTx[4] = auxData.byte.HB;
        buffTx[5] = auxData.byte.LB;
        Crc.Val = CRC16(buffTx,6);
		buffTx[6] = Crc.byte.LB;
		buffTx[7] = Crc.byte.HB;
        
        contTx = 8;
		pint = buffTx;
        SlaveID = Slave;
		U2TXREG = *pint;
}

void SLAVEADDRESS(void)
{
    if(auxRx == SlaveID){
                n=0;
                buffRx[n++]  = auxRx;
                curr_state = Function;
                LED=0;
                //U2TXREG = auxRx; 
            }
    else {
        curr_state =  EsperaSincronismo;
       // U2TXREG = auxRx;
    }    
}

void FUNCTION(void)
{
    buffRx[n++]  = auxRx;
    switch(auxRx)
            {  
                case 1:
                   curr_state = ByteCount;
                break;
                
                case 2:
                    curr_state = ByteCount;
                break;
                         
                case 3:
                    curr_state = ByteCount;
                break;
                
                case 4:
                    curr_state = ByteCount;
                break;
                
                case 5:
                    curr_state = CoilAddress5HI;
                break;
                
                case 6:
                    curr_state = RegisterAddress6HI;
                break;
                         
                case 15:
                    curr_state =  EsperaSincronismo;
                break;
                
                case 16:
                    curr_state =  EsperaSincronismo;
                break; 
                
                default:
                    curr_state =  EsperaSincronismo;
                break;    
            
            }
}

void BYTECOUNT(void)
{
    buffRx[n++]  = auxRx;
    curr_state =  Data;
    j=1;
}

void DATA(void)
{
    buffRx[n++]  = auxRx;
     if (j==buffRx[2])
        curr_state=Crc1Hi;
    j++;
}



void CRC1Hi(void)
{
buffRx[n++] = auxRx;
curr_state=Crc1Lo;
}

void CRC1Lo(void)
{
	buffRx[n++]  = auxRx;
	curr_state=  EsperaSincronismo;
	//CRC
	if(CRC16 (buffRx, n)==0){
		// datos buenos crear respuesta  
		
		LED = !LED; // LED0 cambia cada vez que pasa
        

        
        for (j=1;j==buffRx[1];++j)
        switch(buffRx[1])
        {
            case ReadCoils:
            
                CoilRegister[buffRx[0]][j].Val = buffRx[2+j];
            break;
            case ReadDiscreteInputs:
                DiscreteInputRegister[buffRx[0]][j].Val=buffRx[2+j];
            break;   
            case ReadHoldingRegisters:
                HoldingRegister[buffRx[0]][j].Val=buffRx[2+j];
            break;
            case ReadInputRegisters:
                HoldingRegister[buffRx[0]][j].Val=buffRx[2+j];
            break;
        }
}
}   


void COILADDRESS5HI(void)
{	
	buffRx[n++]  = auxRx;
	dirIn.byte.HB = auxRx;
	curr_state = CoilAddress5LO;
}

void COILADDRESS5LO(void)
{	
	buffRx[n++]  = auxRx;
	dirIn.byte.LB = auxRx;
    curr_state = ForceData5Hi;       
}
void  FORCEDATA5Hi(void)
{
	buffRx[n++]  = auxRx;
	NoIn.byte.HB = auxRx;
	curr_state = ForceData5Lo;
}    

void FORCEDATA5Lo(void){
	
	buffRx[n++]  = auxRx;
	NoIn.byte.LB = auxRx;
	curr_state =  Crc5Hi;
}

void CRC5Hi(void)
{	
	buffRx[n++]  = auxRx;
	Crc.byte.HB = auxRx;
	curr_state =  Crc5Lo;	
}

void CRC5Lo(void){
	
	buffRx[n++]  = auxRx;
	Crc.byte.LB = auxRx;
	curr_state =  EsperaSincronismo;
	//CRC
	if(CRC16 (buffRx, n)==0)
	{
		
        WriteSuccess=true;
       for (j=0;j==7;j++)
           if (buffRx[j]!=buffTx[j])
               WriteSuccess=false;
        
	}           
}

void REGISTERADDRESS6HI(void)
{	
	buffRx[n++]  = auxRx;
	curr_state = RegisterAddress6LO;
}

void REGISTERADDRESS6LO(void)
{	
	buffRx[n++]  = auxRx;
	curr_state = WriteData6Hi;       
}        
void  WRITEDATA6Hi(void)
{
	buffRx[n++]  = auxRx;
	curr_state = WriteData6Lo;
}

void WRITEDATA6Lo(void)
{  
	buffRx[n++]  = auxRx;
	curr_state =  Crc6Hi;
}

void CRC6Hi(void)
{	
	buffRx[n++]  = auxRx;
	curr_state =  Crc6Lo;
}

void CRC6Lo(void)
{	
	buffRx[n++]  = auxRx;
	curr_state =  EsperaSincronismo;
	//CRC
	if(CRC16 (buffRx, n)==0)
	{
        WriteSuccess=true;
       for (j=0;j==7;j++)
           if (buffRx[j]!=buffTx[j])
               WriteSuccess=false;
        
	}           
}

void ESPERASINCRONISMO(void)
{	// este estado no hace nada espera Timer 1 lo saque de aqui	
}
/*
 *This functions starts the module of modbus by trying to comunicate with each 
 * slave asking for its Holding registers 
 */

void Com_MODBUS_Init(void)
{
   

}




/**
    Maintains the driver's transmitter state machine and implements its ISR
*/


void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1TXInterrupt ( void )
{
    IFS1bits.U1RXIF = false;
}




void __attribute__ ( ( interrupt, no_auto_psv ) ) _U1RXInterrupt( void )
{
    IFS1bits.U1RXIF = false;
}

