/**
  UART2 Generated Driver File 

  @Company
    Microchip Technology Inc.

  @File Name
    uart2.c

  @Summary
    This is the generated source file for the UART2 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for driver for UART2. 
    Generation Information : 
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.125
        Device            :  PIC24FJ64GA002
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.36B
        MPLAB             :  MPLAB X v5.20
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
/*The purpose of this UART is the comunication with the other using 
 * MODBUS. This one being the master
 
 */

/**
  Section: Included Files
*/
#include <stdbool.h>
#include <stdint.h>
#include "xc.h"
#include "uart2.h"
#include "CRC.h"
#include "pin_manager.h"

#define SlaveID 1
/**
  Section: Data Type Definitions
*/

/** UART Driver Queue Status

  @Summary
    Defines the object required for the status of the queue.
*/
INT_VAL dirIn;
INT_VAL NoIn;
INT_VAL Crc;
INT_VAL InputRegister[10];
INT_VAL HoldingRegister[10];
INT_VAL CoilRegister;
INT_VAL DiscreteInputRegister;

void (*state_table[120])(void)={SLAVEADDRESS,FUNCTION,STARTINGADDRESS1HI, STARTINGADDRESS1LO,
    NOREGISTER1Hi, NOREGISTER1Lo, CRC1Hi, CRC1Lo,STARTINGADDRESS2HI,
    STARTINGADDRESS2LO, NOREGISTER2Hi, NOREGISTER2Lo, CRC2Hi, CRC2Lo,        
    STARTINGADDRESS3HI, STARTINGADDRESS3LO, NOREGISTER3Hi, NOREGISTER3Lo,
    CRC3Hi, CRC3Lo, STARTINGADDRESS4HI, STARTINGADDRESS4LO, NOREGISTER4Hi,
    NOREGISTER4Lo, CRC4Hi, CRC4Lo,COILADDRESS5HI, COILADDRESS5LO, FORCEDATA5Hi,
    FORCEDATA5Lo, CRC5Hi, CRC5Lo,REGISTERADDRESS6HI, REGISTERADDRESS6LO,
    WRITEDATA6Hi, WRITEDATA6Lo, CRC6Hi, CRC6Lo,ESPERASINCRONISMO};




static uint8_t * volatile rxTail;
static uint8_t *rxHead;
static uint8_t *txTail;
static uint8_t * volatile txHead;
static bool volatile rxOverflowed;
uint8_t *pint;
uint8_t contTx;
ModbusEstados curr_state=SlaveAddress;
uint8_t buffRx[100], buffTx[100],n,auxRx;
/** UART Driver Queue Length

  @Summary
    Defines the length of the Transmit and Receive Buffers

*/

#define UART2_CONFIG_TX_BYTEQ_LENGTH (8+1)
#define UART2_CONFIG_RX_BYTEQ_LENGTH (8+1)

/** UART Driver Queue

  @Summary
    Defines the Transmit and Receive Buffers

*/

static uint8_t txQueue[UART2_CONFIG_TX_BYTEQ_LENGTH];
static uint8_t rxQueue[UART2_CONFIG_RX_BYTEQ_LENGTH];

void (*UART2_TxDefaultInterruptHandler)(void);
void (*UART2_RxDefaultInterruptHandler)(void);

/**
  Section: Driver Interface
*/

void UART2_Initialize(void)
{
    IEC1bits.U2TXIE = 0;
    IEC1bits.U2RXIE = 0;

    // STSEL 1; IREN disabled; PDSEL 8N; UARTEN enabled; RTSMD disabled; USIDL disabled; WAKE disabled; ABAUD disabled; LPBACK disabled; BRGH enabled; RXINV disabled; UEN TX_RX; 
    // Data Bits = 8; Parity = None; Stop Bits = 1;
    U2MODE = (0x8008 & ~(1<<15));  // disabling UART ON bit
    // UTXISEL0 TX_ONE_CHAR; UTXINV disabled; OERR NO_ERROR_cleared; URXISEL RX_ONE_CHAR; UTXBRK COMPLETED; UTXEN disabled; ADDEN disabled; 
    U2STA = 0x00;
    // BaudRate = 9600; Frequency = 16000000 Hz; BRG 416; 
    U2BRG = 0x1A0;
    
//    txHead = txQueue;
//    txTail = txQueue;
//    rxHead = rxQueue;
//    rxTail = rxQueue;
//   
//    rxOverflowed = 0;
//
//    UART2_SetTxInterruptHandler(UART2_Transmit_ISR);
//
//    UART2_SetRxInterruptHandler(UART2_Receive_ISR);

    IEC1bits.U2RXIE = 1;
    
    //Make sure to set LAT bit corresponding to TxPin as high before UART initialization
    U2MODEbits.UARTEN = 1;   // enabling UART ON bit
    U2STAbits.UTXEN = 1;
    
    IFS1bits.U2TXIF = false;	// Clear the Transmit Interrupt Flag 
    IEC1bits.U2TXIE = 1;	// 1 Enable Transmit Interrupts
}

/**
    Maintains the driver's transmitter state machine and implements its ISR
*/

void UART2_SetTxInterruptHandler(void* handler)
{
    UART2_TxDefaultInterruptHandler = handler;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U2TXInterrupt ( void )
{
   PORTAbits.RA0=!PORTAbits.RA0;
  pint++;
    if(--contTx>0) U2TXREG = *pint; 
    
    IFS1bits.U2TXIF = false;
}

void UART2_Transmit_ISR ( void )
{ 
    if(txHead == txTail)
    {
        IEC1bits.U2TXIE = 0;
        return;
    }

    IFS1bits.U2TXIF = 0;

    while(!(U2STAbits.UTXBF == 1))
    {
        U2TXREG = *txHead++;

        if(txHead == (txQueue + UART2_CONFIG_TX_BYTEQ_LENGTH))
        {
            txHead = txQueue;
        }

        // Are we empty?
        if(txHead == txTail)
        {
            break;
        }
    }
}

void UART2_SetRxInterruptHandler(void* handler)
{
    UART2_RxDefaultInterruptHandler = handler;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U2RXInterrupt( void )
{

    TMR1 = 0x00;
    auxRx = U2RXREG;
    LED=1;
    state_table[curr_state]();
    IFS1bits.U2RXIF = false;
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
                   curr_state = StartingAddress1HI;
                break;
                
                case 2:
                    curr_state = StartingAddress2HI;
                break;
                         
                case 3:
                    curr_state = StartingAddress3HI;
                break;
                
                case 4:
                    curr_state = StartingAddress4HI;
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

void STARTINGADDRESS1HI(void)
{
    buffRx[n++]  = auxRx;
    dirIn.byte.HB = auxRx;
    curr_state =  StartingAddress1LO;
}

void STARTINGADDRESS1LO(void)
{
    buffRx[n++]  = auxRx;
    dirIn.byte.LB = auxRx;
    curr_state=  NoRegisters1Hi;
}

void NOREGISTER1Hi(void)
{        
    buffRx[n++]  = auxRx;
    NoIn.byte.HB = auxRx;
    curr_state =  NoRegisters1Lo;
}

void NOREGISTER1Lo(void)
{        
    buffRx[n++]  = auxRx;
    NoIn.byte.LB = auxRx;
    curr_state =  Crc1Hi;
}

void CRC1Hi(void)
{
buffRx[n++] = auxRx;
Crc.byte.HB = auxRx;
curr_state=Crc1Lo;
}

void CRC1Lo(void)
{
	buffRx[n++]  = auxRx;
	Crc.byte.LB = auxRx;
	curr_state=  EsperaSincronismo;
	//CRC
	if(CRC16 (buffRx, n)==0){
		// datos buenos crear respuesta  
		
		LED = !LED; // LED0 cambia cada vez que pasa
		
		buffTx[0]=buffRx[0];
		buffTx[1]=buffRx[1];
		buffTx[2]=1;
		buffTx[3]=CoilRegister.byte.LB;

		Crc.Val=CRC16(buffTx,4);
		buffTx[4]=Crc.byte.LB;
		buffTx[5]=Crc.byte.HB;
		
		// transmite respuesta
		contTx=6;
		pint=buffTx;                
		U2TXREG = *pint;

}
}   
void STARTINGADDRESS2HI(void)
{
    buffRx[n++]  = auxRx;
	dirIn.byte.HB = auxRx;
	curr_state =  StartingAddress2LO;
}

void STARTINGADDRESS2LO(void)
{
	buffRx[n++]  = auxRx;
	dirIn.byte.LB = auxRx;
	curr_state =  NoRegisters2Hi;       
}

void  NOREGISTER2Hi(void)
{	
	buffRx[n++]  = auxRx;
	NoIn.byte.HB = auxRx;
	curr_state =  NoRegisters2Lo;
}

void NOREGISTER2Lo(void)
{	
	buffRx[n++]  = auxRx;
	NoIn.byte.LB = auxRx;
	curr_state =  Crc2Hi;	
}

void CRC2Hi(void)
{	
	buffRx[n++]  = auxRx;
	Crc.byte.HB = auxRx;
	curr_state = Crc2Lo;	
}

void CRC2Lo(void)
{	
	buffRx[n++]  = auxRx;
	Crc.byte.LB = auxRx;
	curr_state =  EsperaSincronismo;
	//CRC
	if(CRC16 (buffRx, n)==0){
		// datos buenos crear respuesta  
		
		LED=!LED; // LED0 cambia cada vez que pasa
		
		buffTx[0]=buffRx[0];
		buffTx[1]=buffRx[1];
		buffTx[2]=1;
		
		buffTx[3]=DiscreteInputRegister.byte.LB;

		Crc.Val=CRC16(buffTx,4);
		buffTx[4]=Crc.byte.LB;
		buffTx[5]=Crc.byte.HB;
		
		// transmite respuesta
		contTx=6;
		pint=buffTx;                
		U2TXREG = *pint;

	}
}




void STARTINGADDRESS3HI(void)
{
    buffRx[n++]  = auxRx;
    dirIn.byte.HB = auxRx;
    curr_state =  StartingAddress3LO;
}

void STARTINGADDRESS3LO(void)
{	
	buffRx[n++]  = auxRx;
	dirIn.byte.LB = auxRx;
	curr_state = NoRegisters3Hi;
}        

void NOREGISTER3Hi(void)
{	
	buffRx[n++]  = auxRx;
	NoIn.byte.HB = auxRx;
	curr_state =  NoRegisters3Lo;	
}

void NOREGISTER3Lo(void)
{	
	buffRx[n++]  = auxRx;
	NoIn.byte.LB = auxRx;
	curr_state =  Crc3Hi;	
}

void CRC3Hi(void)
{	
	buffRx[n++]  = auxRx;
	Crc.byte.HB = auxRx;
	curr_state =  Crc3Lo;
}

void CRC3Lo(void)
{	
	buffRx[n++]  = auxRx;
	Crc.byte.LB = auxRx;
     curr_state =  EsperaSincronismo;
	//CRC
	if(CRC16 (buffRx, n)==0){
		// datos buenos crear respuesta  
		
		LED=!LED; // LED0 cambia cada vez que pasa
		
		buffTx[0]=buffRx[0];
		buffTx[1]=buffRx[1];
		buffTx[2]=4;
		buffTx[3]=HoldingRegister[0].byte.HB;
		buffTx[4]=HoldingRegister[0].byte.LB;
		buffTx[5]=HoldingRegister[1].byte.HB;
		buffTx[6]=HoldingRegister[1].byte.LB;
		Crc.Val=CRC16(buffTx,7);
		buffTx[7]=Crc.byte.LB;
		buffTx[8]=Crc.byte.HB;
		
		// transmite respuesta
		contTx=9;
		pint=buffTx;                
		U2TXREG = *pint;

	} 

}

void STARTINGADDRESS4HI(void)
{	
	buffRx[n++]  = auxRx;
	dirIn.byte.HB = auxRx;
	curr_state =  StartingAddress4LO;
	
}

void STARTINGADDRESS4LO(void)
{	
	buffRx[n++]  = auxRx;
	dirIn.byte.LB = auxRx;
	curr_state =  NoRegisters4Hi;         
}  

void NOREGISTER4Hi(void)
{        
	buffRx[n++]  = auxRx;
	NoIn.byte.HB = auxRx;
	curr_state =  NoRegisters4Lo;
	
}

void NOREGISTER4Lo(void)
{
	buffRx[n++]  = auxRx;
	NoIn.byte.LB = auxRx;
	curr_state =  Crc4Hi;
}

void CRC4Hi(void)
{	
	buffRx[n++]  = auxRx;
	Crc.byte.HB = auxRx;
	curr_state=  Crc4Lo;
	
}

void CRC4Lo(void)
{	
	buffRx[n++]  = auxRx;
	Crc.byte.LB = auxRx;
	curr_state =  EsperaSincronismo;
	//CRC
	if(CRC16 (buffRx, n)==0){
		// datos buenos crear respuesta  
		
		LED = !LED; // LED0 cambia cada vez que pasa
		
		buffTx[0]=buffRx[0];
		buffTx[1]=buffRx[1];
		buffTx[2]=4;
		buffTx[3]=InputRegister[0].byte.HB;
		buffTx[4]=InputRegister[0].byte.LB;
		buffTx[5]=InputRegister[1].byte.HB;
		buffTx[6]=InputRegister[1].byte.LB;
		Crc.Val=CRC16(buffTx,7);
		buffTx[7]=Crc.byte.LB;
		buffTx[8]=Crc.byte.HB;
		
		// transmite respuesta
		contTx=9;
		pint=buffTx;                
		U2TXREG = *pint;

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
		switch(dirIn.Val)
		{
			case 0:
				if(NoIn.Val==0xFF00) LATBbits.LATB0 = 1;
				if(NoIn.Val==0x0000) LATBbits.LATB0 = 0;                        
			break;
			case 1:
				if(NoIn.Val==0xFF00) LATBbits.LATB1 = 1;
				if(NoIn.Val==0x0000) LATBbits.LATB1 = 0;                        
			break;
			case 2:
				if(NoIn.Val==0xFF00) LATBbits.LATB2 = 1;
				if(NoIn.Val==0x0000) LATBbits.LATB2 = 0;                        
			break;
			case 3:
				if(NoIn.Val==0xFF00) LATBbits.LATB3 = 1;
				if(NoIn.Val==0x0000) LATBbits.LATB3 = 0;                        
			break;
		}        

		// datos buenos crear respuesta  
		LED = !LED; // LED0 cambia cada vez que pasa

		buffTx[0]=buffRx[0];
		buffTx[1]=buffRx[1];
		buffTx[2]=buffRx[2];
		buffTx[3]=buffRx[3];
		buffTx[4]=buffRx[4];
		buffTx[5]=buffRx[5];
		buffTx[6]=buffRx[6];
		buffTx[7]=buffRx[7];

		// transmite respuesta
		contTx=8;
		pint=buffTx;                
		U2TXREG = *pint; 
	}           
}

void REGISTERADDRESS6HI(void)
{	
	buffRx[n++]  = auxRx;
	dirIn.byte.HB = auxRx;
	curr_state = RegisterAddress6LO;
}

void REGISTERADDRESS6LO(void)
{	
	buffRx[n++]  = auxRx;
	dirIn.byte.LB = auxRx;
	curr_state = WriteData6Hi;       
}        
void  WRITEDATA6Hi(void)
{
	buffRx[n++]  = auxRx;
	NoIn.byte.HB = auxRx;
	curr_state = WriteData6Lo;
}

void WRITEDATA6Lo(void)
{  
	buffRx[n++]  = auxRx;
	NoIn.byte.LB = auxRx;
	curr_state =  Crc6Hi;
}

void CRC6Hi(void)
{	
	buffRx[n++]  = auxRx;
	Crc.byte.HB = auxRx;
	curr_state =  Crc6Lo;
}

void CRC6Lo(void)
{	
	buffRx[n++]  = auxRx;
	Crc.byte.LB = auxRx;
	curr_state =  EsperaSincronismo;
	//CRC
	if(CRC16 (buffRx, n)==0)
	{
		HoldingRegister[dirIn.Val].Val=NoIn.Val;
		 
		// datos buenos crear respuesta  
		LED = !LED; // LED0 cambia cada vez que pasa

		buffTx[0]=buffRx[0];
		buffTx[1]=buffRx[1];
		buffTx[2]=buffRx[2];
		buffTx[3]=buffRx[3];
		buffTx[4]=buffRx[4];
		buffTx[5]=buffRx[5];
		buffTx[6]=buffRx[6];
		buffTx[7]=buffRx[7];

		// transmite respuesta
		contTx=8;
		pint=buffTx;                
		U2TXREG = *pint; 
	}           
}

void ESPERASINCRONISMO(void)
{	// este estado no hace nada espera Timer 1 lo saque de aqui	
}

void UART2_Receive_ISR(void)
{

    while((U2STAbits.URXDA == 1))
    {
        *rxTail = U2RXREG;

        // Will the increment not result in a wrap and not result in a pure collision?
        // This is most often condition so check first
        if ( ( rxTail    != (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH-1)) &&
             ((rxTail+1) != rxHead) )
        {
            rxTail++;
        } 
        else if ( (rxTail == (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH-1)) &&
                  (rxHead !=  rxQueue) )
        {
            // Pure wrap no collision
            rxTail = rxQueue;
        } 
        else // must be collision
        {
            rxOverflowed = true;
        }

    }

    IFS1bits.U2RXIF = false;
}

void __attribute__ ( ( interrupt, no_auto_psv ) ) _U2ErrInterrupt( void )
{
    if ((U2STAbits.OERR == 1))
    {
        U2STAbits.OERR = 0;
    }
    
    IFS4bits.U2ERIF = 0;
}

/**
  Section: UART Driver Client Routines
*/

uint8_t UART2_Read( void)
{
    uint8_t data = 0;

    while (rxHead == rxTail )
    {
    }
    
    data = *rxHead;

    rxHead++;

    if (rxHead == (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH))
    {
        rxHead = rxQueue;
    }
    return data;
}

void UART2_Write( uint8_t byte)
{
    while(UART2_IsTxReady() == 0)
    {
    }

    *txTail = byte;

    txTail++;
    
    if (txTail == (txQueue + UART2_CONFIG_TX_BYTEQ_LENGTH))
    {
        txTail = txQueue;
    }

    IEC1bits.U2TXIE = 1;
}

bool UART2_IsRxReady(void)
{    
    return !(rxHead == rxTail);
}

bool UART2_IsTxReady(void)
{
    uint16_t size;
    uint8_t *snapshot_txHead = (uint8_t*)txHead;
    
    if (txTail < snapshot_txHead)
    {
        size = (snapshot_txHead - txTail - 1);
    }
    else
    {
        size = ( UART2_CONFIG_TX_BYTEQ_LENGTH - (txTail - snapshot_txHead) - 1 );
    }
    
    return (size != 0);
}

bool UART2_IsTxDone(void)
{
    if(txTail == txHead)
    {
        return (bool)U2STAbits.TRMT;
    }
    
    return false;
}


/*******************************************************************************

  !!! Deprecated API !!!
  !!! These functions will not be supported in future releases !!!

*******************************************************************************/

static uint8_t UART2_RxDataAvailable(void)
{
    uint16_t size;
    uint8_t *snapshot_rxTail = (uint8_t*)rxTail;
    
    if (snapshot_rxTail < rxHead) 
    {
        size = ( UART2_CONFIG_RX_BYTEQ_LENGTH - (rxHead-snapshot_rxTail));
    }
    else
    {
        size = ( (snapshot_rxTail - rxHead));
    }
    
    if(size > 0xFF)
    {
        return 0xFF;
    }
    
    return size;
}

static uint8_t UART2_TxDataAvailable(void)
{
    uint16_t size;
    uint8_t *snapshot_txHead = (uint8_t*)txHead;
    
    if (txTail < snapshot_txHead)
    {
        size = (snapshot_txHead - txTail - 1);
    }
    else
    {
        size = ( UART2_CONFIG_TX_BYTEQ_LENGTH - (txTail - snapshot_txHead) - 1 );
    }
    
    if(size > 0xFF)
    {
        return 0xFF;
    }
    
    return size;
}

unsigned int __attribute__((deprecated)) UART2_ReadBuffer( uint8_t *buffer ,  unsigned int numbytes)
{
    unsigned int rx_count = UART2_RxDataAvailable();
    unsigned int i;
    
    if(numbytes < rx_count)
    {
        rx_count = numbytes;
    }
    
    for(i=0; i<rx_count; i++)
    {
        *buffer++ = UART2_Read();
    }
    
    return rx_count;    
}

unsigned int __attribute__((deprecated)) UART2_WriteBuffer( uint8_t *buffer , unsigned int numbytes )
{
    unsigned int tx_count = UART2_TxDataAvailable();
    unsigned int i;
    
    if(numbytes < tx_count)
    {
        tx_count = numbytes;
    }
    
    for(i=0; i<tx_count; i++)
    {
        UART2_Write(*buffer++);
    }
    
    return tx_count;  
}

UART2_TRANSFER_STATUS __attribute__((deprecated)) UART2_TransferStatusGet (void )
{
    UART2_TRANSFER_STATUS status = 0;
    uint8_t rx_count = UART2_RxDataAvailable();
    uint8_t tx_count = UART2_TxDataAvailable();
    
    switch(rx_count)
    {
        case 0:
            status |= UART2_TRANSFER_STATUS_RX_EMPTY;
            break;
        case UART2_CONFIG_RX_BYTEQ_LENGTH:
            status |= UART2_TRANSFER_STATUS_RX_FULL;
            break;
        default:
            status |= UART2_TRANSFER_STATUS_RX_DATA_PRESENT;
            break;
    }
    
    switch(tx_count)
    {
        case 0:
            status |= UART2_TRANSFER_STATUS_TX_FULL;
            break;
        case UART2_CONFIG_RX_BYTEQ_LENGTH:
            status |= UART2_TRANSFER_STATUS_TX_EMPTY;
            break;
        default:
            break;
    }

    return status;    
}

uint8_t __attribute__((deprecated)) UART2_Peek(uint16_t offset)
{
    uint8_t *peek = rxHead + offset;
    
    while(peek > (rxQueue + UART2_CONFIG_RX_BYTEQ_LENGTH))
    {
        peek -= UART2_CONFIG_RX_BYTEQ_LENGTH;
    }
    
    return *peek;
}

bool __attribute__((deprecated)) UART2_ReceiveBufferIsEmpty (void)
{
    return (UART2_RxDataAvailable() == 0);
}

bool __attribute__((deprecated)) UART2_TransmitBufferIsFull(void)
{
    return (UART2_TxDataAvailable() == 0);
}

uint16_t __attribute__((deprecated)) UART2_StatusGet (void)
{
    return U2STA;
}

unsigned int __attribute__((deprecated)) UART2_TransmitBufferSizeGet(void)
{
    if(UART2_TxDataAvailable() != 0)
    { 
        if(txHead > txTail)
        {
            return(txHead - txTail);
        }
        else
        {
            return(UART2_CONFIG_TX_BYTEQ_LENGTH - (txTail - txHead));
        }
    }
    return 0;
}

unsigned int __attribute__((deprecated)) UART2_ReceiveBufferSizeGet(void)
{
    if(UART2_RxDataAvailable() != 0)
    {
        if(rxHead > rxTail)
        {
            return(rxHead - rxTail);
        }
        else
        {
            return(UART2_CONFIG_RX_BYTEQ_LENGTH - (rxTail - rxHead));
        } 
    }
    return 0;
}

void __attribute__((deprecated)) UART2_Enable(void)
{
    U2MODEbits.UARTEN = 1;
    U2STAbits.UTXEN = 1;
}

void __attribute__((deprecated)) UART2_Disable(void)
{
    U2MODEbits.UARTEN = 0;
    U2STAbits.UTXEN = 0;
}
