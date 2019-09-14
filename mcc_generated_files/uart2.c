
#include <stdbool.h>
#include <stdint.h>
#include "xc.h"
#include "uart2.h"
#include "CRC.h"
#include "pin_manager.h"



INT_VAL dirIn;
INT_VAL NoIn;
INT_VAL Crc;
INT_VAL InputRegister[10][6];
INT_VAL OwnInputRegisters[10][6];
INT_VAL HoldingRegister[10][6];
INT_VAL CoilRegister[10][6];
INT_VAL DiscreteInputRegister[10][6];
uint8_t j=1;
uint8_t SlaveID = 0;
bool WriteSuccess;

void (*state_table[120])(void)={SLAVEADDRESS,FUNCTION,BYTECOUNT, DATA,
    CRC1Hi, CRC1Lo,COILADDRESS5HI, COILADDRESS5LO, FORCEDATA5Hi,
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
/*
 * This function has the Read MODBUS functions
 */
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
