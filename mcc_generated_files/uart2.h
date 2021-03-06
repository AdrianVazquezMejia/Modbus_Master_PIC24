

#ifndef _UART2_H
#define _UART2_H
#define OverLevel 10001
#define ON 0xFF00
#define OFF 0x0000



#include <stdbool.h>
#include <stdint.h>
#include <xc.h>
#include <stdlib.h>


typedef enum
{
    /*  */
    SlaveAddress=0, Function,ByteCount, Data, Crc1Hi, Crc1Lo, CoilAddress5HI, 
    CoilAddress5LO, ForceData5Hi, ForceData5Lo, Crc5Hi, Crc5Lo, RegisterAddress6HI,
    RegisterAddress6LO, WriteData6Hi, WriteData6Lo, Crc6Hi, Crc6Lo,EsperaSincronismo
}ModbusEstados;

typedef enum
{
    ReadCoils=1, ReadDiscreteInputs, ReadHoldingRegisters,ReadInputRegisters, 
    WriteCoil, WriteRegister
} ModbusFunctions;

typedef enum
{
    InValve, BypassValve, OutValve, CleanValve, Fan, Pump
} Coils;

typedef enum
{
    InFluid = 30001, OutFluid, Level
} InputRegisters;

typedef enum
{
    RTU1=1, RTU2, RTU3, RTU4, RTU5, RTU6, RTU7, RTU8, RTU9
} RTUS;


typedef struct
{
    uint16_t    b0:     1;
    uint16_t    b1:     1;
    uint16_t    b2:     1;
    uint16_t    b3:     1;
    uint16_t    b4:     1;
    uint16_t    b5:     1;
    uint16_t    b6:     1;
    uint16_t    b7:     1;
    uint16_t    b8:     1;
    uint16_t    b9:     1;
    uint16_t    b10:    1;
    uint16_t    b11:    1;
    uint16_t    b12:    1;
    uint16_t    b13:    1;
    uint16_t    b14:    1;
    uint16_t    b15:    1;
}WORD_BITS;

typedef union
{
    uint16_t Val;
    WORD_BITS   bits;
    struct
    {
        uint8_t LB;
        uint8_t HB;
    } byte;

} INT_VAL;

typedef union
{
    uint16_t bomba;
    WORD_BITS bits;
    struct
    {
        uint8_t LB;
        uint8_t HB;
    }byte;
}BOMBAS;

typedef union
{
    uint16_t tanque;
    WORD_BITS bits;
    struct
    {
        uint8_t LB;
        uint8_t HB;
    }byte;
}TANQUES;


typedef enum{
    escribir = 1, leer
}situacion;



void UART2_Initialize(void);
void SLAVEADDRESS(void);
void FUNCTION(void);
void BYTECOUNT(void);
void DATA(void);
void REGISTERADDRESS6HI(void);
void REGISTERADDRESS6LO(void);
void WRITEDATA6Hi(void);
void WRITEDATA6Lo(void);
void ESPERASINCRONISMO(void);
void COILADDRESS5HI(void);
void COILADDRESS5LO(void);
void FORCEDATA5Lo(void);
void FORCEDATA5Hi(void);
void NOREGISTER1Lo(void);
void CRC1Hi(void);
void CRC1Lo(void);
void CRC5Hi(void);
void CRC5Lo(void);
void CRC6Hi(void);
void CRC6Lo(void);



void Com_MODBUS_Init(void);
void Com_MODBUS_Read(uint8_t Slave, uint8_t Function, uint16_t StartingAddress, uint16_t Quantity);
void Com_MODBUS_Write(uint8_t Slave, uint8_t Function, uint16_t Address, uint16_t Data);

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif
/*
  Section: UART2 Driver Routines
*/


/**
  @Summary
    Initializes the UART instance : 2

  @Description
    This routine initializes the UART driver instance for : 2
    index.
    This routine must be called before any other UART routine is called.
    
  @Preconditions
    None.

  @Returns
    None.

  @Param
    None.

  @Comment
    None.    
 
  @Example
    None.

**/

void UART2_Initialize(void);

/**
  @Summary
    Read a byte of data from the UART2

  @Description
    This routine reads a byte of data from the UART2.

  @Preconditions
    UART2_Initialize function should have been called 
    before calling this function. The transfer status should be checked to see 
    if the receiver is not empty before calling this function.

  @Param
    None.

  @Returns
    A data byte received by the driver.

  @Example
    None.
*/

uint8_t UART2_Read( void);

/**
  @Summary
    Writes a byte of data to the UART2

  @Description
    This routine writes a byte of data to the UART2.

  @Preconditions
    UART2_Initialize function should have been called 
    before calling this function. The transfer status should be checked to see if
    transmitter is not full before calling this function.

  @Param
    byte         - Data byte to write to the UART2

  @Returns
    None.

  @Example
    None.
*/

void UART2_Write( uint8_t byte);


/**
  @Description
    Indicates of there is data available to read.

  @Returns
    true if byte can be read.
    false if byte can't be read right now.
*/
bool UART2_IsRxReady(void);

/**
  @Description
    Indicates if a byte can be written.
 
 @Returns
    true if byte can be written.
    false if byte can't be written right now.
*/
bool UART2_IsTxReady(void);

/**
  @Description
    Indicates if all bytes have been transferred.
 
 @Returns
    true if all bytes transfered.
    false if there is still data pending to transfer.
*/
bool UART2_IsTxDone(void);

/**
  @Summary
    Assigns a function pointer with a transmit callback address.

  @Description
    This routine assigns a function pointer with a transmit callback address.

  @Param
    Address of the callback routine.

  @Returns
    None
 
  @Example 
    <code>
        UART2_SetTxInterruptHandler(&UART2_Transmit_ISR);
    </code>
*/
void UART2_SetTxInterruptHandler(void* handler);

/**
  @Summary
    Transmit callback routine.

  @Description
    This routine is a transmit callback function.

  @Param
    None.

  @Returns
    None
 
  @Example 
    <code>
        UART2_SetTxInterruptHandler(&UART2_Transmit_ISR);
    </code>
*/
void UART2_Transmit_ISR(void);

/**
  @Summary
    Assigns a function pointer with a receive callback address.

  @Description
    This routine assigns a function pointer with a receive callback address.

  @Param
    Address of the callback routine.

  @Returns
    None
 
  @Example 
    <code>
        UART2_SetRxInterruptHandler(&UART2_Receive_ISR);
    </code>
*/
void UART2_SetRxInterruptHandler(void* handler);

/**
  @Summary
    Receive callback routine.

  @Description
    This routine is a receive callback function.

  @Param
    None.

  @Returns
    None
 
  @Example 
    <code>
        UART2_SetTxInterruptHandler(&UART2_Receive_ISR);
    </code>
*/
void UART2_Receive_ISR(void);


/*******************************************************************************

  !!! Deprecated API and types !!!
  !!! These functions will not be supported in future releases !!!

*******************************************************************************/

/** UART2 Driver Hardware Flags

  @Summary
    Specifies the status of the hardware receive or transmit

  @Description
    This type specifies the status of the hardware receive or transmit.
    More than one of these values may be OR'd together to create a complete
    status value.  To test a value of this type, the bit of interest must be
    AND'ed with value and checked to see if the result is non-zero.
*/
typedef enum
{
    /* Indicates that Receive buffer has data, at least one more character can be read */
    UART2_RX_DATA_AVAILABLE = (1 << 0),
    /* Indicates that Receive buffer has overflowed */
    UART2_RX_OVERRUN_ERROR = (1 << 1),
    /* Indicates that Framing error has been detected for the current character */
    UART2_FRAMING_ERROR = (1 << 2),
    /* Indicates that Parity error has been detected for the current character */
    UART2_PARITY_ERROR = (1 << 3),
    /* Indicates that Receiver is Idle */
    UART2_RECEIVER_IDLE = (1 << 4),
    /* Indicates that the last transmission has completed */
    UART2_TX_COMPLETE = (1 << 8),
    /* Indicates that Transmit buffer is full */
    UART2_TX_FULL = (1 << 9) 
}UART2_STATUS;

/** UART2 Driver Transfer Flags

  @Summary
    Specifies the status of the receive or transmit

  @Description
    This type specifies the status of the receive or transmit operation.
    More than one of these values may be OR'd together to create a complete
    status value.  To test a value of this type, the bit of interest must be
    AND'ed with value and checked to see if the result is non-zero.
*/

typedef enum
{
    /* Indicates that the core driver buffer is full */
    UART2_TRANSFER_STATUS_RX_FULL = (1 << 0) ,
    /* Indicates that at least one byte of Data has been received */
    UART2_TRANSFER_STATUS_RX_DATA_PRESENT = (1 << 1) ,
    /* Indicates that the core driver receiver buffer is empty */
    UART2_TRANSFER_STATUS_RX_EMPTY = (1 << 2) ,
    /* Indicates that the core driver transmitter buffer is full */
    UART2_TRANSFER_STATUS_TX_FULL = (1 << 3) ,
    /* Indicates that the core driver transmitter buffer is empty */
    UART2_TRANSFER_STATUS_TX_EMPTY = (1 << 4) 
} UART2_TRANSFER_STATUS;

/**
  @Summary
    Returns the number of bytes read by the UART2 peripheral

  @Description
    This routine returns the number of bytes read by the Peripheral and fills the
    application read buffer with the read data.

  @Preconditions
    UART2_Initialize function should have been called 
    before calling this function

  @Param
    buffer       - Buffer into which the data read from the UART2

  @Param
    numbytes     - Total number of bytes that need to be read from the UART2
                   (must be equal to or less than the size of the buffer)

  @Returns
    Number of bytes actually copied into the caller's buffer or -1 if there
    is an error.

  @Example
    <code>
    char                     myBuffer[MY_BUFFER_SIZE];
    unsigned int             numBytes;
    UART2_TRANSFER_STATUS status ;

    // Pre-initialize myBuffer with MY_BUFFER_SIZE bytes of valid data.

    numBytes = 0;
    while( numBytes < MY_BUFFER_SIZE);
    {
        status = UART2_TransferStatusGet ( ) ;
        if (status & UART2_TRANSFER_STATUS_RX_FULL)
        {
            numBytes += UART2_ReadBuffer( myBuffer + numBytes, MY_BUFFER_SIZE - numBytes )  ;
            if(numBytes < readbufferLen)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        else
        {
            continue;
        }

        // Do something else...
    }
    </code>
*/
unsigned int __attribute__((deprecated)) UART2_ReadBuffer( uint8_t *buffer ,  unsigned int numbytes);

/**
  @Summary
    Returns the number of bytes written into the internal buffer

  @Description
    This API transfers the data from application buffer to internal buffer and 
    returns the number of bytes added in that queue

  @Preconditions
    UART2_Initialize function should have been called 
    before calling this function

  @Example
    <code>
    char                     myBuffer[MY_BUFFER_SIZE];
    unsigned int             numBytes;
    UART2_TRANSFER_STATUS status ;

    // Pre-initialize myBuffer with MY_BUFFER_SIZE bytes of valid data.

    numBytes = 0;
    while( numBytes < MY_BUFFER_SIZE);
    {
        status = UART2_TransferStatusGet ( ) ;
        if (status & UART2_TRANSFER_STATUS_TX_EMPTY)
        {
            numBytes += UART2_WriteBuffer ( myBuffer + numBytes, MY_BUFFER_SIZE - numBytes )  ;
            if(numBytes < writebufferLen)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        else
        {
            continue;
        }

        // Do something else...
    }
    </code>
*/
unsigned int __attribute__((deprecated)) UART2_WriteBuffer( uint8_t *buffer , unsigned int numbytes );

/**
  @Summary
    Returns the transmitter and receiver transfer status

  @Description
    This returns the transmitter and receiver transfer status.The returned status 
    may contain a value with more than one of the bits
    specified in the UART2_TRANSFER_STATUS enumeration set.  
    The caller should perform an "AND" with the bit of interest and verify if the
    result is non-zero (as shown in the example) to verify the desired status
    bit.

  @Preconditions
    UART2_Initialize function should have been called 
    before calling this function

  @Param
    None.

  @Returns
    A UART2_TRANSFER_STATUS value describing the current status 
    of the transfer.

  @Example
    Refer to UART2_ReadBuffer and UART2_WriteBuffer for example

*/
UART2_TRANSFER_STATUS __attribute__((deprecated)) UART2_TransferStatusGet (void );

/**
  @Summary
    Returns the character in the read sequence at the offset provided, without
    extracting it

  @Description
    This routine returns the character in the read sequence at the offset provided,
    without extracting it
 
  @Param
    None.
    
  @Example 
    <code>
    uint8_t readBuffer[5];
    unsigned int data, numBytes = 0;
    unsigned int readbufferLen = sizeof(readBuffer);
    UART2_Initialize();
    
    while(numBytes < readbufferLen)        
    {   
        UART2_TasksReceive ( );
        //Check for data at a particular place in the buffer
        data = UART2_Peek(3);
        if(data == 5)
        {
            //discard all other data if byte that is wanted is received.    
            //continue other operation
            numBytes += UART2_ReadBuffer ( readBuffer + numBytes , readbufferLen ) ;
        }
        else
        {
            break;
        }
    }
    </code>
 
*/
uint8_t __attribute__((deprecated)) UART2_Peek(uint16_t offset);

/**
  @Summary
    Returns the status of the receive buffer

  @Description
    This routine returns if the receive buffer is empty or not.

  @Param
    None.
 
  @Returns
    True if the receive buffer is empty
    False if the receive buffer is not empty
    
  @Example
    <code>
    char                     myBuffer[MY_BUFFER_SIZE];
    unsigned int             numBytes;
    UART2_TRANSFER_STATUS status ;

    // Pre-initialize myBuffer with MY_BUFFER_SIZE bytes of valid data.

    numBytes = 0;
    while( numBytes < MY_BUFFER_SIZE);
    {
        status = UART2_TransferStatusGet ( ) ;
        if (!UART2_ReceiveBufferIsEmpty())
        {
            numBytes += UART2_ReadBuffer( myBuffer + numBytes, MY_BUFFER_SIZE - numBytes )  ;
            if(numBytes < readbufferLen)
            {
                continue;
            }
            else
            {
                break;
            }
        }
        else
        {
            continue;
        }

        // Do something else...
    }
    </code>
 
*/
bool __attribute__((deprecated)) UART2_ReceiveBufferIsEmpty (void);

/**
  @Summary
    Returns the status of the transmit buffer

  @Description
    This routine returns if the transmit buffer is full or not.

 @Param
    None.
 
 @Returns
    True if the transmit buffer is full
    False if the transmit buffer is not full

 @Example
    Refer to UART2_Initialize() for example.
 
*/
bool __attribute__((deprecated)) UART2_TransmitBufferIsFull (void);

/**
  @Summary
    Returns the transmitter and receiver status

  @Description
    This returns the transmitter and receiver status. The returned status may 
    contain a value with more than one of the bits
    specified in the UART2_STATUS enumeration set.  
    The caller should perform an "AND" with the bit of interest and verify if the
    result is non-zero (as shown in the example) to verify the desired status
    bit.

  @Preconditions
    UART2_Initialize function should have been called 
    before calling this function

  @Param
    None.

  @Returns
    A UART2_STATUS value describing the current status 
    of the transfer.

  @Example
    <code>
        while(!(UART2_StatusGet & UART2_TX_COMPLETE ))
        {
           // Wait for the tranmission to complete
        }
    </code>
*/
uint16_t __attribute__((deprecated)) UART2_StatusGet (void );

/**
  @Summary
    Allows setting of a the enable bit for the UART2 mode

  @Description
    This routine is used to enable the UART2
  
  @Preconditions
    UART2_Initialize() function should have been 
    called before calling this function.
 
  @Returns
    None

  @Param
    None
  
  @Example
    Refer to UART2_Initialize(); for an example
*/

void __attribute__((deprecated)) UART2_Enable(void);

/**
  @Summary
    Allows setting of a the disable bit for the UART2 mode

  @Description
    This routine is used to disable the UART2
  
  @Preconditions
    UART2_Initialize() function should have been 
    called before calling this function.
 
  @Returns
    None

  @Param
    None
  
  @Example
    Refer to UART2_Initialize(); for an example
*/

void __attribute__((deprecated)) UART2_Disable(void);

/**
  @Summary
    Returns the number of bytes remaining in the receive buffer

  @Description
    This routine returns the number of bytes remaining in the receive buffer.

  @Param
    None.

  @Returns
    Remaining size of receive buffer.
    
  @Example 
    <code>
    uint8_t readBuffer[MY_BUFFER_SIZE];
    unsigned int size, numBytes = 0;
    UART2_Initialize();

    // Pre-initialize readBuffer with MY_BUFFER_SIZE bytes of valid data.
    
    while (size < MY_BUFFER_SIZE) {
        size = UART2_ReceiveBufferSizeGet();
    }
    numBytes = UART2_ReadBuffer(readBuffer, MY_BUFFER_SIZE);
    </code>
 
*/

unsigned int __attribute__((deprecated)) UART2_ReceiveBufferSizeGet(void);

/**
  @Summary
    Returns the number of bytes remaining in the transmit buffer.

  @Description
    This routine returns the number of bytes remaining in the transmit buffer.

 @Param
    None.
 
 @Returns
    Remaining size of transmit buffer.

 @Example
    Refer to UART2_Initialize(); for example.
*/

unsigned int __attribute__((deprecated)) UART2_TransmitBufferSizeGet(void);

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif
    
#endif  // _UART2_H