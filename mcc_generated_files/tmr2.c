
/**
  TMR2 Generated Driver API Source File 

  @Company
    Microchip Technology Inc.

  @File Name
    tmr2.c

  @Summary
    This is the generated source file for the TMR2 driver using PIC24 / dsPIC33 / PIC32MM MCUs

  @Description
    This source file provides APIs for driver for TMR2. 
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

/**
  Section: Included Files
*/

#include <stdio.h>
#include "tmr2.h"
#include "uart1.h"
#include "uart2.h"
#include "CRC.h"

extern ModbusEstados curr_state; 
int ss = 1;
static int i = 1;
static int j = 0;
int k = 0;
int cont = 1;
extern INT_VAL Bomba;
TANQUES tanques;
extern INT_VAL DicreteInputRegister[10][6];
extern INT_VAL HoldingRegister[10][6];
extern uint8_t buffTx[100], contTx, *pint;
extern INT_VAL Tanque[7];

/**
 Section: File specific functions
*/
void (*TMR2_InterruptHandler)(void) = NULL;
void TMR2_CallBack(void);

/**
  Section: Data Type Definitions
*/

/** TMR Driver Hardware Instance Object

  @Summary
    Defines the object required for the maintenance of the hardware instance.

  @Description
    This defines the object required for the maintenance of the hardware
    instance. This object exists once per hardware instance of the peripheral.

  Remarks:
    None.
*/

typedef struct _TMR_OBJ_STRUCT
{
    /* Timer Elapsed */
    volatile bool           timerElapsed;
    /*Software Counter value*/
    volatile uint8_t        count;

} TMR_OBJ;

static TMR_OBJ tmr2_obj;


/**
  Section: Driver Interface
*/

void TMR2_Initialize (void)
{
    //TMR2 0; 
    TMR2 = 0x00;
    //Period = 0.05 s; Frequency = 16000000 Hz; PR2 12499; 
    PR2 = 0x30D3;
    //TCKPS 1:64; T32 16 Bit; TON enabled; TSIDL disabled; TCS FOSC/2; TGATE enabled; 
    T2CON = 0x8020;
    
    if(TMR2_InterruptHandler == NULL)
    {
        TMR2_SetInterruptHandler(&TMR2_CallBack);
    }



    IFS0bits.T2IF = false;
    IEC0bits.T2IE = true;
    
    tmr2_obj.timerElapsed = false;
     /* Start the Timer */
    T2CONbits.TON = 1;
    
    
	
    

}




void __attribute__ ( ( interrupt, no_auto_psv ) ) _T2Interrupt (  )
{
    /* Check if the Timer Interrupt/Status is set */

    //***User Area Begin
    switch(ss){
        case  escribir:
            if (cont > 5){
                ss++;
                cont=1;
            } // voy ahora a leer 
            
                switch(cont){
            case RTU1:
                Com_MODBUS_Write(RTU1,WriteCoil,1,Bomba.bits.b1);
                cont++;
                break;

            case RTU2:
                switch(i){
                    case 1:
                        Com_MODBUS_Write(RTU2,WriteRegister,i,Tanque[i].Val);
                        i++;
                        break;
                    case 2:
                        Com_MODBUS_Write(RTU2,WriteRegister,i,Tanque[i].Val);
                        i++;
                        break;
                    case 3:
                        Com_MODBUS_Write(RTU2,WriteRegister,i,Tanque[i].Val);
                        i++;
                        break;
                    case 4:
                        Com_MODBUS_Write(RTU2,WriteRegister,i,Tanque[i].Val);
                        i++;
                        break;
                    case 5:
                        Com_MODBUS_Write(RTU2,WriteRegister,i,Tanque[i].Val);
                        i++;
                        break;
                    case 6:
                        Com_MODBUS_Write(RTU2,WriteRegister,i,Tanque[i].Val);
                        i++;
                        break;
                    case 7:                     
                        Com_MODBUS_Write(RTU2,WriteRegister,i,Tanque[i].Val);
                        i = 1;
                        cont++;
                        break;
                    default:
                        break;
                         
                }
                
                
                break;

            case RTU3: 
                switch(j){
                    case 0:
                        Com_MODBUS_Write(RTU3,WriteCoil,6,Bomba.bits.b2);
                        j++;
                        break;
                    case 1:
                        Com_MODBUS_Write(RTU2,WriteCoil,6,Bomba.bits.b3);
                        j=0;
                        cont++;
                        break;
                }
                
                break;
            case RTU4:
                switch(k){
                    case 0:
                        Com_MODBUS_Write(RTU3,WriteCoil,6,Bomba.bits.b4);
                        k++;
                        break;
                    case 1:
                        Com_MODBUS_Write(RTU2,WriteCoil,6,Bomba.bits.b5);
                        k++;
                        break;
                    case 2:
                        Com_MODBUS_Write(RTU2,WriteCoil,6,Bomba.bits.b6);
                        k=0;
                        cont++;
                        break;
                }
                
                break;
                        
            case RTU5:
                Com_MODBUS_Write(RTU5,WriteCoil,6,Bomba.bits.b7);
                cont++;
                break;

            default:
                break;
             
        }
        
        break;
 
        case  leer:
            if (cont > 9){ss--; cont = 1;} // voy ahora a escribir
                switch(cont){
            case RTU1:
                switch(k){
                    case 0: 
                       Com_MODBUS_Read(RTU1,ReadInputRegisters,0,1);
                       k++;
                       break;
                    case 1: 
                        Com_MODBUS_Read(RTU1,ReadInputRegisters,1,1);
                        k=0;
                        cont++;
                        break;
                }
                
                break;

            case RTU2:
                Com_MODBUS_Read(RTU2,ReadInputRegisters,0,1);
                cont++;
                break;

            case RTU3: 
                Com_MODBUS_Read(RTU3,ReadInputRegisters,1,1);
                cont++;
                break;

            case RTU4:
                switch(j){
                    case 0:
                        Com_MODBUS_Read(RTU4,ReadInputRegisters,1,1);
                        j++;
                        break;
                    case 1:
                        Com_MODBUS_Read(RTU4,ReadInputRegisters,0,1);
                        j=0;
                        cont++;
                        break;
                         }
                
                break;

            case RTU5:
                Com_MODBUS_Read(RTU5,ReadInputRegisters,0,1);
                cont++;
                break;

            case RTU6:
                Com_MODBUS_Read(RTU6,ReadInputRegisters,0,1);
                cont++;
                break;

            case RTU7:
                Com_MODBUS_Read(RTU7,ReadInputRegisters,0,1);
                cont++;
                break;

            case RTU8:
                Com_MODBUS_Read(RTU8,ReadInputRegisters,0,1);
                cont++;
                break;

            case RTU9:
                Com_MODBUS_Read(RTU9,ReadInputRegisters,0,1);
                cont++;
                break;

            default:
                break;
                
       
        }
        break;
        
        default:
            break;
           
    }
    
    

    // ticker function call;
    // ticker is 1 -> Callback function gets called everytime this ISR executes
    if(TMR2_InterruptHandler) 
    { 
           TMR2_InterruptHandler(); 
    }

    //***User Area End

    tmr2_obj.count++;
    tmr2_obj.timerElapsed = true;
    IFS0bits.T2IF = false;
}

void TMR2_Period16BitSet( uint16_t value )
{
    /* Update the counter values */
    PR2 = value;
    /* Reset the status information */
    tmr2_obj.timerElapsed = false;
}

uint16_t TMR2_Period16BitGet( void )
{
    return( PR2 );
}

void TMR2_Counter16BitSet ( uint16_t value )
{
    /* Update the counter values */
    TMR2 = value;
    /* Reset the status information */
    tmr2_obj.timerElapsed = false;
}

uint16_t TMR2_Counter16BitGet( void )
{
    return( TMR2 );
}


void __attribute__ ((weak)) TMR2_CallBack(void)
{
    // Add your custom callback code here
}

void  TMR2_SetInterruptHandler(void (* InterruptHandler)(void))
{ 
    IEC0bits.T2IE = false;
    TMR2_InterruptHandler = InterruptHandler; 
    IEC0bits.T2IE = true;
}

void TMR2_Start( void )
{
    /* Reset the status information */
    tmr2_obj.timerElapsed = false;

    /*Enable the interrupt*/
    IEC0bits.T2IE = true;

    /* Start the Timer */
    T2CONbits.TON = 1;
}

void TMR2_Stop( void )
{
    /* Stop the Timer */
    T2CONbits.TON = false;

    /*Disable the interrupt*/
    IEC0bits.T2IE = false;
}

bool TMR2_GetElapsedThenClear(void)
{
    bool status;
    
    status = tmr2_obj.timerElapsed;

    if(status == true)
    {
        tmr2_obj.timerElapsed = false;
    }
    return status;
}

int TMR2_SoftwareCounterGet(void)
{
    return tmr2_obj.count;
}

void TMR2_SoftwareCounterClear(void)
{
    tmr2_obj.count = 0; 
}

/**
 End of File
*/
