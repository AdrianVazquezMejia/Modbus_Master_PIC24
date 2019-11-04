
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

extern INT_VAL InputRegister[10];
extern INT_VAL HoldingRegister[10];
extern uint8_t buffTx[100], contTx, *pint;
extern Com_MODBUD_Write();
extern Com_MODBUD_Write1();


/**
 Section: File specific functions
*/

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



/**
  Section: Driver Interface
*/

void TMR2_Initialize (void)
{
    //TMR2 0; 
    TMR2 = 0x00;
    //Period = 0 s; Frequency = 16000000 Hz; PR2 0; 
    PR2 = 0x00;
    //TCKPS 1:1; T32 16 Bit; TON enabled; TSIDL disabled; TCS FOSC/2; TGATE disabled; 
    T2CON = 0x8000;


    IFS0bits.T2IF = false;
    IEC0bits.T2IE = true;
	

}


int cont = 1;

void __attribute__ ( ( interrupt, no_auto_psv ) ) _T2Interrupt ( )
{
    if(cont > 9){cont=1;}
    
    switch(cont){
        case RTU1:
            Com_MODBUS_Write(RTU1, WriteCoil, OutValve, ON);
            cont++;
            break;
            
        case RTU2:
            Com_MODBUS_Write(RTU2,WriteCoil,OutValve,ON);
            cont++;
            break;
            
        case RTU3: 
            Com_MODBUS_Write(RTU3,WriteCoil,OutValve,ON);
            cont++;
            break;
            
        case RTU4:
            Com_MODBUS_Write1(RTU4,WriteCoil,OutValve,ON);
            cont++;
            break;
            
        case RTU5:
            Com_MODBUS_Write1(RTU5,WriteCoil,OutValve,ON);
            cont++;
            break;
            
        case RTU6:
            Com_MODBUS_Write1(RTU6,WriteCoil,OutValve,ON);
            cont++;
            break;
            
        case RTU7:
            Com_MODBUS_Write1(RTU7,WriteCoil,OutValve,ON);
            cont++;
            break;
            
        case RTU8:
            Com_MODBUS_Write1(RTU8,WriteCoil,OutValve,ON);
            cont++;
            break;
            
        case RTU9:
            Com_MODBUS_Write1(RTU9,WriteCoil,OutValve,ON);
            cont++;
            break;
            
        default:
            break;
            
    }
    

    IFS0bits.T2IF = false; 
}


