
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
int i = 1;
int cont = 1;
BOMBAS bombas;
TANQUES tanques;
extern INT_VAL InputRegister[10];
extern INT_VAL HoldingRegister[10];
extern uint8_t buffTx[100], contTx, *pint;

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
    //Period = 0.05 s; Frequency = 16000000 Hz; PR2 12499; 
    PR2 = 0x30D3;
    //TCKPS 1:64; T32 16 Bit; TON enabled; TSIDL disabled; TCS FOSC/2; TGATE enabled; 
    T2CON = 0x8020;
    

    IFS0bits.T2IF = false;
    IEC0bits.T2IE = true;

     /* Start the Timer */
    T2CONbits.TON = 1;
    
    
	
    

}




void __attribute__ ( ( interrupt, no_auto_psv ) ) _T2Interrupt (  )
{
    /* Check if the Timer Interrupt/Status is set */

    //***User Area Begin
    switch(ss){
        case  escribir:
                switch(cont){
            case RTU1:
                Com_MODBUS_Write(RTU1,WriteCoil,0,ON);
//                tanques.tanque = 0b0000000000000001
                cont++;
                break;

            case RTU2:
//                switch(i){
//                    case 1:
//                        Com_MODBUS_Write(RTU2,WriteRegister,i,mimico[i]);
//                        i++;
//                        break;
//                    case 2:
//                        Com_MODBUS_Write(RTU2,WriteRegister,i,mimico[i]);
//                        i++;
//                        break;
//                    case 3:
//                        Com_MODBUS_Write(RTU2,WriteRegister,i,mimico[i]);
//                        i++;
//                        break;
//                    case 4:
//                        Com_MODBUS_Write(RTU2,WriteRegister,i,mimico[i]);
//                        i++;
//                        break;
//                    case 5:
//                        Com_MODBUS_Write(RTU2,WriteRegister,i,mimico[i]);
//                        i++;
//                        break;
//                    case 6:
//                        Com_MODBUS_Write(RTU2,WriteRegister,i,mimico[i]);
//                        i++;
//                        break;
//                    case 7:                     
//                        Com_MODBUS_Write(RTU2,WriteRegister,i,mimico[i]);
//                        i = 1;
//                        break;
//                    default:
//                        break;
//                         
//                };
                
                cont++;
                break;

            case RTU3: 
                Com_MODBUS_Write(RTU3,WriteCoil,0,ON);
//                tanques.tanque = 0b0000000000000110
                cont++;
                break;

            case RTU4:
                Com_MODBUS_Write(RTU4,WriteCoil,0,ON);
//                bombas.bomba = 0b0000000000110100
//                Com_MODBUS_Write(RTU4,WriteCoil,tanques.tanque = 0b0000000000000010,ON);
                cont++;
                break;

            case RTU5:
                Com_MODBUS_Write(RTU5,WriteCoil,0,ON);
//                bombas.bomba = 0b0000000001000000
                ss++;
                cont = 1;
                break;

            default:
                break;
             
        };
        
        break;
 
        case  leer:
                switch(cont){
            case RTU1:
                Com_MODBUS_Read(RTU1,ReadInputRegisters,tanques.tanque = 0b0000000000000001,1);
                cont++;
                break;

            case RTU2:
                Com_MODBUS_Read(RTU2,ReadInputRegisters,0,1);
                cont++;
                break;

            case RTU3: 
                Com_MODBUS_Read(RTU3,ReadInputRegisters,tanques.tanque = 0b0000000000000110,1);
                cont++;
                break;

            case RTU4:
                Com_MODBUS_Read(RTU4,ReadInputRegisters,bombas.bomba = 0b0000000000110100,1);
                Com_MODBUS_Read(RTU4,ReadInputRegisters,tanques.tanque = 0b0000000000000010,1);
                cont++;
                break;

            case RTU5:
                Com_MODBUS_Read(RTU5,ReadInputRegisters,bombas.bomba = 0b0000000001000000,1);
                cont++;
                break;

            case RTU6:
                Com_MODBUS_Read(RTU6,ReadInputRegisters,tanques.tanque = 0b0000000000000100,1);
                cont++;
                break;

            case RTU7:
                Com_MODBUS_Read(RTU7,ReadInputRegisters,bombas.bomba = 0b0000000000001000,1);
                cont++;
                break;

            case RTU8:
                Com_MODBUS_Read(RTU8,ReadInputRegisters,bombas.bomba = 0b0000000000010000,1);
                cont++;
                break;

            case RTU9:
                Com_MODBUS_Read(RTU9,ReadInputRegisters,bombas.bomba = 0b0000000001000000,1);
                cont = 1;
                ss--;
                break;

            default:
                break;
                
       
        }
        break;
        
        default:
            break;
           
    }


    //***User Area End

    IFS0bits.T2IF = false;
}





/**
 End of File
*/
