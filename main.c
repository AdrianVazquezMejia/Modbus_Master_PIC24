/**
  Generated main.c file from MPLAB Code Configurator

  @Company
    Microchip Technology Inc.

  @File Name
    main.c

  @Summary
    This is the generated main.c using PIC24 / dsPIC33 / PIC32MM MCUs.

  @Description
    This source file provides main entry point for system initialization and application code development.
    Generation Information :
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.125
        Device            :  PIC24FJ64GA002
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.36B
        MPLAB 	          :  MPLAB X v5.20
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
#include "mcc_generated_files/system.h"
#include "mcc_generated_files/uart2.h"
#include "mcc_generated_files/CRC.h"
#include "mcc_generated_files/pin_manager.h"
/*

                         Main application
 */
extern INT_VAL InputRegister[10];
extern INT_VAL HoldingRegister[10];
extern uint8_t buffTx[100], contTx, *pint;
extern INT_VAL CoilRegister;
extern INT_VAL DiscreteInputRegister,Crc;

unsigned long t;
int main(void)
{
    // initialize the device
    SYSTEM_Initialize();

    while (1)
    {
        // Requesting a test information to a slave connected in UART2
        buffTx[0]=01;
        buffTx[1]=03;
        buffTx[2]=00;
        buffTx[3]=00;
        buffTx[4]=00;
        buffTx[5]=02;
        Crc.Val=CRC16(buffTx,6);
		buffTx[6]=Crc.byte.LB;
		buffTx[7]=Crc.byte.HB;
        
        contTx=8;
		pint=buffTx;                
		U2TXREG = *pint;
		
        
        for(t=0;t<1000000;t++);  
        LED1=!LED1;
    }

    return 1;
}
/**
 End of File
*/

