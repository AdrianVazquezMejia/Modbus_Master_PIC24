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
#define Manual 0
#define Auto 1

/*
 * 
                         Main application
 */
extern INT_VAL InputRegister[10];
extern INT_VAL HoldingRegister[10];
extern INT_VAL Tanque[7];
extern uint8_t buffTx[100], contTx, *pint;
extern INT_VAL DInputRegister[10][6];
extern INT_VAL Bomba;
bool modo;
//extern INT_VAL CoilRegister;
//extern INT_VAL DiscreteInputRegister,Crc;
uint16_t s=0;
unsigned long t;
int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    HoldingRegister[0].Val=5;
    HoldingRegister[1].Val=10;
    extern INT_VAL Tanque[7];
    while (1)
    {

       // logica mimico
        
        if (DInputRegister[2][0].bits.b0 == 0){
            modo = Manual;
            Bomba.Val=DInputRegister[2][0].Val;
        }
        else modo = Auto;
        
        if (modo == Manual)
            Bomba.Val= DInputRegister[2][0].byte.LB;
        
        Tanque[0].Val=Bomba.Val;
        Tanque[1].Val=DInputRegister[1][0].byte.LB;
        Tanque[2].Val=DInputRegister[4][0].byte.LB;
        Tanque[3].Val=DInputRegister[6][0].byte.LB;
        Tanque[4].Val=DInputRegister[7][0].byte.LB;
        Tanque[5].Val=DInputRegister[8][0].byte.LB;
        Tanque[6].Val=DInputRegister[5][0].byte.LB;
        Tanque[7].Val=DInputRegister[9][0].byte.LB;
        
        //Logica en Modo automatico
        if (modo == Auto){
            if(Tanque[1].Val < 512)
                Bomba.bits.b1 = 1;
            if (Tanque[1].Val > 1000)
                Bomba.bits.b1 = 1;
            
            if((Tanque[6].Val < 512)&(Tanque[1].Val >512))
                Bomba.bits.b3 = 1;
            if (Tanque[6].Val > 0x1000)
                Bomba.bits.b1 = 0;
        
            if((Tanque[7].Val < 512)&(Tanque[6].Val >512))
                Bomba.bits.b7 = 1;
            if (Tanque[6].Val > 0x1000)
                Bomba.bits.b7 = 0;
            
            if((Tanque[2].Val < 512)&(Tanque[1].Val >512))
                Bomba.bits.b2 = 1;
            if (Tanque[2].Val > 0x1000)
                Bomba.bits.b2 = 0;
        
            if((Tanque[3].Val < 512)&(Tanque[2].Val >512))
                Bomba.bits.b4 = 1;
            if (Tanque[3].Val > 0x1000)
                Bomba.bits.b4 = 0;
        
            if((Tanque[4].Val < 512)&(Tanque[2].Val >512))
                Bomba.bits.b5 = 1;
            if (Tanque[4].Val > 0x1000)
                Bomba.bits.b5 = 0;
            
            if((Tanque[5].Val < 512)&(Tanque[2].Val >512))
                Bomba.bits.b6 = 1;
            if (Tanque[5].Val > 0x1000)
                Bomba.bits.b6 = 0;
        }
        
    }
        
    
    return 1;
}
/**
 End of File
*/

