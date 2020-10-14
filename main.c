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
        Product Revision  :  PIC24 / dsPIC33 / PIC32MM MCUs - 1.155.0-a
        Device            :  dsPIC33EV256GM102
    The generated drivers are tested against the following:
        Compiler          :  XC16 v1.40
        MPLAB 	          :  MPLAB X v5.25
*/

/*
    (c) 2019 Microchip Technology Inc. and its subsidiaries. You may use this
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
#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/interrupt_manager.h"

#include "i2c_devices.h"

/*
                         Local functions
 */



/*
                         Main application
 */
int main(void)
{
    /* Initialize the device (PIC) */
    SYSTEM_Initialize();
    INTERRUPT_GlobalEnable();
    
    /* Turn ON LV lines */
    // needed for I2C communication (as of Oct 1, 2020)
    LV_ON_OFF_SetDigitalOutput();
    LV_ON_OFF_SetHigh();
    
    // some delay; necessary?
    int i = 0;
    while (i < 1000)
    {
        if (i == 255) break;
        ++i;
    }

    /* Turn ON HV (set pin as output high) */
    // set HV value via DAC first? if float, need to convert to hex
    
    /*
     * @Commands
     *      0b0000: Write to input register
     *      0b0001: Update (power up) DAC register
     *      0b0011: Write to and update (power up) DAC register
     *      0b0100: Power down
     *      0b0110: Select internal reference
     *      0b0111: Select external reference
     */
    
//    uint8_t setHVBuffer[3] = {0}, *pHV;
//    setHVBuffer[0] = 0x30; // command
//    setHVBuffer[1] = 0xFF; // MS data
//    setHVBuffer[2] = 0xFF; // LS data; last 4 bits are don't care
//    pHV = setHVBuffer;
//    
////    I2C_Write(LTC2631_ADDR, 3, pHV);
    
    // configure HV pins
//    HV_ON_OFF_SetDigitalOutput();
//    HV_ON_OFF_SetHigh();
    
//    setHVBuffer[1] = 0x00;
//    setHVBuffer[2] = 0x00;
//    I2C_Write(LTC2631_ADDR, 3, pHV);
    
//    HV_ON_OFF_SetDigitalOutput();
//    HV_ON_OFF_SetLow();
    
    
    /* Make RH&T measurement & fetch results from HIH6030-021 sensor */
//    while(1)
//    {
//        uint16_t H_dat, T_dat, *pHum, *pTemp;
//        pHum = &H_dat;
//        pTemp = &T_dat; 
//        uint8_t _status;
//
//        _status = fetch_RHT(pHum, pTemp);
//    }
    
    /* Communication with trigger board (I2C) */
//    while(1)
//    {
//        uint8_t setTrigBuffer[2], *pTrig;
//        setTrigBuffer[0] = 0x0F;
//        setTrigBuffer[1] = 0xFF;
//        pTrig = setTrigBuffer;
//
//        I2C_Write(MCP4725_DAC0_ADDR, 2, pTrig);
//    }
//    
//    
//    uint8_t setHVBuffer[3], *pHV;
//    setHVBuffer[0] = 0x30; // command
//    setHVBuffer[1] = 0xFF; // MS data
//    setHVBuffer[2] = 0xFF; // LS data; last 4 bits are don't care
//    pHV = setHVBuffer;
//    
//    I2C_Write(LTC2631_ADDR, 3, pHV);


    /* Read from photodiode (ADC) */
    
    
    /* Scan for I2C device addresses */
//    I2C1_MESSAGE_STATUS status;
//    uint8_t ad, ndev = 0;
//    uint8_t dummy[2] = {0}, *pDummy;
//    pDummy = dummy;
//    
//    for (ad = 0x50; ad <= 0x73; ++ad)
//    {
//        status = I2C_Write(ad, 2, pDummy);
//        
//        if (status == I2C1_MESSAGE_COMPLETE)
//            ++ndev;
//    }

    
    /* Turn OFF LV lines */
//    LV_ON_OFF_SetDigitalOutput();
//    LV_ON_OFF_SetLow();
    
    while (1)
    {
        // Add your application code
        
        /* CANbus communication with Raspberry Pi + PiCAN HAT */
        uCAN_MSG rxCANmsg, *pRxCANmsg, txCANmsg, *pTxCANmsg;
        pRxCANmsg = &rxCANmsg;
        pTxCANmsg = &txCANmsg;
        uint32_t msgID;
        
        CAN1_ReceiveEnable();
        
        CAN1_receive(pRxCANmsg);            // breakpoint here
        while (C1RXFUL1 == 0x0000)          // #TODO: fix this
        {
            if (C1RXFUL1bits.RXFUL1 == 0)   // if specific register is empty
                break;
        }
        C1RXFUL1bits.RXFUL1 = 0;
        
        msgID = rxCANmsg.frame.id;
        
        // RH&T variables
        uint8_t sensorData[4], *pData;
        uint8_t humH, humL, tempH, tempL;
        pData = sensorData;
        uint8_t _status;
        
        // LV variables

        switch(msgID)
        {
            case 0x123: // RH&T request
                _status = fetch_RHT(pData);
                humH = sensorData[0];
                humL = sensorData[1];
                tempH = sensorData[2];
                tempL = sensorData[3];
                
                txCANmsg.frame.id = 0x123;
                txCANmsg.frame.idType = CAN_FRAME_STD;
                txCANmsg.frame.msgtype = CAN_MSG_DATA;
                txCANmsg.frame.dlc = 0b1000;
                txCANmsg.frame.data0 = _status;
                txCANmsg.frame.data1 = humH;
                txCANmsg.frame.data2 = humL;
                txCANmsg.frame.data3 = tempH;
                txCANmsg.frame.data4 = tempL;
                txCANmsg.frame.data5 = 0x00;
                txCANmsg.frame.data6 = 0x00;
                txCANmsg.frame.data7 = 0x00;

                CAN1_TransmitEnable();

                CAN_TX_PRIOIRTY msg_prio = CAN_PRIORITY_MEDIUM;
                int c;
                for (c = 0; c < 100; c++)
                {
                    CAN1_transmit(msg_prio, pTxCANmsg);
                    while (C1TR01CONbits.TXREQ0 == 1);
                }
                
                msgID = 0x00;
                CAN1_ReceiveEnable();
                break;
            
            case 0x001: // Turn OFF LV request
                LV_ON_OFF_SetDigitalOutput();
                LV_ON_OFF_SetLow();
                
                msgID = 0x00;
                break;
            
            case 0x002: // Turn ON LV request
                LV_ON_OFF_SetDigitalOutput();
                LV_ON_OFF_SetHigh();
                
                msgID = 0x00;
                break;
            
            case 0x003: // Turn off HV request
                // set HV_Ctrl HV value to 0
                
                // configure pins
                HV_ON_OFF_SetDigitalOutput();
                HV_ON_OFF_SetLow();
                
                msgID = 0x00;
                break;
            
            case 0x004: // Turn on HV request
                // set HV_Ctrl HV value
                
                // configure pins
                HV_ON_OFF_SetDigitalOutput();
                HV_ON_OFF_SetHigh();
                
                msgID = 0x00;
                break;
            
            case 0x005: // Set HV but do not turn on? Do we want this?
                // set HV_Ctrl HV value
                
                msgID = 0x00;
                break;
        
        } // end switch-case for CAN
    }
    return 1; 
}

/**
 End of File
*/