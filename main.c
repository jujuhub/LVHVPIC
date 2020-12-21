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
#define DISABLED 0
#define ENABLED 1


/*
                         Main application
 */
int main(void)
{
    /* Initialize the device (PIC) */
    SYSTEM_Initialize();
    INTERRUPT_GlobalEnable();
    
    int i = 0;
    for (i = 0; i < 255; ++i);
    
    /* Make sure LV and HV are OFF */
    LV_ON_OFF_SetDigitalOutput();
    LV_ON_OFF_SetLow();
    uint16_t LV_EN; 
    LV_EN = LV_ON_OFF_GetValue(); // what values does this give back? 
    while (LV_EN != DISABLED)
    {
        LV_ON_OFF_SetLow();
        LV_EN = LV_ON_OFF_GetValue();
    }
    
    HV_ON_OFF_SetDigitalOutput();
    HV_ON_OFF_SetLow();
    uint16_t HV_EN;
    HV_EN = HV_ON_OFF_GetValue();
    while (HV_EN != DISABLED)
    {
        HV_ON_OFF_SetLow();
        HV_EN = HV_ON_OFF_GetValue();
    }

/////////////////////////////////////////////////////////////////////////////    
    /* Turn ON LV lines */
    // needed for I2C communication (as of Oct 1, 2020)
//    LV_ON_OFF_SetDigitalOutput();
//    LV_ON_OFF_SetHigh();
    
    // some delay; necessary?
//    while (i < 255)
//    {
//        if (i == 254) break;
//        ++i;
//    }

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


    
    /* Turn OFF LV lines */
//    LV_ON_OFF_SetDigitalOutput();
//    LV_ON_OFF_SetLow();
/////////////////////////////////////////////////////////////////////////////

    while (1)
    {
        // Add your application code
        
        /* CANbus communication with Raspberry Pi + PiCAN HAT */
        // declare CAN msg variables
        uCAN_MSG rxCANmsg, *pRxCANmsg, txCANmsg, *pTxCANmsg;
        pRxCANmsg = &rxCANmsg;
        pTxCANmsg = &txCANmsg;
        uint32_t msgID = 0x00;
        CAN_TX_PRIOIRTY msg_prio = CAN_PRIORITY_MEDIUM;
        
        CAN1_ReceiveEnable();
        
        CAN1_receive(pRxCANmsg);            // breakpoint here
        while (C1RXFUL1 == 0x0000)          // #TODO: fix this
        {
            if (C1RXFUL1bits.RXFUL1 == 0)   // if specific register is empty
                break;
        }
        C1RXFUL1bits.RXFUL1 = 0; // need to clear out the buffers?
        
        msgID = rxCANmsg.frame.id;
        
        // RH&T variables
        uint8_t sensorData[4] = {0}, *pData;
        uint8_t humH, humL, tempH, tempL;
        pData = sensorData;
        uint8_t _status;
        
        // LV variables
        
        // HV variables
        uint8_t setHVBuffer[3] = {0}, *pHV;
        pHV = setHVBuffer;
        

        switch(msgID)
        {
            /********* HUMIDITY & TEMPERATURE *********/
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
                CAN1_transmit(msg_prio, pTxCANmsg);
                while (C1TR01CONbits.TXREQ0 == 1);
                msgID = 0x00;
                CAN1_ReceiveEnable(); // test this outside of switch-statement
                break;
            
            /********* LOW VOLTAGE *********/
            case 0x010: // Turn OFF LV request
                LV_ON_OFF_SetDigitalOutput();
                LV_ON_OFF_SetLow();
                LV_EN = LV_ON_OFF_GetValue();
                while (LV_EN != DISABLED)
                {
                    LV_ON_OFF_SetLow();
                    LV_EN = LV_ON_OFF_GetValue();
                }
                txCANmsg.frame.id = 0x011;
                txCANmsg.frame.idType = CAN_FRAME_STD;
                txCANmsg.frame.msgtype = CAN_MSG_DATA;
                txCANmsg.frame.dlc = 0b1000;
                txCANmsg.frame.data0 = (LV_EN >> 8); // high part
                txCANmsg.frame.data1 = (uint8_t)(LV_EN); // low part
                txCANmsg.frame.data2 = 0x00;
                txCANmsg.frame.data3 = 0x00;
                txCANmsg.frame.data4 = 0x00;
                txCANmsg.frame.data5 = 0x00;
                txCANmsg.frame.data6 = 0x00;
                txCANmsg.frame.data7 = 0x00;
                CAN1_TransmitEnable();
                CAN1_transmit(msg_prio, pTxCANmsg);
                while (C1TR01CONbits.TXREQ0 == 1);
                msgID = 0x00;
                break;
            
            case 0x020: // Turn ON LV request
                LV_ON_OFF_SetDigitalOutput();
                LV_ON_OFF_SetHigh();
                LV_EN = LV_ON_OFF_GetValue();
                while (LV_EN != ENABLED)
                {
                    LV_ON_OFF_SetHigh();
                    LV_EN = LV_ON_OFF_GetValue();
                }
                txCANmsg.frame.id = 0x021;
                txCANmsg.frame.idType = CAN_FRAME_STD;
                txCANmsg.frame.msgtype = CAN_MSG_DATA;
                txCANmsg.frame.dlc = 0b1000;
                txCANmsg.frame.data0 = (LV_EN >> 8); // high part
                txCANmsg.frame.data1 = (uint8_t)(LV_EN); // low part
                txCANmsg.frame.data2 = 0x00;
                txCANmsg.frame.data3 = 0x00;
                txCANmsg.frame.data4 = 0x00;
                txCANmsg.frame.data5 = 0x00;
                txCANmsg.frame.data6 = 0x00;
                txCANmsg.frame.data7 = 0x00;
                CAN1_TransmitEnable();
                CAN1_transmit(msg_prio, pTxCANmsg);
                while (C1TR01CONbits.TXREQ0 == 1);
                msgID = 0x00;
                break;
            
            /********* HIGH VOLTAGE *********/
            case 0x030: // Turn off HV request
                // set HV_Ctrl HV value to 0
                // configure pins
                HV_ON_OFF_SetDigitalOutput();
                HV_ON_OFF_SetLow();
                HV_EN = HV_ON_OFF_GetValue();
                while (HV_EN != DISABLED)
                {
                    HV_ON_OFF_SetLow();
                    HV_EN = HV_ON_OFF_GetValue();
                }
                msgID = 0x00;
                break;
            
            case 0x040: // Turn on HV request
                // set HV_Ctrl HV value
                // configure pins
                HV_ON_OFF_SetDigitalOutput();
                HV_ON_OFF_SetHigh();
                HV_EN = HV_ON_OFF_GetValue();
                while (HV_EN != ENABLED)
                {
                    HV_ON_OFF_SetHigh();
                    HV_EN = HV_ON_OFF_GetValue();
                }
                msgID = 0x00;
                break;
            
            case 0x050: // Set the HV % (write to DAC)
                // set HV_Ctrl HV value
                setHVBuffer[0] = 0x30; // command
                setHVBuffer[1] = rxCANmsg.frame.data0; // MS data
                setHVBuffer[2] = rxCANmsg.frame.data1; // LS data; last 4 bits are don't care
                I2C_Write(LTC2631_ADDR, 3, pHV);
                // extract data from CAN msg and send to DAC
                msgID = 0x00;
                break;
        
        } // end switch-case for CAN
        
        int j;
        for (j = 0; j < 10; j++);
        msgID = 0x00;
    }
    return 1; 
}

/**
 End of File
*/