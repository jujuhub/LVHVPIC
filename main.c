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

#include "hih6030.h"

/*
                         Local functions
 */
#define LTC2631_ADDR 0x73
#define LTC2631_I2C_TIMEOUT 255
#define LTC2631_MAX_RETRY 255

void set_HV(uint8_t *pHV)
{
    /**
     *  @Summary
     *      Send nominal HV value to LTC2631 (DAC), which in turn sets the HV 
     *      for C40N (DC-HVDC converter)
     *  @Param
     *      pHV: pointer to array containing HV set value in 3 bytes
     *  @Return
     *      none
     */

    uint16_t retryTimeOut = 0, slaveTimeOut = 0;
    uint8_t setHVBuffer[3];
    setHVBuffer[0] = *pHV; // command
    setHVBuffer[1] = *(pHV + 1); // MS data
    setHVBuffer[2] = *(pHV + 2); // LS data; last 4 bits are don't care
    
    I2C1_MESSAGE_STATUS i2c_stat;
    i2c_stat = I2C1_MESSAGE_PENDING;
    
    /* Initiate communication with DAC & transmit data (3 bytes) */
    while (i2c_stat != I2C1_MESSAGE_COMPLETE)
    {
        I2C1_MasterWrite(setHVBuffer, 3, LTC2631_ADDR, &i2c_stat);
        
        while (i2c_stat == I2C1_MESSAGE_PENDING)
        {
            // check for timeout
            if (slaveTimeOut == LTC2631_I2C_TIMEOUT)
                break;
            else
                slaveTimeOut++;
        }
        
        // check for max retry
        if (retryTimeOut == LTC2631_MAX_RETRY)
            break;
        else
            retryTimeOut++;
        
        // if transmission failed (or ACK was not received from slave device)
        if (i2c_stat == I2C1_MESSAGE_FAIL)
            // do something else?
            break;
    }
    
    /* Power down DAC */

}

/*
                         Main application
 */
int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    INTERRUPT_GlobalEnable();
    
    /* Turn on LV (set pin as output high) */
    LV_ON_OFF_SetDigitalOutput();
    LV_ON_OFF_SetHigh();
    
    /* Turn on HV (set pin as output high) */
    // set HV value via DAC first? if float, need to convert to hex
    uint8_t setHVBuffer[3] = {0}, *pHV;
    setHVBuffer[0] = 0x03; // command
    setHVBuffer[1] = 0xFF; // MS data
    setHVBuffer[2] = 0xF0; // LS data; last 4 bits are don't care
    pHV = setHVBuffer;
    
    set_HV(pHV);
    
    // configure pins
    HV_ON_OFF_SetDigitalOutput();
    HV_ON_OFF_SetHigh();
    
    /* Read from photodiode (ADC) */
    
    
    /* Communication with trigger board (SPI) */
    
    
    /*
     *      HIH6030 Relative Humidity & Temperature Sensor
     */
    
    /* Enter Command Mode within 3 ms */
//    I2C1_MESSAGE_STATUS i2cStatus;
//    HIH6030_Write(0xA0, 0x0000, &i2cStatus);
//    while (i2cStatus != I2C1_MESSAGE_COMPLETE);
    /* Read/Write from EEPROM location */
//    uint8_t cmdmodeData[3], *pD;
//    pD = cmdmodeData;
//    HIH6030_Read(0x18, pD); // Alarm_High_On


    while (1)
    {
        // Add your application code
        
        /* CANbus communication with Raspberry Pi + PiCAN HAT */

        uCAN_MSG rxCANmsg, *pRxCANmsg, txCANmsg, *pTxCANmsg;
        pRxCANmsg = &rxCANmsg;
        pTxCANmsg = &txCANmsg;
        uint32_t msgID;
        
        CAN1_ReceiveEnable();
        
        CAN1_receive(pRxCANmsg);
        while (C1RXFUL1 == 0x0000)          // #TODO: fix this
        {
            if (C1RXFUL1bits.RXFUL1 == 0) // if specific register is empty
                break;
        }
        C1RXFUL1bits.RXFUL1 = 0;
        
        msgID = rxCANmsg.frame.id;
        
        // RH&T variables
        uint16_t H_dat, T_dat, *pHum, *pTemp;
        uint8_t humH, humL, tempH, tempL;
        pHum = &H_dat;
        pTemp = &T_dat; 
        uint8_t _status;
        
        // LV variables
        
        switch(msgID)
        {
            case 0x123: // RH&T request
                _status = fetch_RHT(pHum, pTemp);
                humH = (H_dat >> 8);
                humL = (H_dat & 0xFF);
                tempH = (T_dat >> 8);
                tempL = (T_dat & 0xFF);
                
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
                CAN1_transmit(msg_prio, pTxCANmsg);
                while (C1TR01CONbits.TXREQ0 == 1);
                
                msgID = 0x00;
                break;
            
            case 0x001: // Turn off LV request
                LV_ON_OFF_SetDigitalOutput();
                LV_ON_OFF_SetLow();
                
                msgID = 0x00;
                break;
            
            case 0x002: // Turn on LV request
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
        
        }

        /* Make RH&T measurement, fetch results from HIH6030-021 sensor */
//        _status = fetch_RHT(pHum, pTemp);
//
//        switch(_status) // should use this to send CAN messages to Raspberry Pi
//        {
//            case 0:
//                printf("Normal.\n");
//                // send data to RP
//                break;
//
//            case 1:
//                printf("Stale Data.\n");
//                // do not send data; try to make a new measurement
//                break;
//
//            case 2:
//                // this shouldn't happen in Normal Operation mode
//                printf("Command Mode.\n");
//                // send warning to RP
//                break;
//
//            default: 
//                printf("Diagnostic, or Invalid Data.\n");
//                // send warning to RP
//                break;
//        } // end RH&T measurement

    } // end main while loop

    return 1; 
}

/**
 End of File
*/