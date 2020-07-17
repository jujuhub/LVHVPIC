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

#define HIH6030_RETRY_MAX 255 // define retry count (100)
#define HIH6030_I2C_TIMEOUT 255 // define slave timeout (50)
#define HIH6030_ADDRESS 0x27 // HIH6030 slave device address

/*
 *          Local functions
 */
void HIH6030_Write(uint8_t command, uint16_t dat, I2C1_MESSAGE_STATUS *pstatus);
void HIH6030_Read(uint8_t command, uint8_t *pData);
uint8_t fetch_RHT(float *pHum, float *pTemp);

/*
                         Main application
 */
int main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    INTERRUPT_GlobalEnable();
    
    uint8_t d = 0;
    for(d = 0; d < 255; ++d); // some delay
    
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
        
        /* Make RH&T measurement, fetch results from HIH6030-021 sensor */
        float hum = 0., temp = 0., *pHum, *pTemp;
        pHum = &hum;
        pTemp = &temp;
        uint8_t _status = fetch_RHT(pHum, pTemp);

        switch(_status) // should use this to send CAN messages to Raspberry Pi
        {
            case 0:
                printf("Normal.\n");
                // send data to RP
                break;

            case 1:
                printf("Stale Data.\n");
                // do not send data; try to make a new measurement
                break;

            case 2:
                // this shouldn't happen in Normal Operation mode
                printf("Command Mode.\n");
                // send warning to RP
                break;

            default: 
                printf("Diagnostic, or Invalid Data.\n");
                // send warning to RP
                break;
        } // end RH&T sensor
        
        /* CANbus testing */
        // reception
        uCAN_MSG rxCANmsg, *pRxCANmsg;
        pRxCANmsg = &rxCANmsg;
        uint32_t msgID;
        
        CAN1_ReceiveEnable();
        
        CAN1_receive(pRxCANmsg);
        while (C1RXFUL1 == 0x0000)
        {
            if (C1RXFUL1bits.RXFUL1 == 0) // if specific register is empty
                break;
        }
        C1RXFUL1bits.RXFUL1 = 0;
        
        msgID = rxCANmsg.frame.id;
        
        switch(msgID)
        {
            case 0x123: // RH&T request
                uint16_t H_dat, T_dat, *pHum, *pTemp;
                uint8_t humH, humL, tempH, tempL;
                pHum = &H_dat;
                pTemp = &T_dat;
                
                uint8_t _status = fetch_RHT(pHum, pTemp);
                humH = H_dat >> 8;
                humL = H_dat & 0xFF;
                tempH = T_dat >> 8;
                tempL = T_dat & 0xFF;
                
                uCAN_MSG txCANmsg, *pTxCANmsg;
                pTxCANmsg = &txCANmsg;
                
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
                
                break;

            case 0x100: // Power shut-off request
                // do something here
                break;
        }
    }
    return 1; 
}

uint8_t fetch_RHT(uint16_t *pHum, uint16_t *pTemp)
{
    /**
     *  @Summary
     *      Read from HIH6030 device and return data status, RH, T
     *  @Param
     *      pHum: pointer to variable where RH will be stored
     *  @Param
     *      pTemp: pointer to variable where T will be stored
     *  @Return
     *      status of message
     */

    /* Declare variables */
    uint8_t writeBuffer[1], sensorData[4] = {0}, _status, *pD;
    uint16_t H_dat, T_dat, retryTimeOut = 0, slaveTimeOut = 0;
    pD = sensorData;
    writeBuffer[0] = (HIH6030_ADDRESS << 1); // dummy data

    I2C1_MESSAGE_STATUS i2cStatus;
    i2cStatus = I2C1_MESSAGE_PENDING;

    /* Begin transmission */
//    I2C1CONLbits.SEN = 1;
//    while (I2C1CONLbits.SEN == 1); // wait for SEN bit to clear

    /* Send Measurement Request (MR) */
    while (i2cStatus != I2C1_MESSAGE_COMPLETE)
    {
        I2C1_MasterWrite(writeBuffer, 1, HIH6030_ADDRESS, &i2cStatus);
        
        while(i2cStatus == I2C1_MESSAGE_PENDING)
        {
            // check for timeout
            if (slaveTimeOut == HIH6030_I2C_TIMEOUT)
                break;
            else
                slaveTimeOut++;
        }
        
        // check for max retry
        if (retryTimeOut == HIH6030_RETRY_MAX)
            break;
        else
            retryTimeOut++;

        // if transmission failed (or ACK was not received from slave device?)
        if (i2cStatus == I2C1_MESSAGE_FAIL)
            // do something more?
            break;
    }

    /* Fetch the RH & T data (4 bytes) */
    if (i2cStatus == I2C1_MESSAGE_COMPLETE)
    {
        retryTimeOut = 0;
        slaveTimeOut = 0;
        
        while (i2cStatus != I2C1_MESSAGE_FAIL)
        {
            I2C1_MasterRead(pD, 4, HIH6030_ADDRESS, &i2cStatus);
            
//            uint8_t dt;
//            for (dt = 0; dt < 255; ++dt);
//            for (dt = 0; dt < 255; ++dt);
            
            while (i2cStatus == I2C1_MESSAGE_PENDING)
            {
                // add delay here
                
                // check for timeout
                if (slaveTimeOut == HIH6030_I2C_TIMEOUT)
                    break;
                else
                    slaveTimeOut++;
            }

            // check for max retry
            if (retryTimeOut == HIH6030_RETRY_MAX)
                break;
            else
                retryTimeOut++;

            // check if bytes successfully received
            if ((i2cStatus == I2C1_MESSAGE_COMPLETE) || (I2C1STATbits.P == 1))
                break;
        }
    }
    
    // another read after the first to see if data fetched is the same
    uint8_t readBuffer[4] = {0}, *pRD;
    pRD = readBuffer;
    
    if (i2cStatus == I2C1_MESSAGE_COMPLETE) // never calls on this bc MSG_FAIL
    {
        retryTimeOut = 0;
        slaveTimeOut = 0;
        
        while (i2cStatus != I2C1_MESSAGE_FAIL)
        {
            I2C1_MasterRead(pRD, 4, HIH6030_ADDRESS, &i2cStatus);

            while (i2cStatus == I2C1_MESSAGE_PENDING)
            {
                // add delay here
                
                // check for timeout
                if (slaveTimeOut == HIH6030_I2C_TIMEOUT)
                    break;
                else
                    slaveTimeOut++;
            }

            // check for max retry
            if (retryTimeOut == HIH6030_RETRY_MAX)
                break;
            else
                retryTimeOut++;

            // check if bytes successfully received
            if ((i2cStatus == I2C1_MESSAGE_COMPLETE) || (I2C1STATbits.P == 1))
                break;
        }
    }

    /* Get status of data (first 2 bits) */
    _status = (readBuffer[0] >> 6) & 0x03;

    /* Convert RH&T counts to float */
    H_dat = (sensorData[0] & 0x3F)*256 + sensorData[1]; // #TODO: FIX CONVERSIONS
    T_dat = (sensorData[2]*256 + sensorData[3]) >> 2;

    *pHum = H_dat; ///16382. * 100.; // %RH
    *pTemp = T_dat; ///16382.*165. - 40.; // degrees C

    return _status;

}

void HIH6030_Write(uint8_t command, uint16_t dat, I2C1_MESSAGE_STATUS *pstatus)
{
    /**
     *  @Summary
     *      Write to HIH6030 slave device in Command Mode
     *  @Param
     *      command: command byte
     *  @Param
     *      dat: 16-bit (2-byte) data
     *  @Param
     *      pstatus: pointer to I2C status variable
     */

    // begin transmission
    I2C1CONLbits.SEN = 1;
    
    uint16_t retryTimeOut = 0, slaveTimeOut = 0;
    
    uint8_t dat_H, dat_L; // need to split up into 8-bit parts
    dat_H = (dat >> 8);
    dat_L = (uint8_t)(dat);
    
    uint8_t writeBuffer[3]; // command byte and data to be written
    writeBuffer[0] = command;
    writeBuffer[1] = dat_H;
    writeBuffer[2] = dat_L;
    
    // a work around for slow devices
    while (*pstatus != I2C1_MESSAGE_FAIL)
    {
        // supposedly this only writes 1 byte? #TODO: CHECK THIS!
        I2C1_MasterWrite(writeBuffer, 3, HIH6030_ADDRESS, pstatus);
        
        // wait for msg to be sent or status changed
        while (*pstatus == I2C1_MESSAGE_PENDING)
        {
            // add some delay here
            uint8_t dt; // need to use timers for delay?
            for (dt = 0; dt < 20; ++dt);
            
            // check for timeout
            if (slaveTimeOut == HIH6030_I2C_TIMEOUT)
                break;
            else
                slaveTimeOut++;
        }
        if ((slaveTimeOut == HIH6030_I2C_TIMEOUT) || 
                (*pstatus == I2C1_MESSAGE_COMPLETE))
            break;
        
        // check for max retry and skip this byte
        if (retryTimeOut == HIH6030_RETRY_MAX)
            break;
        else
            retryTimeOut++;
    }
    
    if (*pstatus == I2C1_MESSAGE_FAIL)
        return;

    // end transmission
    I2C1CONLbits.PEN = 1;

}

void HIH6030_Read(uint8_t command, uint8_t *pData)
{
    /**
     *  @Summary
     *      Read from HIH6030 slave device in Command Mode
     *  @Param
     *      command: EEPROM location / command byte
     *  @Param
     *      pData: pointer to data block that stores data
     */
    
    I2C1_MESSAGE_STATUS i2cStatus;
    i2cStatus = I2C1_MESSAGE_PENDING;
    
    uint8_t *pD, b;
    pD = pData; // #TODO: may need to make data uint16_t?
    uint16_t retryTimeOut, slaveTimeOut;
    
    // send the command to read
    HIH6030_Write(command, 0x0000, &i2cStatus);
    while(i2cStatus != I2C1_MESSAGE_COMPLETE); // delay
    
    if (i2cStatus == I2C1_MESSAGE_COMPLETE)
    {
        // start transmission; should it be Restart bit?
        I2C1CONLbits.SEN = 1;
        
        for (b = 0; b < 3; b++) // read 3 bytes, one at a time
        {
            retryTimeOut = 0;
            slaveTimeOut = 0;
        
            while (i2cStatus != I2C1_MESSAGE_FAIL)
            {
                I2C1_MasterRead(pD, 1, HIH6030_ADDRESS, &i2cStatus);
            
                while (i2cStatus == I2C1_MESSAGE_PENDING)
                {
                    // add delay here
                    uint8_t dt;
                    for (dt = 0; dt < 20; ++dt);
                    
                    // check for timeout
                    if (slaveTimeOut == HIH6030_I2C_TIMEOUT)
                        return;
                    else
                        slaveTimeOut++;
                }
            
                if (i2cStatus == I2C1_MESSAGE_COMPLETE)
                    break;
                
                // check for retry and skip this byte
                if (retryTimeOut == HIH6030_RETRY_MAX)
                    break;
                else
                    retryTimeOut++;
            }
            pD++;
        }
        
        // end transmission
        I2C1CONLbits.PEN = 1;
    }

    if (i2cStatus == I2C1_MESSAGE_FAIL)
    {
        return;
    }
    
    //return;
}

/**
 End of File
*/

