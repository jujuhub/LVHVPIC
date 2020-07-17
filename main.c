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
#include <time.h>

#include "mcc_generated_files/system.h"
#include "mcc_generated_files/mcc.h"

#define HIH6030_RETRY_MAX 100 // define retry count
#define HIH6030_I2C_TIMEOUT 50 // define slave timeout
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
    
    /*
     *      HIH6030 Relative Humidity & Temperature Sensor
     */
    
    // Enter Command Mode within 3 ms
    I2C1_MESSAGE_STATUS i2cStatus;
    i2cStatus = I2C1_MESSAGE_PENDING;
//    HIH6030_Write(0xA0, 0x0000, &i2cStatus);
//    while (i2cStatus != I2C1_MESSAGE_COMPLETE);

    // Read/Write from EEPROM location
    uint8_t cmdmodeData[3], *pD;
    pD = cmdmodeData;

//    HIH6030_Read(0x18, pD); // Alarm_High_On
    
//    i2cStatus = I2C1_MESSAGE_PENDING; // need to reset msg status?
//    HIH6030_Write(0x58, 0x3333, &i2cStatus); // set Alarm_High_On at 80% RH
//    while (i2cStatus != I2C1_MESSAGE_COMPLETE);
    
//    i2cStatus = I2C1_MESSAGE_PENDING;
//    HIH6030_Write(0x59, 0x3000, &i2cStatus); // set Alarm_High_Off at 75% RH
//    while (i2cStatus != I2C1_MESSAGE_COMPLETE);

//    HIH6030_Write(0x5A, 0x0CCD, &i2cStatus); // set Alarm_Low_On at 20% RH
//    HIH6030_Write(0x5B, 0x1000, &i2cStatus); // set Alarm_Low_Off at 25% RH

    // Enter Normal Operation
//    HIH6030_Write(0x80, 0x0000, &i2cStatus);
    
    float hum, temp, *pHum, *pTemp;
    uint8_t _status = fetch_RHT(pHum, pTemp);
    
//    while (1)
    {
        // Add your application code
        
        // Fetch RH&T data from HIH6030-021 sensor
        //float hum, temp, *pHum, *pTemp;
        //uint8_t _status = fetch_RHT(pHum, pTemp);
        
    }
    return 1; 
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

uint8_t fetch_RHT(float *pHum, float *pTemp)
{
    /**
     *  @Summary
     *      Read from HIH6030 device and return data status, RH, T
     *  @Param
     *      pHum: pointer to RH variable
     *  @Param
     *      pTemp: pointer to T variable
     */

    uint8_t sensorData[4], _status, *pD, b;
    uint16_t H_dat, T_dat, retryTimeOut, slaveTimeOut;
    pD = sensorData;

    I2C1_MESSAGE_STATUS i2cStatus;
    i2cStatus = I2C1_MESSAGE_PENDING;

    // Begin transmission
    I2C1CONLbits.SEN = 1; 
    
    // Read the RH & T data bytes
    for (b = 0; b < 4; b++)
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
                    break;
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
        
        if (i2cStatus == I2C1_MESSAGE_FAIL)
        {
            break;
        }
        
        pD++;
    }
    
    // Get status of data (first 2 bits)
    _status = (sensorData[0] >> 6);

    // Convert RH & T counts to float; #TODO: fix bitwise operations
    H_dat = (sensorData[0] << 2)*256 + sensorData[1];
    T_dat = sensorData[2]*256 + (sensorData[4] >> 2);
    
    *pHum = H_dat/16382. * 100.; // %RH
    *pTemp = T_dat/16382.*165. - 40.; // degrees C
    
    return _status;

}

/**
 End of File
*/

