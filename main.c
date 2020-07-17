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
uint8_t HIH6030_Read(uint8_t command, uint8_t *pData);

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
    // Power on
    //SEN = 1;
    
    // Enter Command Mode within 3 ms
    I2C1_MESSAGE_STATUS i2cStatus; // needed for write function
    HIH6030_Write(0xA0, 0x0000, &i2cStatus);

    // Read/Write from EEPROM location 0x18 (Alarm_High_On)
    uint8_t sensorData[3];
    uint8_t *pD;
    pD = sensorData;

    HIH6030_Read(0x18, pD);
    
    HIH6030_Write(0x58, 0x3333, &i2cStatus);
    HIH6030_Write(0x59, 0x3000, &i2cStatus);

    HIH6030_Write(0x5A, 0x0CCD, &i2cStatus);
    HIH6030_Write(0x5B, 0x1000, &i2cStatus);

    // Power off / Need to end transmission???
    //PEN = 1;
    
    // Enter Normal Operation
    HIH6030_Write(0x80, 0x0000, &i2cStatus);
    
    //while (1)
    {
        // Add your application code
    }
    return 1; 
}

void HIH6030_Write(uint8_t command, uint16_t dat, I2C1_MESSAGE_STATUS *pstatus)
{
    /**
     *  @Summary
     *      Write to HIH6030 slave device
     *  @Param
     *      command: command value
     *  @Param
     *      dat: 16-bit data
     *  @Param
     *      pstatus: pointer to status variable
     */

    *pstatus = I2C1_MESSAGE_PENDING;
    uint16_t counter, timeOut = 0, slaveTimeOut = 0;
    
    uint8_t dat_H, dat_L; // need to split up data into 8-bit parts
    dat_H = (dat >> 8);
    dat_L = (uint8_t)(dat);
    
    uint8_t writeBuffer[3]; // command and data to be written to device
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
            uint8_t i; // need to use timers for delay?
            for (i = 0; i < 20; ++i);
                //printf("%d", i);
            
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
        if (timeOut == HIH6030_RETRY_MAX)
            break;
        else
            timeOut++;
    }
    
    if (*pstatus == I2C1_MESSAGE_FAIL) // #TODO: this breaks out of nothing
        return;

}

uint8_t HIH6030_Read(uint8_t command, uint8_t *pData)
{
    /**
     *  @Summary
     *      Read from HIH6030 slave device and return message status
     *  @Param
     *      command: command value or EEPROM location
     *  @Param
     *      pData: pointer to data block that stores data
     */
    
    I2C1_MESSAGE_STATUS i2cStatus;
    uint8_t *pD, i;
    pD = pData; // #TODO: may need to make data uint16_t
    uint16_t retryTimeOut, slaveTimeOut;
    
    for (i = 0; i < 3; ++i) // read 3 bytes, one at a time
    {
        HIH6030_Write(command, 0x0000, &i2cStatus); // initiate i2c communication
    
        if (i2cStatus == I2C1_MESSAGE_COMPLETE)
        {
            retryTimeOut = 0;
            slaveTimeOut = 0;
        
            while (i2cStatus != I2C1_MESSAGE_FAIL)
            {
                I2C1_MasterRead(pD, 1, HIH6030_ADDRESS, &i2cStatus);
            
                while (i2cStatus == I2C1_MESSAGE_PENDING)
                {
                    // add delay here
                    uint8_t i;
                    for (i = 0; i < 20; ++i)
                        printf("%d", i);
                    
                    // check for timeout
                    if (slaveTimeOut == HIH6030_I2C_TIMEOUT)
                        return 0;
                    else
                        slaveTimeOut++;
                }
            
                if (i2cStatus == I2C1_MESSAGE_COMPLETE)
                    break;
            
                if (retryTimeOut == HIH6030_RETRY_MAX)
                    break;
                else
                    retryTimeOut++;
            }
        }
        
        if (i2cStatus == I2C1_MESSAGE_FAIL)
        {
            return 0;
            break;
        }
        
        pD++;
    }
    
    return 1;
}

char fetch_RHT(uint8_t addr, uint8_t *pHum, uint8_t *pTemp)
{
    /**
     *  @Summary
     *      Read from HIH6030 device and return data status, RH, T
     *  @Param
     *      addr: address of targeted sensor
     *  @Param
     *      pHum: pointer to RH variable
     *  @Param
     *      pTemp: pointer to T variable
     */
    
    I2C1_MESSAGE_STATUS i2cStatus;
    uint8_t sensorData[4], _status;
    uint8_t *pD;
    uint16_t H_dat, T_dat;
    
    // Initiate data fetch
    HIH6030_Write(0x80, 0x0000, &i2cStatus);
    
    // Read the RH and T high and low bytes
    HIH6030_Read(command???, *pD);
    
    return _status;
}

/**
 End of File
*/

