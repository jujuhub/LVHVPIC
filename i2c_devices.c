/*
 * File: i2c_devices.c
 * Author: Julie He <juhe@ucdavis.edu>
 * 
 * Description: This script contains functions used to read and write 
 * data to the devices on the I2C lines of the ds33EV PIC microcontroller 
 * 
 * Created on July 17, 2020
 */

#include "i2c_devices.h"


I2C1_MESSAGE_STATUS I2C_Write(uint8_t addr, uint8_t nbytes, uint8_t *pData)
{
    /**
     *  @Summary
     *      Simple I2C write function that WRITES nbytes of data to device with
     *      7-bit address (addr)
     *      Length of array must match nbytes and each element must be 1 byte 
     *      (8 bits) in size
     *  @Param
     *      addr: 7-bit device address
     *      nbytes: number of bytes to send; must match length of pData array
     *      pData: pointer to array holding data
     *  @Return
     *      i2c_stat: status of I2C message sent to device
     */
    
    uint16_t retryTimeOut = 0, slaveTimeOut = 0;
    
    I2C1_MESSAGE_STATUS i2c_stat;
    i2c_stat = I2C1_MESSAGE_PENDING;
    
    /* Initiate communication with device & transmit (n bytes) of data */
    while (i2c_stat != I2C1_MESSAGE_COMPLETE)
    {
        I2C1_MasterWrite(pData, nbytes, addr, &i2c_stat);
        
        while (i2c_stat == I2C1_MESSAGE_PENDING)
        {
            // check for timeout
            if (slaveTimeOut == I2C_TIMEOUT)
                break;
            else
                slaveTimeOut++;
        }
        
        // check for max retry
        if (retryTimeOut == I2C_MAX_RETRY)
            break;
        else
            retryTimeOut++;
        
        // if transmission failed or ACK was not received from any device
        if (i2c_stat == I2C1_MESSAGE_FAIL || i2c_stat == I2C1_DATA_NO_ACK)
            return i2c_stat;
    }
    
    return i2c_stat;
}

I2C1_MESSAGE_STATUS I2C_Read(uint8_t addr, uint8_t nbytes, uint8_t *pData)
{
    /**
     *  @Summary
     *      Simple I2C read function that READS nbytes of data to device with
     *      7-bit address (addr)
     *      Length of array must match nbytes and each element must be 1 byte 
     *      (8 bits) in size
     *  @Param
     *      addr: 7-bit device address
     *      nbytes: number of bytes to send; must match length of pData array
     *      pData: pointer to array holding data
     *  @Return
     *      i2c_stat: status of I2C message sent to device
     */
    
    uint16_t retryTimeOut = 0, slaveTimeOut = 0;
    
    I2C1_MESSAGE_STATUS i2c_stat;
    i2c_stat = I2C1_MESSAGE_PENDING;
    
    /* Initiate communication with device & receive (n bytes) of data */
    while (i2c_stat != I2C1_MESSAGE_COMPLETE)
    {
        I2C1_MasterRead(pData, nbytes, addr, &i2c_stat);
        
        while (i2c_stat == I2C1_MESSAGE_PENDING)
        {
            // check for timeout
            if (slaveTimeOut == I2C_TIMEOUT)
                break;
            else
                slaveTimeOut++;
        }
        
        // check for max retry
        if (retryTimeOut == I2C_MAX_RETRY)
            break;
        else
            retryTimeOut++;
        
        // if transmission failed or ACK was not received from any device
        if (i2c_stat == I2C1_MESSAGE_FAIL || i2c_stat == I2C1_DATA_NO_ACK)
            return i2c_stat;
    }
    
    return i2c_stat;
}

uint8_t fetch_RHT(uint8_t *pData)
{
    /**
     *  @Summary
     *      Read from HIH6030 device and return data status, RH, T
     *  @Param
     *      pData: pointer to variable where RH & T will be stored
     *  @Return
     *      status of message
     */

    /* Declare variables */
    uint8_t writeBuffer[1], _status;
//    uint8_t sensorData[4] = {0}, *pD;
//    pD = sensorData;
    uint16_t retryTimeOut = 0, slaveTimeOut = 0;
    writeBuffer[0] = (HIH6030_ADDR << 1); // dummy data

    I2C1_MESSAGE_STATUS i2cStatus;
    i2cStatus = I2C1_MESSAGE_PENDING;

    /* Send Measurement Request (MR) */
    while (i2cStatus != I2C1_MESSAGE_COMPLETE)
    {
        I2C1_MasterWrite(writeBuffer, 1, HIH6030_ADDR, &i2cStatus);
        
        while(i2cStatus == I2C1_MESSAGE_PENDING)
        {
            // check for timeout
            if (slaveTimeOut == I2C_TIMEOUT)
                break;
            else
                slaveTimeOut++;
        }
        
        // check for max retry
        if (retryTimeOut == I2C_MAX_RETRY)
            break;
        else
            retryTimeOut++;

        // if transmission failed (or ACK was not received from slave device?)
        if (i2cStatus == I2C1_MESSAGE_FAIL || i2cStatus == I2C1_DATA_NO_ACK)
            _status = 0x03;
            break;
    }
    
    // delay btwn MR and data fetch
    int dt;
    for (dt = 0; dt < 10; dt++);

    /* Fetch the RH & T data (4 bytes) */
    if (i2cStatus == I2C1_MESSAGE_COMPLETE)
    {
        retryTimeOut = 0;
        slaveTimeOut = 0;
        
        while (i2cStatus != I2C1_MESSAGE_FAIL)
        {
            I2C1_MasterRead(pData, 4, HIH6030_ADDR, &i2cStatus);
            
            while (i2cStatus == I2C1_MESSAGE_PENDING)
            {
                // check for timeout
                if (slaveTimeOut == I2C_TIMEOUT)
                    break;
                else
                    slaveTimeOut++;
            }

            // check for max retry
            if (retryTimeOut == I2C_MAX_RETRY)
                break;
            else
                retryTimeOut++;

            // check if bytes successfully received
            if ((i2cStatus == I2C1_MESSAGE_COMPLETE) || (I2C1STATbits.P == 1))
                break;
        }
    }
    
    else
    { // if I2C message status is anything else but I2C1_MESSAGE_COMPLETE
        _status = 0x03; // failed measurement
        return _status;
    }
    
    // another read after the first to see if data fetched is the same
//    uint8_t readBuffer[4] = {0}, *pRD;
//    pRD = readBuffer;
//    
//    if (i2cStatus == I2C1_MESSAGE_COMPLETE) // never calls on this bc MSG_FAIL
//    {
//        retryTimeOut = 0;
//        slaveTimeOut = 0;
//        
//        while (i2cStatus != I2C1_MESSAGE_FAIL)
//        {
//            I2C1_MasterRead(pRD, 4, HIH6030_ADDR, &i2cStatus);
//
//            while (i2cStatus == I2C1_MESSAGE_PENDING)
//            {
//                // check for timeout
//                if (slaveTimeOut == I2C_TIMEOUT)
//                    break;
//                else
//                    slaveTimeOut++;
//            }
//
//            // check for max retry
//            if (retryTimeOut == I2C_MAX_RETRY)
//                break;
//            else
//                retryTimeOut++;
//
//            // check if bytes successfully received
//            if ((i2cStatus == I2C1_MESSAGE_COMPLETE) || (I2C1STATbits.P == 1))
//                break;
//        }
//    }

    /* Get status of data (first 2 bits) */
    _status = (*pData >> 6) & 0x03; //(sensorData[0] >> 6) & 0x03;

    /* Convert RH&T counts to float */
//    H_dat = (sensorData[0] & 0x3F)*256 + sensorData[1]; // #TODO: FIX CONVERSIONS
//    T_dat = (sensorData[2]*256 + sensorData[3]) >> 2;
//    *pHum = H_dat; ///16382. * 100.; // %RH
//    *pTemp = T_dat; ///16382.*165. - 40.; // degrees C

    return _status;

}

void init_LTC2631()
{
    /**
     *  @Summary
     *      Select type of reference voltage, internal or external
     *  @Param
     *      refV: internal (1) or external (0)
     *  @Return
     *      none
     */
    
    bool refV = true;
    uint8_t cmd;
    
    if (refV)
        cmd = 0x60;         // INTERNAL reference; last 4 bits are don't care
    else
        cmd = 0x70;         // EXTERNAL reference; last 4 bits are don't care
    
    uint16_t slaveTimeOut = 0, retryTimeOut = 0;
    uint8_t cmdBuffer[3];
    cmdBuffer[0] = 0x60;
    cmdBuffer[1] = 0x00;
    cmdBuffer[2] = 0x00;
    
    I2C1_MESSAGE_STATUS i2c_stat;
    i2c_stat = I2C1_MESSAGE_PENDING;
    
    /* Initiate communication with DAC & transmit data (3 bytes) */
    while (i2c_stat != I2C1_MESSAGE_COMPLETE)
    {
        I2C1_MasterWrite(cmdBuffer, 3, LTC2631_ADDR, &i2c_stat);
        
        while (i2c_stat == I2C1_MESSAGE_PENDING)
        {
            // check for timeout
            if (slaveTimeOut == I2C_TIMEOUT)
                break;
            else
                slaveTimeOut++;
        }
        
        // check for max retry
        if (retryTimeOut == I2C_MAX_RETRY)
            break;
        else
            retryTimeOut++;
        
        // if transmission failed (or ACK was not received from slave device)
        if (i2c_stat == I2C1_MESSAGE_FAIL)
            // do something else?
            break;
    }
}