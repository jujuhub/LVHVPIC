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


void I2C_Write(uint8_t addr, uint8_t nbytes, uint8_t *pData)
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
     *      none
     */
    
    uint16_t retryTimeOut = 0, slaveTimeOut = 0;
    uint8_t *dataBuffer; // may not actually need this; can pass ptr directly
    uint8_t n;
    
    // load the data into a buffer array
    dataBuffer = (uint8_t *) malloc (nbytes + 1);
    for (n = 0; n < nbytes; ++n)
    {
        dataBuffer[n] = *(pData + n);
    }
    
    I2C1_MESSAGE_STATUS i2c_stat;
    i2c_stat = I2C1_MESSAGE_PENDING;
    
    /* Initiate communication with device & transmit (n bytes) of data */
    while (i2c_stat != I2C1_MESSAGE_COMPLETE)
    {
        I2C1_MasterWrite(dataBuffer, nbytes, addr, &i2c_stat);
        
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
        if (i2c_stat == I2C1_MESSAGE_FAIL)
            // do something else?
            break;
    }
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
    writeBuffer[0] = (HIH6030_ADDR << 1); // dummy data

    I2C1_MESSAGE_STATUS i2cStatus;
    i2cStatus = I2C1_MESSAGE_PENDING;

    /* Begin transmission */
//    I2C1CONLbits.SEN = 1;
//    while (I2C1CONLbits.SEN == 1); // wait for SEN bit to clear

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
            I2C1_MasterRead(pD, 4, HIH6030_ADDR, &i2cStatus);
            
//            uint8_t dt;
//            for (dt = 0; dt < 255; ++dt);
//            for (dt = 0; dt < 255; ++dt);
            
            while (i2cStatus == I2C1_MESSAGE_PENDING)
            {
                // add delay here
                
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
    
    // another read after the first to see if data fetched is the same
    uint8_t readBuffer[4] = {0}, *pRD;
    pRD = readBuffer;
    
    if (i2cStatus == I2C1_MESSAGE_COMPLETE) // never calls on this bc MSG_FAIL
    {
        retryTimeOut = 0;
        slaveTimeOut = 0;
        
        while (i2cStatus != I2C1_MESSAGE_FAIL)
        {
            I2C1_MasterRead(pRD, 4, HIH6030_ADDR, &i2cStatus);

            while (i2cStatus == I2C1_MESSAGE_PENDING)
            {
                // add delay here
                
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
        I2C1_MasterWrite(writeBuffer, 3, HIH6030_ADDR, pstatus);
        
        // wait for msg to be sent or status changed
        while (*pstatus == I2C1_MESSAGE_PENDING)
        {
            // add some delay here
            uint8_t dt; // need to use timers for delay?
            for (dt = 0; dt < 20; ++dt);
            
            // check for timeout
            if (slaveTimeOut == I2C_TIMEOUT)
                break;
            else
                slaveTimeOut++;
        }
        if ((slaveTimeOut == I2C_TIMEOUT) || 
                (*pstatus == I2C1_MESSAGE_COMPLETE))
            break;
        
        // check for max retry and skip this byte
        if (retryTimeOut == I2C_MAX_RETRY)
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
                I2C1_MasterRead(pD, 1, HIH6030_ADDR, &i2cStatus);
            
                while (i2cStatus == I2C1_MESSAGE_PENDING)
                {
                    // add delay here
                    uint8_t dt;
                    for (dt = 0; dt < 20; ++dt);
                    
                    // check for timeout
                    if (slaveTimeOut == I2C_TIMEOUT)
                        return;
                    else
                        slaveTimeOut++;
                }
            
                if (i2cStatus == I2C1_MESSAGE_COMPLETE)
                    break;
                
                // check for retry and skip this byte
                if (retryTimeOut == I2C_MAX_RETRY)
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