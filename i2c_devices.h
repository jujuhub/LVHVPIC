/* 
 * File:   hih6030.h
 * Author: Julie He <juhe@ucdavis.edu>
 *
 * Created on August 4, 2020
 */

#ifndef HIH6030_H
#define	HIH6030_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <xc.h>

#include "mcc_generated_files/i2c1.h"
#include "mcc_generated_files/spi2.h"

#ifdef	__cplusplus
    extern "C" {
#endif

#define I2C_MAX_RETRY 2         // max retry count
#define I2C_TIMEOUT 255         // max timeout count

#define HIH6030_ADDR 0x27       // humidity sensor
#define LTC2631_ADDR 0x73       // DAC for HV
#define MCP4725_DAC0_ADDR 0x60  // channel comparator DAC
#define MCP4725_DAC1_ADDR 0x61  // multiplicity comparator DAC
#define IMAGINARY_ADDR 0x42     // imaginary device for resets

/****************************************************************************
 *
 * Function:        I2C_Write
 * Description:     Generic write function to send data to I2C devices
 * 
 * Return Value:    Status of I2C message
 *                      I2C1_MESSAGE_FAIL
 *                      I2C1_MESSAGE_PENDING
 *                      I2C1_MESSAGE_COMPLETE
 *                      I2C1_STUCK_START
 *                      I2C1_MESSAGE_ADDRESS_NO_ACK
 *                      I2C1_DATA_NO_ACK
 *                      I2C1_LOST_STATE
 *
 ****************************************************************************/
I2C1_MESSAGE_STATUS I2C_Write(uint8_t addr, uint8_t nbytes, uint8_t *pData);

/****************************************************************************
 *
 * Function:        I2C_Read
 * Description:     Generic read function to receive data from I2C devices
 * 
 * Return Value:    Status of I2C message
 *                      I2C1_MESSAGE_FAIL
 *                      I2C1_MESSAGE_PENDING
 *                      I2C1_MESSAGE_COMPLETE
 *                      I2C1_STUCK_START
 *                      I2C1_MESSAGE_ADDRESS_NO_ACK
 *                      I2C1_DATA_NO_ACK
 *                      I2C1_LOST_STATE
 *
 ****************************************************************************/
I2C1_MESSAGE_STATUS I2C_Read(uint8_t addr, uint8_t nbytes, uint8_t *pData);

/****************************************************************************
 * 
 *  Function:       fetch_RHT
 *  Description:    Requests relative humidity and temperature data from 
 *                  HIH6030-021 sensor
 * 
 *  Return Value:   Status of data
 *                      00 - Normal Operation
 *                      01 - Stale Data
 *                      10 - Command Mode (should not happen under Normal Op)
 *                      11 - Failed Measurement
 * 
 ****************************************************************************/
uint8_t fetch_RHT(uint8_t *pData);


/****************************************************************************
 *
 * Function:        init_LTC2631
 * Description:     Initializes the LTC2631 DAC; selects reference voltage 
 *                  type (external or internal)
 *
 * Return Value:    None
 ****************************************************************************/
void init_LTC2631();


#ifdef	__cplusplus
    }
#endif

#endif	/* HIH6030_H */

