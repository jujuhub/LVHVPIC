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

#ifdef	__cplusplus
    extern "C" {
#endif

#define I2C_MAX_RETRY 255       // max retry count
#define I2C_TIMEOUT 255         // max timeout count

#define HIH6030_ADDR 0x27
#define LTC2631_ADDR 0x73
#define MCP4725_DAC0_ADDR 0x60
#define MCP4725_DAC1_ADDR 0x61


/****************************************************************************
 *
 * Function:        I2C_Write
 * Description:     Simple function to write data to I2C devices
 * 
 * Return Value:    none
 *
 ****************************************************************************/
void I2C_Write(uint8_t addr, uint8_t nbytes, uint8_t *pData);

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
 * 
 ****************************************************************************/
uint8_t fetch_RHT(uint16_t *pHum, uint16_t *pTemp);

/****************************************************************************
 *
 * Function:        HIH6030_Write
 * Description:     Write to HIH6030 sensor during Command Mode
 * 
 * Return Value:    None
 * 
 ****************************************************************************/
void HIH6030_Write(uint8_t command, uint16_t dat, I2C1_MESSAGE_STATUS *pstatus);

/****************************************************************************
 * 
 * Function:        HIH6030_Read
 * Description:     Read Command Mode data from HIH6030 sensor
 * 
 * Return Value:    None
 *
 ****************************************************************************/
void HIH6030_Read(uint8_t command, uint8_t *pData);

/****************************************************************************
 *
 * Function:        init_LTC2631
 * Description:     
 *
 * Return Value:    None
 ****************************************************************************/
void init_LTC2631();


#ifdef	__cplusplus
    }
#endif

#endif	/* HIH6030_H */

