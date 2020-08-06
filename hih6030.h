/* 
 * File:   hih6030.h
 * Author: Julie He <juhe@ucdavis.edu>
 *
 * Created on August 4, 2020, 1:06 PM
 */

#ifndef HIH6030_H
#define	HIH6030_H

#include <stdint.h>
#include <stdlib.h>
#include <xc.h>

#include "mcc_generated_files/i2c1.h"

#ifdef	__cplusplus
    extern "C" {
#endif

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

#ifdef	__cplusplus
    }
#endif

#endif	/* HIH6030_H */

