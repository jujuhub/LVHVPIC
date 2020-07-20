/* 
 * File:   hih6030.h
 * Author: physicslaptop
 *
 * Created on July 17, 2020, 11:56 AM
 */

#ifndef HIH6030_H
#define	HIH6030_H

#ifdef	__cplusplus // Provide C++ compatibility
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

