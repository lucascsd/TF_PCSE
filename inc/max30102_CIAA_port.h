/*
 * max30102_CIAA_port.h
 *
 *  Created on: 11 oct. 2021
 *      Author: lucascsd
 */

#ifndef PRACTICAFINAL_PDM_PCSE_INC_MAX30102_CIAA_PORT_H_
#define PRACTICAFINAL_PDM_PCSE_INC_MAX30102_CIAA_PORT_H_

#include "sapi_max30102.h"

#include "sapi.h"

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"
#include "sapi_peripheral_map.h"

/*==================[c++]====================================================*/

/*==================[macros]=================================================*/

#define i2cConfig i2cInit

/*==================[typedef]================================================*/

/*==================[external functions declaration]=========================*/

bool_t i2cInit_CIAA_port( i2cMap_t i2cNumber, uint32_t clockRateHz );

/* Funciones de lectura y escritura */
bool_t max30102Write_CIAA_port ( i2cMap_t  i2cNumber, uint8_t  i2cSlaveAddress, uint8_t registerAddr, uint8_t data );

bool_t max30102Read_CIAA_port ( i2cMap_t  i2cNumber, uint8_t  i2cSlaveAddress, uint8_t registerAddr, uint8_t* receiveDataBuffer, uint16_t receiveDataBufferSize );

void delay_CIAA_port(uint32_t millisecs);

/*==================[ISR external functions declaration]=====================*/

//bool_t spiRead_CIAA_port(uint8_t* buffer, uint32_t bufferSize);
//uint8_t spiReadRegister_CIAA_port(uint8_t reg);
//void spiWrite_CIAA_port(uint8_t* buffer, uint32_t bufferSize);
//void spiWriteByte_CIAA_port(uint8_t data);
//void delay_CIAA_port(uint32_t millisecs);

#endif /* PRACTICAFINAL_PDM_PCSE_INC_MAX30102_CIAA_PORT_H_ */
