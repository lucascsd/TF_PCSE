/*
 * sapi_max30102.h
 *
 *  Created on: 2 oct. 2021
 *      Author: lucascsd
 */

#ifndef PRACTICAFINAL_PDM_PCSE_INC_SAPI_MAX30102_H_
#define PRACTICAFINAL_PDM_PCSE_INC_SAPI_MAX30102_H_

/*
 * Libreria para el manejo del Pulsioximetro MAX30102 para la plataforma EDU-CIAA-NXP
 */

#include "sapi.h"

/* REGISTER MAPS */

/* STATUS REGISTER */
#define INT_STATUS_1          	0x00	/* Read only */
#define INT_STATUS_2          	0x01	/* Read only */
#define INT_ENABLE_1          	0x02	/* Read / Write */
#define INT_ENABLE_2          	0x03	/* Read / Write */

/* FIFO */
#define FIFO_WRITE_POINTER		0x04	/* Read / Write */
#define OVERFLOW_COUNTER		0x05	/* Read / Write */
#define FIFO_READ_POINTER		0x06	/* Read / Write */
#define FIFO_DATA_REGISTER		0x07	/* Read / Write */

/* CONFIGURATION */
#define FIFO_CONFIG				0x08	/* Read / Write */
#define MODE_CONFIG				0x09	/* Read / Write */
#define SPO2_CONFIG				0x0A	/* Read / Write */
#define LED_PULSE_AMP_1			0x0C	/* Read / Write */
#define LED_PULSE_AMP_2			0x0D	/* Read / Write */
#define LED_CTRL_REG_12			0x11	/* Read / Write */
#define LED_CTRL_REG_34			0x12	/* Read / Write */

/* TEMPERATURE */
#define TEMP_INTEGER			0x1F	/* Read only */
#define TEMP_FRACTION			0x20	/* Read only */
#define TEMP_CONFIG				0x21	/* Read / Write */

/* PART ID */
#define REVISION_ID         	0xFE	/* Read only */
#define PART_ID             	0xFF	/* Read only */

/************************************** REGISTERS VALUE *******************************************/

/* I2C SLAVE ID */
#define PART_ID_POR				0x15

/* Enable interrupts */
#define INT_ENB_A_FULL			((uint8_t)0x80)
#define INT_ENB_PPG_RDY			((uint8_t)0x40)
#define INT_ENB_ALC_OVF			((uint8_t)0x20)
#define INT_ENB_DIE_TEMP_RDY	((uint8_t)0x02)

/* Mode configuration */
#define MODE_SHDN				((uint8_t)0x80)
#define MODE_RESET				((uint8_t)0x40)
#define MODE_TEMP_EN			((uint8_t)0x01)
#define MODE_HR					((uint8_t)0x02)
#define MODE_SPO2				((uint8_t)0x03)
#define MODE_MULTI				((uint8_t)0x07)

/* SpO2 ADC Range Control (18-Bit Resolution) */
#define SPO2_ADC_RGE_2048 		((uint8_t)0x00)		/* 00000000 */
#define SPO2_ADC_RGE_4096 		((uint8_t)0x20)		/* 00100000 */
#define SPO2_ADC_RGE_8192 		((uint8_t)0x40)		/* 01000000 */
#define SPO2_ADC_RGE_16384 		((uint8_t)0x60)		/* 01100000 */

/* FIFO Configuration SMP_AVE[2:0] */
#define SMP_AVE_0				((uint8_t)0x00)		/* 000XXXXX = No averaging */
#define SMP_AVE_2				((uint8_t)0x20)		/* 001XXXXX = 2  */
#define SMP_AVE_4				((uint8_t)0x40)		/* 010XXXXX = 4  */
#define SMP_AVE_8				((uint8_t)0x60)		/* 011XXXXX = 8  */
#define SMP_AVE_16				((uint8_t)0x80)		/* 101XXXXX = 16 */
#define SMP_AVE_32				((uint8_t)0xA0)		/* 101XXXXX = 32 */
#define SMP_AVE_32A				((uint8_t)0xC0)		/* 110XXXXX = 32 */
#define SMP_AVE_MASK			((uint8_t)0xE0)		/* 111XXXXX = 32 */

/* Sample Rate values */
static const uint8_t SAMPLERATE_50 =	0x00;
static const uint8_t SAMPLERATE_100 =	0x04;
static const uint8_t SAMPLERATE_200 =	0x08;
static const uint8_t SAMPLERATE_400 =	0x0C;
static const uint8_t SAMPLERATE_800 =	0x10;
static const uint8_t SAMPLERATE_1000 =	0x14;
static const uint8_t SAMPLERATE_1600 =	0x18;
static const uint8_t SAMPLERATE_3200 =	0x1C;

/* Pulse Width values */
static const uint8_t PULSEWIDTH_69 = 	0x00;
static const uint8_t PULSEWIDTH_118 = 	0x01;
static const uint8_t PULSEWIDTH_215 = 	0x02;
static const uint8_t PULSEWIDTH_411 = 	0x03;

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
static const uint8_t PULSEAMPLITUDE_00 = 	0x00;
static const uint8_t PULSEAMPLITUDE_06 = 	0x1F;
static const uint8_t PULSEAMPLITUDE_12 = 	0x3F;
static const uint8_t PULSEAMPLITUDE_25 = 	0x7F;
static const uint8_t PULSEAMPLITUDE_50 = 	0xFF;

/* Multi-led values */
static const uint8_t MULTILED_CONTROL_OFF	=	0x00;
static const uint8_t MULTILED_CONTROL_RED	=	0x01;
static const uint8_t MULTILED_CONTROL_IR	=	0x02;
static const uint8_t MULTILED_CONTROL_OFF1	=	0x03;
static const uint8_t MULTILED_CONTROL_OFF2 	= 	0x04;

/* FIFO Rolls on Full */
#define FIFO_ROLLOVER_EN				((uint8_t)0x10)		/* Bit 4 = 1 */
#define FIFO_ROLLOVER_DE				((uint8_t)0x00)		/* Bit 4 = 0 */
/* I2C baudrate */
#define MAX30102_I2C_RATE				400000 // 400 kHz

#define STORAGE_SIZE					3

/* Slave ID Description */
typedef enum {
   WRITE_ADDRESS 	= 0x57, //0xAE,
   READ_ADDRESS 	= 0x57 //0xAF
} slave_address_t;

typedef struct
{
	/* Direccion del dispositivo R/W */
	slave_address_t _addrWrite;
	slave_address_t _addrRead;

	uint8_t			registerAddress;

	uint8_t			_avg;

	uint8_t 		_pW;
	uint8_t			_sr;
	uint8_t 		_ledC;
	uint8_t			_hr;

	/* buffer para lectura */
	uint8_t 		_buffer;

	/* buffer para datos de LED RED e IR */
	uint32_t 		red[STORAGE_SIZE];
	uint32_t 		IR[STORAGE_SIZE];

	uint8_t head;
	uint8_t tail;

	uint8_t			_dataQty;

	uint8_t 		_PartID;
	uint8_t			_RevisionID;

} max30102_t;

///**
// * @brief  MAX30102 driver extended internal structure definition
// */
//typedef struct
//{
//	OXIMETER_StatusTypeDef (*Enable_Free_Fall_Detection) (void);
//	OXIMETER_StatusTypeDef (*Disable_Free_Fall_Detection) (void);
//	OXIMETER_StatusTypeDef (*Get_Status_Free_Fall_Detection) (uint8_t *);
//} MAX30102_DrvExtTypeDef;

uint32_t HR;      // Last heart rate datapoint
uint32_t SPO2;    // Last oximetry datapoint

/* Mis funciones */
void initStructMax30102();

/* Inicializar device */
bool_t max30102Init();

/* Funciones de lectura y escritura */
uint8_t max30102Write ( uint8_t registerAddr, uint8_t data );
bool_t max30102Read ( uint8_t registerAddr, uint8_t _dataQty );

/* Lectura de Part ID y Revision ID */
bool_t readPartID ( void );
bool_t readRevisionID ( void );

bool_t max30102_setup ( void );
bool_t max30102_reset ( void );
bool_t setSmpAvgFIFO (uint8_t avgsamples);
bool_t rollOver ( uint8_t _rollOver );
bool_t ledMode ( uint8_t _ledMode );
bool_t spo2Config ( uint8_t _sop2Mode );
bool_t sampleRate ( uint8_t _srMode );
bool_t pulseWidth ( uint8_t _pwMode );
bool_t pulseAmplitude ( uint8_t _ledMode, uint8_t _registerLed );
bool_t ledConfig ( uint8_t _ledMode, uint8_t _registerLed, uint8_t shitf );
bool_t clearFIFO ( void );
uint32_t getRed ( void );
uint32_t getIR ( void );
bool_t readNewValue ( void );
uint16_t check ( void );
uint8_t getWritePointer ( void );
uint8_t getReadPointer ( void );
/* Funcion para enmascarar registros */
void maskRegister ( uint8_t _register, uint8_t _mask, uint8_t bitMask );



#endif /* PRACTICAFINAL_PDM_PCSE_INC_SAPI_H_ */
