/*
 * sapi_max30102.c
 *
 *  Created on: 2 oct. 2021
 *      Author: lucascsd
 */

#include "sapi_max30102.h"

/* Variables goblaes */

max30102_t _max30102;

/* Mis funciones */


void initStructMax30102()
{

	/* TODO: Inicializar parametros de estructura por default */

	_max30102._addrRead 	= READ_ADDRESS;
	_max30102._addrWrite 	= WRITE_ADDRESS;

	/* Single read */
	_max30102._dataQty		= 1;

	/* Average Samples */
	_max30102._avg			= SMP_AVE_8;

}

/* Init board MAX30102 */
bool_t max30102Init( )
{

	/* TODO: CREAR E INICIALIZAR ESTRUCTURA DE CONTROL */
	initStructMax30102();

	/* Selecciono puerto I2C e inicializo */
	if ( i2cInit(I2C0, 100000) )
	{
		printf("Se inicializa I2C correctamente\n\r");
	}

	/* Verificar conexión de MAX30102 */
	if ( !readPartID( ) )
		return false;

	printf("Part ID = 0x%x \n\r", _max30102._buffer);

	/* Lectura de Revision ID */
	readRevisionID ( );

	printf("Revision ID = 0x%x \n\r", _max30102._buffer);

	if ( !max30102_setup ( ) )
		return FALSE;

	//maskRegister ( MODE_CONFIG, ~MODE_RESET, MODE_RESET );

	return true;

}

/* Obtiene puntero de escritura */
uint8_t max30102Write ( uint8_t registerAddr, uint8_t data )
{

	uint8_t transmitDataBuffer[2];
	transmitDataBuffer[0] = registerAddr;
	transmitDataBuffer[1] = data;

	return i2cWrite(I2C0, _max30102._addrWrite, transmitDataBuffer, 2, TRUE);
}

/* Obtiene puntero de escritura */
bool_t max30102Read ( uint8_t registerAddr, uint8_t _dataQty )
{
	return i2cRead ( I2C0, _max30102._addrRead, &registerAddr, 1, TRUE, &_max30102._buffer, _dataQty, TRUE );
}

/* Funcion para obtener PART ID */
bool_t readPartID ( )
{
	return max30102Read ( PART_ID, 1 );
}

/* Funcion para obtener REVISION ID */
bool_t readRevisionID ( )
{
	return max30102Read ( REVISION_ID, 1 );
}

/* Pasar por argumento SMP_AVE_N, */
bool_t max30102_setup ( )
{
	bool_t stateMax30102;

	stateMax30102 = max30102_reset();

	if ( stateMax30102 )
		printf("Se resetea los registros del dispositivo correctamente\n\r");
	else
		return FALSE;

	/* Sample average */
	stateMax30102 = setSmpAvgFIFO ( SMP_AVE_4 );

	if ( stateMax30102 )
		printf("Cantidad de muestras por promedio configuradas correctamente\n\r");
	else
		return FALSE;

	/* Roll Over */
	stateMax30102 = rollOver ( FIFO_ROLLOVER_EN );

	if ( stateMax30102 )
		printf("Roll Over configuradas correctamente\n\r");
	else
		return FALSE;

	/* Set LED Mode */
	stateMax30102 = ledMode ( MODE_MULTI );

	if ( stateMax30102 )
		printf("Modo Leds configurado correctamente\n\r");
	else
		return FALSE;

	/* Set ADC Range Mode */
	stateMax30102 = spo2Config ( SPO2_ADC_RGE_4096 );

	if ( stateMax30102 )
		printf("Modo SPO2 configurado correctamente\n\r");
	else
		return FALSE;

	/* Set Sample Rate Mode */
	stateMax30102 = sampleRate ( SAMPLERATE_400 );

	if ( stateMax30102 )
		printf("Sample Rate configurado correctamente\n\r");
	else
		return FALSE;

	/* Set Pulse width Mode */
	stateMax30102 = pulseWidth ( PULSEWIDTH_411 );

	if ( stateMax30102 )
		printf("Pulse Width configurado correctamente\n\r");
	else
		return FALSE;

	/* Set Led_1_2 Mode */
	stateMax30102 = pulseAmplitude ( PULSEAMPLITUDE_06, LED_PULSE_AMP_1 );

	if ( stateMax30102 )
		printf("LED 1 configurado correctamente\n\r");
	else
		return FALSE;

	stateMax30102 = pulseAmplitude ( PULSEAMPLITUDE_06, LED_PULSE_AMP_2 );

	if ( stateMax30102 )
		printf("LED 2 configurado correctamente\n\r");
	else
		return FALSE;

	/* Set Led_1_2 Config */
	stateMax30102 = ledConfig ( MULTILED_CONTROL_RED, LED_CTRL_REG_12, 0 );

	if ( stateMax30102 )
		printf("LED 1 Config configurado correctamente\n\r");
	else
		return FALSE;

	stateMax30102 = ledConfig ( MULTILED_CONTROL_IR, LED_CTRL_REG_12, 4 );

	if ( stateMax30102 )
		printf("LED 2 Config configurado correctamente\n\r");
	else
		return FALSE;

	//	  clearFIFO(); //Reset the FIFO before we begin checking the sensor
	return TRUE;

}

/* Reset all configuration, threshold, and data registers to POR values */
bool_t max30102_reset ()
{

	maskRegister ( MODE_CONFIG, ~MODE_RESET, MODE_RESET );

	if ( (_max30102._buffer & MODE_RESET) == 0 )
		return TRUE;
	else
		return FALSE;

}

bool_t setSmpAvgFIFO ( uint8_t avgsamples )
{

	maskRegister ( FIFO_CONFIG, SMP_AVE_MASK, avgsamples );

	if ( _max30102._buffer == avgsamples )
		return TRUE;
	else
		return FALSE;

}

bool_t rollOver ( uint8_t _rollOver )
{

	uint8_t dataActual;

	max30102Read ( FIFO_CONFIG, 1 );

	dataActual =  _max30102._buffer;
	/* Enable roll over if FIFO over flows */
	maskRegister ( FIFO_CONFIG, ~_rollOver, _rollOver );

	if ( _max30102._buffer == dataActual | _rollOver )
		return TRUE;
	else
		return FALSE;

}

bool_t ledMode ( uint8_t _ledMode )
{

	uint8_t ledActual;

	max30102Read ( MODE_CONFIG, 1 );

	ledActual =  _max30102._buffer;

	/* Enable roll over if FIFO over flows */
	maskRegister ( MODE_CONFIG, ~_ledMode, _ledMode );

	if ( _max30102._buffer == ledActual | _ledMode )
		return TRUE;
	else
		return FALSE;

}

bool_t spo2Config ( uint8_t _sop2Mode )
{

	uint8_t spo2Actual;

	max30102Read ( SPO2_CONFIG, 1 );

	spo2Actual =  _max30102._buffer;

	/* Enable roll over if FIFO over flows */
	maskRegister ( SPO2_CONFIG, ~_sop2Mode, _sop2Mode );

	if ( _max30102._buffer == spo2Actual | _sop2Mode )
		return TRUE;
	else
		return FALSE;

}

bool_t sampleRate ( uint8_t _srMode )
{

	uint8_t srActual;

	max30102Read ( SPO2_CONFIG, 1 );

	srActual =  _max30102._buffer;

	/* Enable roll over if FIFO over flows */
	maskRegister ( SPO2_CONFIG, ~_srMode, _srMode );

	if ( _max30102._buffer == srActual | _srMode )
		return TRUE;
	else
		return FALSE;

}

bool_t pulseWidth ( uint8_t _pwMode )
{

	uint8_t srActual;

	max30102Read ( SPO2_CONFIG, 1 );

	srActual =  _max30102._buffer;

	/* Enable roll over if FIFO over flows */
	maskRegister ( SPO2_CONFIG, ~_pwMode, _pwMode );

	if ( _max30102._buffer == srActual | _pwMode )
		return TRUE;
	else
		return FALSE;

}


bool_t pulseAmplitude ( uint8_t _ledMode, uint8_t _registerLed )
{

	uint8_t ledActual;

	max30102Read ( _registerLed, 1 );

	ledActual =  _max30102._buffer;

	/* Enable roll over if FIFO over flows */
	maskRegister ( _registerLed, ~_ledMode, _ledMode );

	if ( _max30102._buffer == ledActual | _ledMode )
		return TRUE;
	else
		return FALSE;

}

bool_t ledConfig ( uint8_t _ledMode, uint8_t _registerLed, uint8_t shitf )
{

	uint8_t ledActual;

	max30102Read ( _registerLed, 1 );

	ledActual =  _max30102._buffer;

	/* Enable roll over if FIFO over flows */
	maskRegister ( _registerLed, ~_ledMode, _ledMode << shitf );

	if ( _max30102._buffer == ledActual | _ledMode )
		return TRUE;
	else
		return FALSE;

}

bool_t clearFIFO ( )
{
	if ( !max30102Write ( FIFO_WRITE_POINTER, 0x00 ) )
		return FALSE;
	if ( !max30102Write ( OVERFLOW_COUNTER, 0x00 ) )
		return FALSE;
	if ( !max30102Write ( FIFO_READ_POINTER, 0x00 ) )
		return FALSE;

	return TRUE;
}

/* Report the most recent red value */
uint32_t getRed ( void )
{
	/* Check the sensor for new data for 250ms */
	if( readNewValue() )
		return _max30102.red[_max30102.head];
	else
		return 0 ; /* Sensor failed to find new data */
}

/* Report the most recent IR value */
uint32_t getIR ( void )
{
	/* Check the sensor for new data for 250ms */
	if( readNewValue() )
		return _max30102.IR[_max30102.head];
	else
		return 0 ; /* Sensor failed to find new data */
}

/* Check for new data but give up after a certain amount of time 	*/
/* Returns true if new data was found 								*/
/* Returns false if new data was not found 							*/
bool_t readNewValue ( )
{

	if ( !check() ) /* We found new data! */
		return TRUE;
	else
		return FALSE;

}

/* Read the FIFO Write Pointer */
uint8_t getWritePointer ( )
{
	if ( max30102Read ( FIFO_WRITE_POINTER, 1 ) )
		return _max30102._buffer;
}

/* Read the FIFO Read Pointer */
uint8_t getReadPointer ( )
{
	if ( max30102Read ( FIFO_READ_POINTER, 1 ) )
			return _max30102._buffer;
}

/* Polls the sensor for new data 												*/
/* Call regularly																*/
/* If new data is available, it updates the head and tail in the main struct	*/
/* Returns number of new samples obtained										*/
//uint16_t check ( )
//{
//	/* Read register FIDO_DATA in (3-byte * number of active LED) chunks Until FIFO_RD_PTR = FIFO_WR_PTR */
//
//	uint8_t readPointer = getReadPointer();
//	uint8_t writePointer = getWritePointer();
//
//	uint8_t numberOfSamples = 0;
//
//	/* Do we have new data? */
//	if (readPointer != writePointer)
//	{
//		/* Calculate the number of readings we need to get from sensor */
//		numberOfSamples = writePointer - readPointer;
//		if (numberOfSamples < 0) numberOfSamples += 32; /* Wrap condition */
//
//		/* We now have the number of readings, now calc bytes to read */
//		//For this example we are just doing Red and IR (3 bytes each)
//		uint8_t bytesLeftToRead = numberOfSamples * 2 * 3;
//
//		//Get ready to read a burst of data from the FIFO register
//		_i2cPort->beginTransmission(MAX30105_ADDRESS);
//		_i2cPort->write(MAX30105_FIFODATA);
//		_i2cPort->endTransmission();
//
//		max30102Write ( FIFO_DATA_REGISTER,  )
//
//		//We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
//		//I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
//		//Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
//		while (bytesLeftToRead > 0)
//		{
//			int toGet = bytesLeftToRead;
//			if (toGet > I2C_BUFFER_LENGTH)
//			{
//				//If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
//				//32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
//				//32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.
//
//				toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
//			}
//
//			bytesLeftToRead -= toGet;
//
//			//Request toGet number of bytes from sensor
//			_i2cPort->requestFrom(MAX30105_ADDRESS, toGet);
//
//			while (toGet > 0)
//			{
//				sense.head++; //Advance the head of the storage struct
//				sense.head %= STORAGE_SIZE; //Wrap condition
//
//				byte temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
//				uint32_t tempLong;
//
//				//Burst read three bytes - RED
//				temp[3] = 0;
//				temp[2] = _i2cPort->read();
//				temp[1] = _i2cPort->read();
//				temp[0] = _i2cPort->read();
//
//				//Convert array to long
//				memcpy(&tempLong, temp, sizeof(tempLong));
//
//				tempLong &= 0x3FFFF; //Zero out all but 18 bits
//
//				sense.red[sense.head] = tempLong; //Store this reading into the sense array
//
//				if (activeLEDs > 1)
//				{
//					//Burst read three more bytes - IR
//					temp[3] = 0;
//					temp[2] = _i2cPort->read();
//					temp[1] = _i2cPort->read();
//					temp[0] = _i2cPort->read();
//
//					//Convert array to long
//					memcpy(&tempLong, temp, sizeof(tempLong));
//
//					tempLong &= 0x3FFFF; //Zero out all but 18 bits
//
//					sense.IR[sense.head] = tempLong;
//				}
//
//				if (activeLEDs > 2)
//				{
//					//Burst read three more bytes - Green
//					temp[3] = 0;
//					temp[2] = _i2cPort->read();
//					temp[1] = _i2cPort->read();
//					temp[0] = _i2cPort->read();
//
//					//Convert array to long
//					memcpy(&tempLong, temp, sizeof(tempLong));
//
//					tempLong &= 0x3FFFF; //Zero out all but 18 bits
//
//					sense.green[sense.head] = tempLong;
//				}
//
//				toGet -= activeLEDs * 3;
//			}
//
//		} //End while (bytesLeftToRead > 0)
//
//	} //End readPtr != writePtr
//
//	return (numberOfSamples); //Let the world know how much new data we found
//}

void maskRegister ( uint8_t _register, uint8_t _mask, uint8_t bitConfig )
{

	uint8_t registerActual = 0;

	if ( max30102Read ( _register, 1 ) )
		registerActual = _max30102._buffer;

	/* Enmascaro los bits del registro */
	registerActual = registerActual & _mask;

	/* Escribo el registro con las máscara más el bit o bits a cambiar */
	if ( max30102Write ( _register, registerActual | bitConfig ) )
	{
		delay (10);
		max30102Read ( _register, 1 );
		printf("Valor leido escrito = 0x%x \n\r", _max30102._buffer);
	}
}
