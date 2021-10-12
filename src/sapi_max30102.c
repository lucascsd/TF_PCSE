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

/* Init Driver */
void initStructMax30102()
{

	/* TODO: Inicializar parametros de estructura por default */
	/* Direccopm de esclavo */
	_max30102._address 	=	MAX_ADDRESS;

	/* Single read */
	_max30102._dataQty	=	1;

	/* Average Samples */
	_max30102._avg		=	SMP_AVE_8;

	/* Clear register read */
	for ( uint8_t index = 0; index < 192; index++)
		_max30102.redIR[index] = 0;

}

/* Init board MAX30102 */
bool_t max30102Init( )
{

	/* TODO: CREAR E INICIALIZAR ESTRUCTURA DE CONTROL */
	initStructMax30102();

	/* Selecciono puerto I2C e inicializo */
	if ( !i2cInit(I2C0, MAX30102_I2C_RATE_STD) )
		return FALSE;

	/* Verificar conexión de MAX30102 */
	if ( !readPartID( ) )
		return false;
	printf("Part ID = 0x%x \n\r", _max30102._buffer);

	/* Lectura de Revision ID */
	if ( !readRevisionID ( ) )
		return false;
	printf("Revision ID = 0x%x \n\r", _max30102._buffer);

	if ( !max30102_setup ( ) )
		return FALSE;

	return true;
}

/* Obtiene puntero de escritura */
uint8_t max30102Write ( uint8_t registerAddr, uint8_t data )
{

	uint8_t transmitDataBuffer[2];
	transmitDataBuffer[0] = registerAddr;
	transmitDataBuffer[1] = data;

	return i2cWrite(I2C0, _max30102._address, transmitDataBuffer, 2, TRUE);
}

/* Obtiene puntero de escritura */
bool_t max30102Read ( uint8_t registerAddr, uint8_t _dataQty )
{
	return i2cRead ( I2C0, _max30102._address, &registerAddr, 1, TRUE, &_max30102._buffer, _dataQty, TRUE );
}

/* Obtiene puntero de escritura */
bool_t max30102ReadMoreSamples ( uint8_t registerAddr, uint8_t _dataQty )
{
	return i2cRead ( I2C0, _max30102._address, &registerAddr, 1, TRUE, _max30102.redIR, _dataQty, TRUE );
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
bool_t max30102_setup (  )
{
	bool_t stateMax30102;

	/* POR register */
	stateMax30102 = max30102_reset();
	if ( !stateMax30102 )
		return FALSE;

	/* Sample average */
	stateMax30102 = setSmpAvgFIFO ( SMP_AVE_8 );

	if ( !stateMax30102 )
		return FALSE;

	/* Roll Over */
	stateMax30102 = rollOver ( FIFO_ROLLOVER_EN );
	if ( !stateMax30102 )
		return FALSE;

	/* Set LED Mode */
	stateMax30102 = ledMode ( MODE_MULTI );

	if ( !stateMax30102 )
		return FALSE;

	/* Set ADC Range Mode */
	stateMax30102 = spo2Config ( SPO2_ADC_RGE_8192 );

	if ( !stateMax30102 )
		return FALSE;

	/* Set Sample Rate Mode */
	stateMax30102 = sampleRate ( SAMPLERATE_800 );

	if ( !stateMax30102 )
		return FALSE;

	/* Set Pulse width Mode */
	stateMax30102 = pulseWidth ( PULSEWIDTH_411 );

	if ( !stateMax30102 )
		return FALSE;

	/* Set Led_1 Mode */
	stateMax30102 = pulseAmplitude ( PULSEAMPLITUDE_06, LED_PULSE_AMP_1 );

	if ( !stateMax30102 )
		return FALSE;

	/* Set Led_2 Mode */
	stateMax30102 = pulseAmplitude ( PULSEAMPLITUDE_06, LED_PULSE_AMP_2 );

	if ( !stateMax30102 )
		return FALSE;

	/* Set Led_1 Config */
	stateMax30102 = ledConfig ( MULTILED_CONTROL_RED, LED_CTRL_REG_12, 0 );

	if ( !stateMax30102 )
		return FALSE;

	/* Set Led_2 Config */
	stateMax30102 = ledConfig ( MULTILED_CONTROL_IR, LED_CTRL_REG_12, 4 );

	if ( !stateMax30102 )
		return FALSE;

	/* Reset the FIFO before we begin checking the sensor */
	stateMax30102 = clearFIFO();

	if ( !stateMax30102 )
		return FALSE;

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

bool_t setSmpAvgFIFO ( avgsamples_t avgsamples )
{

	maskRegister ( FIFO_CONFIG, SMP_AVE_MASK, avgsamples );

	if ( _max30102._buffer == avgsamples )
		return TRUE;
	else
		return FALSE;

}

bool_t rollOver ( rollOver_t _rollOver )
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

bool_t ledMode ( ledMode_t _ledMode )
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

bool_t spo2Config ( spo2Mode_t _spo2Mode )
{

	uint8_t spo2Actual;

	max30102Read ( SPO2_CONFIG, 1 );

	spo2Actual =  _max30102._buffer;

	/* Enable roll over if FIFO over flows */
	maskRegister ( SPO2_CONFIG, ~_spo2Mode, _spo2Mode );

	if ( _max30102._buffer == spo2Actual | _spo2Mode )
		return TRUE;
	else
		return FALSE;

}

bool_t sampleRate ( srMode_t _srMode )
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

bool_t pulseWidth ( pwMode_t _pwMode )
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

bool_t pulseAmplitude ( ledMode_t _ledMode, registerLed_t _registerLed )
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

bool_t ledConfig ( ledMode_t _ledMode, registerLed_t _registerLed, uint8_t shitf )
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

bool_t readNewValue ( )
{
	uint16_t numberOfSamples = check();

	if ( !numberOfSamples ) /* We found new data! */
		return FALSE;

	return TRUE;

}

/* Read the FIFO Write Pointer */
uint8_t getWritePointer ( )
{
	if ( max30102Read ( FIFO_WRITE_POINTER, 1 ) )
		return _max30102._buffer;

	return 0;
}

/* Read the FIFO Read Pointer */
uint8_t getReadPointer ( )
{
	if ( max30102Read ( FIFO_READ_POINTER, 1 ) )
		return _max30102._buffer;

	return 0;
}

int16_t check ( )
{

	uint8_t readPointer = getReadPointer();
	uint8_t writePointer = getWritePointer();
	int8_t numberSamples = 0;

	/* Do we have new data? */
	if (readPointer != writePointer)
	{
		/* Calculate the number of readings we need to get from sensor */
		numberSamples = writePointer - readPointer;

		if (numberSamples < 0) numberSamples += 32; /* Wrap condition */

		uint32_t datoLeido[numberSamples];

		uint8_t bytesLeftToRead = numberSamples * 2 * 3;

		while (bytesLeftToRead > 0)
		{
			int8_t samplesToRead = bytesLeftToRead;

			if (samplesToRead > MAX30102_BUFFER_LENGTH)
				samplesToRead = MAX30102_BUFFER_LENGTH - (MAX30102_BUFFER_LENGTH % (2 * 3));

			bytesLeftToRead = bytesLeftToRead - samplesToRead;

			max30102ReadMoreSamples ( FIFO_DATA_REGISTER, bytesLeftToRead );
			uint8_t index;

			for (uint8_t i = 0; i < numberSamples; i++)
			{
				index = i * 3;
				datoLeido[i] = (_max30102.redIR[index] << 16) | (_max30102.redIR[index+1] << 8) | _max30102.redIR[index+2];
				datoLeido[i] = datoLeido[i] & 0x3FFFF;
				//printf("Dato %d leido = %d \n\r", i, datoLeido[i]);
			}

		} //FIN while (bytesLeftToRead > 0)

	} //FIN readPtr != writePtr

	return numberSamples; //Let the world know how much new data we found
}

void maskRegister ( uint8_t _register, uint8_t _mask, uint8_t bitConfig )
{

	uint8_t registerActual = 0;

	if ( max30102Read ( _register, 1 ) )
		registerActual = _max30102._buffer;

	/* Enmascaro los bits del registro */
	registerActual = registerActual & _mask;

	/* Escribo el registro con las máscara más el bit o bits a cambiar */
	max30102Write ( _register, registerActual | bitConfig );

}
