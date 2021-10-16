/*=============================================================================
 * Author: Lucas Zalazar <lucas.zalazar6@gmail.com>
 * Date: 2021/09/30
 * Version: 1
 *===========================================================================*/

/*=====[Inclusions of function dependencies]=================================*/

#include "sapi_max30102.h"
#include "max30102_CIAA_port.h"
#include "main.h"


#include "sapi.h"

/*=====[Definition macros of private constants]==============================*/

/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

/*=====[Main function, program entry point after power on or reset]==========*/

int main( void )
{

	max30102_t driver_max30102;
	float_t spo2Data;
	uint32_t BPM;
	/* ------------- INICIALIZACIONES ------------- */
	boardConfig();

	/* Inicializo driver para lectura de MAX30102 */
	driver_max30102._i2cPortFn 	= i2cInit_CIAA_port;
	driver_max30102._i2cWriteFn 	= max30102Write_CIAA_port;
	driver_max30102._i2cReadFn 	= max30102Read_CIAA_port;
	driver_max30102._delay		= delay_CIAA_port;

	delayInit( &beatTime, RET_BPM );
	printf("\e[1;1H\e[2J");
	// Inicializar la IMU
	printf("Inicializando MAX30102...\r\n" );
	bool_t status;
	status = max30102_Init( driver_max30102 );
	/* TODO: REVISAR */
	if( status == FALSE ){
		printf( "MAX30102 no inicializado, chequee las conexiones:\r\n\r\n" );
		printf( "MAX30102 ---- EDU-CIAA-NXP\r\n\r\n" );
		printf( "    VCC ---- 3.3V\r\n" );
		printf( "    GND ---- GND\r\n" );
		printf( "    SCL ---- SCL\r\n" );
		printf( "    SDA ---- SDA\r\n" );
		printf( "Se detiene el programa.\r\n" );
		while(1);
	}
	printf("MAX30102 inicializado correctamente.\r\n\r\n" );

	// ----- Repeat for ever -------------------------
	while( true ) {
		/* Lectura de los leds IR y RED del modulo */

//		delay ( 1000 );
//		spo2Data = max30102_readNewValue();
//		printf("\033[A\33[2KT\rSaturacion de oxigeno en sangre SPO2 = %1.2f\r\n", spo2Data );

		BPM = max30102_hearBeat();
		max30102_clearFIFO();
		printf("\033[A\33[2KT\rBPM = %d\r\n", BPM );
	}

	// YOU NEVER REACH HERE, because this program runs directly or on a
	// microcontroller and is not called by any Operating System, as in the
	// case of a PC program.
	return 0;
}
