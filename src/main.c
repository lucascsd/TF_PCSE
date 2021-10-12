/*=============================================================================
 * Author: Lucas Zalazar <lucas.zalazar6@gmail.com>
 * Date: 2021/09/30
 * Version: 1
 *===========================================================================*/

/*=====[Inclusions of function dependencies]=================================*/

#include "sapi_max30102.h"
#include "max30102_CIAA_port.h"

#include "sapi.h"

/*=====[Definition macros of private constants]==============================*/

/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

/*=====[Main function, program entry point after power on or reset]==========*/

int main( void )
{

	max30102_t driver_max30102;

	/* ------------- INICIALIZACIONES ------------- */
   boardConfig();

   // Inicializar la IMU
   printf("Inicializando MAX30102...\r\n" );
   bool_t status;
   status = max30102Init();
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
	   readNewValue();
	   clearFIFO(); //Reset the FIFO before we begin checking the sensor
	   delay(1000);
   }

   // YOU NEVER REACH HERE, because this program runs directly or on a
   // microcontroller and is not called by any Operating System, as in the 
   // case of a PC program.
   return 0;
}
