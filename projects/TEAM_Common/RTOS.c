/**
 * \file
 * \brief Real Time Operating System (RTOS) main program.
 * \author Erich Styger, erich.styger@hslu.ch
 */

#include "Platform.h"
#if PL_CONFIG_HAS_RTOS
#include "RTOS.h"
#include "FRTOS1.h"
#include "Application.h"


//static void firstTask(void){
static void firstTask(void *pvParameters){
	(void)pvParameters;	//cast

	for(;;){
		vTaskDelay(500/portTICK_RATE_MS);
	}
}

void RTOS_Init(void) {
/*! \todo Create tasks here */
  /*! \todo Create tasks here */
	xTaskHandle taskHndl;

	//<< create task
	BaseType_t res;
	res = xTaskCreate(
			firstTask,	/* function */
			"firstTask",	/* kernel awareness name */
			configMINIMAL_STACK_SIZE+10,	/* stack */
			(void*) NULL, 	/* task parameter */
			tskIDLE_PRIORITY, 	/* priority */
			&taskHndl 	/* handle */
		); //>> create task

	if (res!=pdPASS){ /* error handling */ }

	vTaskStartScheduler();		/* starts scheduler, creates IDLE task */

}

void RTOS_Deinit(void) {
  /* nothing needed for now */
}

#endif /* PL_CONFIG_HAS_RTOS */
