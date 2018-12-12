/**
 * \file
 * \brief Semaphore usage
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * Module using semaphores.
 */

/**
 * \file
 * \brief Semaphore usage
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * Module using semaphores.
 */

#include "Platform.h" /* interface to the platform */
#if PL_CONFIG_HAS_SEMAPHORE
#include "FRTOS1.h"
#include "Sem.h"
#include "LED.h"

static xSemaphoreHandle sem = NULL;

static void vSlaveTask(void *pvParameters) {
  /*! \todo Implement functionality */
	// ========== [ loop ] ==========
	for(;;){
		xSemaphoreTake(sem,10);
		LED2_Neg();
	}
	//xMutex = xSemaphoreCreateMutex(); not used

}

static void vMasterTask(void *pvParameters) {
  /*! \todo send semaphore from master task to slave task */
	xTaskHandle taskHndlSem2;
	sem = xSemaphoreCreateBinary();
	//xMutex = xSemaphoreCreateMutex(); not used
	// ========== [ task - slave ] ==========
	  if(xTaskCreate(
		  vSlaveTask,						/* function */
		  "vSlaveTask",						/* kernel awareness name */
		  configMINIMAL_STACK_SIZE+100,		/* stack */
		  sem,								/* task parameter - sem as parameter */
		  tskIDLE_PRIORITY+2,				/* priority */
		  &taskHndlSem2						/* handle */
	  )!=pdPASS){
		  for(;;);/* error handling */
	  }
	  // ========== [ error check ] ==========
	  if(sem == NULL){
		  for(;;);/* error handling */
	  }
	  // ========== [ loop ] ==========
	  for(;;){
		  xSemaphoreGive(sem);
		  // pdMS_TO_TICKS: Converts a time in milliseconds to a time in ticks
		  vTaskDelay(pdMS_TO_TICKS(500)/portTICK_RATE_MS);
	  }
}

void SEM_Deinit(void) {
}

/*! \brief Initializes module */
void SEM_Init(void) {
	 //xTaskHandle taskHndlSem1;
	 // ========== [ task - master ] ==========
//	  if(xTaskCreate(
//		  vMasterTask,						/* function */
//		  "vMasterTask",					/* kernel awareness name */
//		  configMINIMAL_STACK_SIZE+100,		/* stack */
//		  (void*) NULL,						/* task parameter */
//		  tskIDLE_PRIORITY+1,				/* priority */
//		  &taskHndlSem1						/* handle */
//	  )!=pdPASS){
//		  for(;;);/* error handling */
//	  }
}
#endif /* PL_CONFIG_HAS_SEMAPHORE */
