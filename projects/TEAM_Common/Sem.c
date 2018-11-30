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
	for(;;){
		xSemaphoreTake(sem,10);
		LED2_Neg();
	}
	//xMutex = xSemaphoreCreateMutex();

}

static void vMasterTask(void *pvParameters) {
  /*! \todo send semaphore from master task to slave task */
	//xSemaphoreCreateBinary();
	//xMutex = xSemaphoreCreateMutex();
	for(;;){
		xSemaphoreGive(sem);
		vTaskDelay(500/portTICK_RATE_MS);
	}

}

void SEM_Deinit(void) {
}

/*! \brief Initializes module */

void SEM_Init(void) {
	  // ========== [ task ] ==========
	 xTaskHandle taskHndlSem;

	  // ----- | slave | -----
	  if(xTaskCreate(
		  vSlaveTask,
		  "vSlaveTask",
		  3000/sizeof(StackType_t),
		  (void*) NULL,
		  tskIDLE_PRIORITY+2,
		  &taskHndlSem
	  )!=pdPASS){
		  /* error handling */
	  }
//	  // ----- | master| -----
	  if(xTaskCreate(
		  vMasterTask,
		  "vMasterTask",
		  500/sizeof(StackType_t),
		  (void*) NULL,
		  tskIDLE_PRIORITY+1,
		  &taskHndlSem
	  )!=pdPASS){
		  /* error handling */
	  }
}
#endif /* PL_CONFIG_HAS_SEMAPHORE */
