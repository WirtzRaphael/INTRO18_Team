/**
 * \file
 * \brief Main application file
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * This provides the main application entry point.
 */

#include "Platform.h"
#include "Application.h"
#include "Event.h"
#include "LED.h"
#include "Trigger.h"
#include "WAIT1.h"
#include "CS1.h"
#include "KeyDebounce.h"
#include "CLS1.h"
#include "KIN1.h"
#if PL_CONFIG_HAS_KEYS
  #include "Keys.h"
#endif
#if PL_CONFIG_HAS_SHELL
  #include "CLS1.h"
  #include "Shell.h"
  #include "RTT1.h"
#endif
#if PL_CONFIG_HAS_BUZZER
  #include "Buzzer.h"
#endif
#if PL_CONFIG_HAS_RTOS
  #include "FRTOS1.h"
  #include "RTOS.h"
#endif
#if PL_CONFIG_HAS_QUADRATURE
  #include "Q4CLeft.h"
  #include "Q4CRight.h"
#endif
#if PL_CONFIG_HAS_MOTOR
  #include "Motor.h"
#endif
#if PL_CONFIG_BOARD_IS_ROBO_V2
  #include "PORT_PDD.h"
#endif
#if PL_CONFIG_HAS_LINE_FOLLOW
  #include "LineFollow.h"
#endif
#if PL_CONFIG_HAS_LCD_MENU
  #include "LCD.h"
#endif
#if PL_CONFIG_HAS_SNAKE_GAME
  #include "Snake.h"
#endif
#if PL_CONFIG_HAS_REFLECTANCE
  #include "Reflectance.h"
#endif
#if PL_CONFIG_HAS_REMOTE
  #include "RStdIO.h"
#endif
#include "Sumo.h"

#if PL_CONFIG_HAS_EVENTS

static void BtnMsg(int btn, const char *msg) {
#if PL_CONFIG_HAS_SHELL
  #if PL_CONFIG_HAS_SHELL_QUEUE
    uint8_t buf[48];

    UTIL1_strcpy(buf, sizeof(buf), "Button pressed: ");
    UTIL1_strcat(buf, sizeof(buf), msg);
    UTIL1_strcat(buf, sizeof(buf), ": ");
    UTIL1_strcatNum32s(buf, sizeof(buf), btn);
    UTIL1_strcat(buf, sizeof(buf), "\r\n");
    SHELL_SendString(buf);
  #else
    CLS1_SendStr("Button pressed: ", CLS1_GetStdio()->stdOut);
    CLS1_SendStr(msg, CLS1_GetStdio()->stdOut);
    CLS1_SendStr(": ", CLS1_GetStdio()->stdOut);
    CLS1_SendNum32s(btn, CLS1_GetStdio()->stdOut);
    CLS1_SendStr("\r\n", CLS1_GetStdio()->stdOut);
  #endif /* PL_CONFIG_HAS_SHELL_QUEUE */
#endif /* PL_CONFIG_HAS_SHELL */
}

void APP_EventHandler(EVNT_Handle event) {
  /*! \todo handle events */
  switch(event) {

  case EVNT_STARTUP:
    {
	//BUZ_Beep(300,1000);
	 CLS1_SendStr("startup...\n", CLS1_GetStdio()->stdOut);
      int i;
      for (i=0;i<5;i++) {
        //LED1_Neg();
        WAIT1_Waitms(50);
      }
      LED1_Off();
    }
    break;

  case EVNT_LED_HEARTBEAT:
    LED2_Neg();
    break;
//---
#if PL_CONFIG_NOF_KEYS>=1	//rw: depends on PL_LOCAL_CONFIG_NOF_KEYS, in KeyDebounce.c
  case EVNT_SW1_PRESSED:
	  BtnMsg(1, "pressed");
     break;
  case EVNT_SW1_RELEASED:
	  //BtnMsg(1,"released");
#if PL_CONFIG_BOARD_IS_ROBO
#if PL_CONFIG_HAS_LINE_FOLLOW
	  LF_StartStopFollowing();
#endif /* HAS_LINE_FOLLOW */
#endif /* IS_ROBO */
	  break;
  case EVNT_SW1_LONG_PRESSED:
	  //BtnMsg(1,"long pressed");
	  break;
  case EVNT_SW1_LONG_RELEASED:
	  //BtnMsg(1,"long released");
#if PL_CONFIG_BOARD_IS_REMOTE	/* push button left */
#if PL_CONFIG_HAS_REMOTE
	  (void)RSTDIO_SendToTxStdio(RSTDIO_QUEUE_TX_IN,
			  "pid status\r\n",
			  sizeof("pid status\r\n")-1);
#endif /* HAS_REMOTE */
#endif /* IS_REMOTE */
	  break;
#endif /* PL_CONFIG_NOF_KEYS */
//---
#if PL_CONFIG_NOF_KEYS>=2	/* push button right */
  case EVNT_SW2_PRESSED:
     BtnMsg(2, "pressed");
     break;
  case EVNT_SW2_RELEASED:
     //BtnMsg(2, "released");
     break;
  case EVNT_SW2_LONG_PRESSED:
     //BtnMsg(2, "long pressed");
     break;
  case EVNT_SW2_LONG_RELEASED:
     //BtnMsg(2, "long released");
#if PL_CONFIG_HAS_REMOTE
	  (void)RSTDIO_SendToTxStdio(RSTDIO_QUEUE_TX_IN,
			  "ref status\r\n",
			  sizeof("ref status\r\n")-1);
#endif /* HAS_REMOTE */
     break;
#endif /* PL_CONFIG_NOF_KEYS */
//---
#if PL_CONFIG_NOF_KEYS>=3	/* push button up */
  case EVNT_SW3_PRESSED:
     BtnMsg(3, "pressed");
     break;
  case EVNT_SW3_RELEASED:
     //BtnMsg(3, "released");
#if PL_CONFIG_HAS_REMOTE
     (void)RSTDIO_SendToTxStdio(RSTDIO_QUEUE_TX_IN,
    		 "line start\r\n",
			 sizeof("line start\r\n")-1);
#endif
     break;
  case EVNT_SW3_LONG_PRESSED:
     //BtnMsg(3, "long pressed");
     break;
  case EVNT_SW3_LONG_RELEASED:
     //BtnMsg(3, "long released");
     break;
#endif /* PL_CONFIG_NOF_KEYS */
//---
#if PL_CONFIG_NOF_KEYS>=4		/* push button middle */
  case EVNT_SW4_PRESSED:
     BtnMsg(4, "pressed");
     break;
  case EVNT_SW4_RELEASED:
     //BtnMsg(4, "released");
     break;
  case EVNT_SW4_LONG_PRESSED:
     //BtnMsg(4, "long pressed");
#if PL_CONFIG_HAS_REMOTE
	  (void)RSTDIO_SendToTxStdio(RSTDIO_QUEUE_TX_IN,
			  "buzzer buz 800 400\r\n",
			  sizeof("buzzer buz 800 400\r\n")-1);
#endif /* HAS_REMOTE */
     break;
  case EVNT_SW4_LONG_RELEASED:
     //BtnMsg(4, "long released");
     break;
#endif /* PL_CONFIG_NOF_K§EYS */
//---
#if PL_CONFIG_NOF_KEYS>=5	/* push button down */
  case EVNT_SW5_PRESSED:
     BtnMsg(5, "pressed");
#if PL_CONFIG_HAS_REMOTE
     (void)RSTDIO_SendToTxStdio(RSTDIO_QUEUE_TX_IN,
    		 "line stop\r\n",
			 sizeof("line stop\r\n")-1);
#endif
     break;
  case EVNT_SW5_RELEASED:
    //BtnMsg(5, "released");
     break;
  case EVNT_SW5_LONG_PRESSED:
     //BtnMsg(5, "long pressed");
     break;
  case EVNT_SW5_LONG_RELEASED:
     //BtnMsg(5, "long released");
     break;
#endif /* PL_CONFIG_NOF_KEYS */
//---
#if PL_CONFIG_NOF_KEYS>=6
  case EVNT_SW6_PRESSED:
     BtnMsg(6, "pressed");
     break;
  case EVNT_SW6_RELEASED:
     //BtnMsg(6, "released");
     break;
  case EVNT_SW6_LONG_PRESSED:
     //BtnMsg(6, "long pressed");
     break;
  case EVNT_SW6_LONG_RELEASED:
    //BtnMsg(6, "long released");
     break;
#endif /* PL_CONFIG_NOF_KEYS */
//---
#if PL_CONFIG_NOF_KEYS>=7
  case EVNT_SW7_PRESSED:
     BtnMsg(7, "pressed");
     break;
  case EVNT_SW7_RELEASED:
     //BtnMsg(7, "released");
     break;
  case EVNT_SW7_LONG_PRESSED:
     //BtnMsg(7, "long pressed");
     break;
  case EVNT_SW7_LONG_RELEASED:
     //BtnMsg(7, "long released");
     break;
#endif /* PL_CONFIG_NOF_KEYS */
//---
    default:
      break;
   } /* switch */
}
#endif /* PL_CONFIG_HAS_EVENTS */

#if PL_CONFIG_HAS_MOTOR /* currently only used for robots */
static const KIN1_UID RoboIDs[] = {
  /* 0: L20, V2 */ {{0x00,0x03,0x00,0x00,0x67,0xCD,0xB7,0x21,0x4E,0x45,0x32,0x15,0x30,0x02,0x00,0x13}},
  /* 1: L21, V2 */ {{0x00,0x05,0x00,0x00,0x4E,0x45,0xB7,0x21,0x4E,0x45,0x32,0x15,0x30,0x02,0x00,0x13}},
  /* 2: L4, V1  */ {{0x00,0x0B,0xFF,0xFF,0x4E,0x45,0xFF,0xFF,0x4E,0x45,0x27,0x99,0x10,0x02,0x00,0x24}},
  /* 3: L23, V2 */ {{0x00,0x0A,0x00,0x00,0x67,0xCD,0xB8,0x21,0x4E,0x45,0x32,0x15,0x30,0x02,0x00,0x13}},
  /* 4: L11, V2 */ {{0x00,0x19,0x00,0x00,0x67,0xCD,0xB9,0x11,0x4E,0x45,0x32,0x15,0x30,0x02,0x00,0x13}},
  /* 5: L5, V2 */  {{0x00,0x38,0x00,0x00,0x67,0xCD,0xB5,0x41,0x4E,0x45,0x32,0x15,0x30,0x02,0x00,0x13}},
  /* 6: L3, V1 */  {{0x00,0x33,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x4E,0x45,0x27,0x99,0x10,0x02,0x00,0x0A}},
  /* 7: L1, V1 */  {{0x00,0x19,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x4E,0x45,0x27,0x99,0x10,0x02,0x00,0x25}},
};
#endif

static void APP_AdoptToHardware(void) {
  KIN1_UID id;
  uint8_t res;

  res = KIN1_UIDGet(&id);
  if (res!=ERR_OK) {
    for(;;); /* error */
  }
#if PL_CONFIG_HAS_MOTOR
  if (KIN1_UIDSame(&id, &RoboIDs[0])) { /* L20 */
#if PL_CONFIG_HAS_QUADRATURE
    (void)Q4CRight_SwapPins(TRUE);
#endif
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE); /* invert left motor */
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), TRUE); /* invert left motor */
  } else if (KIN1_UIDSame(&id, &RoboIDs[1])) { /* V2 L21 */
    /* no change needed */
  } else if (KIN1_UIDSame(&id, &RoboIDs[2])) { /* V1 L4 */
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE); /* revert left motor */
#if PL_CONFIG_HAS_QUADRATURE
    (void)Q4CLeft_SwapPins(TRUE);
    (void)Q4CRight_SwapPins(TRUE);
#endif
  } else if (KIN1_UIDSame(&id, &RoboIDs[3])) { /* L23 */
#if PL_CONFIG_HAS_QUADRATURE
    (void)Q4CRight_SwapPins(TRUE);
#endif
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE); /* invert left motor */
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), TRUE); /* invert left motor */
  } else if (KIN1_UIDSame(&id, &RoboIDs[4])) { /* L11 */
#if PL_CONFIG_HAS_QUADRATURE
    (void)Q4CRight_SwapPins(TRUE);
#endif
  } else if (KIN1_UIDSame(&id, &RoboIDs[5])) { /* L5, V2 */
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_RIGHT), TRUE); /* invert right motor */
    (void)Q4CRight_SwapPins(TRUE);
  } else if (KIN1_UIDSame(&id, &RoboIDs[6])) { /* L3, V1 */
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE); /* invert right motor */
#if PL_CONFIG_HAS_QUADRATURE
    (void)Q4CLeft_SwapPins(TRUE);
    (void)Q4CRight_SwapPins(TRUE);
#endif
  } else if (KIN1_UIDSame(&id, &RoboIDs[7])) { /* L1, V1 */
    MOT_Invert(MOT_GetMotorHandle(MOT_MOTOR_LEFT), TRUE); /* invert right motor */
#if PL_CONFIG_HAS_QUADRATURE
    (void)Q4CLeft_SwapPins(TRUE);
    (void)Q4CRight_SwapPins(TRUE);
#endif
  }
#endif
#if PL_CONFIG_HAS_QUADRATURE && PL_CONFIG_BOARD_IS_ROBO_V2
  /* pull-ups for Quadrature Encoder Pins */
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 10, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 10, PORT_PDD_PULL_ENABLE);
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 11, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 11, PORT_PDD_PULL_ENABLE);
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 16, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 16, PORT_PDD_PULL_ENABLE);
  PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 17, PORT_PDD_PULL_UP);
  PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 17, PORT_PDD_PULL_ENABLE);
#endif
}

static void blinkLED(void *p){
	LED1_Neg();
	TRG_SetTrigger(TRG_BLINK,1000/TMR_TICK_MS,blinkLED, NULL);
}

/*
 * FUNCTION
 * attention: 	function KEY_Scan in Keys.c
 */
void KEY_scan(void){
#if PL_CONFIG_HAS_DEBOUNCE
	/* Key FSM */
	KEYDBNC_Process();	// Key - FSM, replaces KEYx_Get()
#else
	/* Key polling */
	if(KEY1_Get()){
		EVNT_SetEvent(EVNT_SW1_PRESSED);
		//BtnMsg(1, "pressed");
	}
#if PL_CONFIG_BOARD_IS_REMOTE
	if(KEY2_Get()){
		EVNT_SetEvent(EVNT_SW2_PRESSED);
	}
	if(KEY3_Get()){
		EVNT_SetEvent(EVNT_SW3_PRESSED);
	}
	if(KEY4_Get()){
		EVNT_SetEvent(EVNT_SW4_PRESSED);
	}
	if(KEY5_Get()){
		EVNT_SetEvent(EVNT_SW5_PRESSED);
	}
	if(KEY6_Get()){
		EVNT_SetEvent(EVNT_SW6_PRESSED);
	}
	if(KEY7_Get()){
		EVNT_SetEvent(EVNT_SW7_PRESSED);
	}
#endif /* PL_CONFIG_BOARD_IS_REMOTE */
#endif /* PL_CONFIG_HAS_DEBOUNCE */

	EVNT_HandleEvent(APP_EventHandler, TRUE); // Event handling
}

/* --------------------------------------------------
 * TASK
 * more features, more stack
 * --------------------------------------------------
 * ----- vTaskDelay()
 * vTaskDelay() specifies a time at which the task wishes to unblock relative to
 * the time at which vTaskDelay() is called.  For example, specifying a block
 * period of 100 ticks will cause the task to unblock 100 ticks after
 * vTaskDelay() is called.  vTaskDelay() does not therefore provide a good method
 * of controlling the frequency of a periodic task as the path taken through the
 * code, as well as other task and interrupt activity, will effect the frequency
 * at which vTaskDelay() gets called and therefore the time at which the task
 * next executes.
 *
 * ----- vTaskDelayUntil()
 * Whereas vTaskDelay () specifies a wake time relative to the time at which the function
 * is called, vTaskDelayUntil () specifies the absolute (exact) time at which it wishes to
 * unblock.
 *
 * ----- vTaskDelay() vs vTaskDelayUntil()
 * Whereas vTaskDelay () specifies a wake time relative to the time at which the function
 * is called, vTaskDelayUntil () specifies the absolute (exact) time at which it wishes to
 * unblock.
 -------------------------------------------------- */
static void appTask(void *pvParameters){
	//(void)pvParameters;	//cast, necessary?
	TickType_t last = xTaskGetTickCount();

	// ========== [ start sequency ] ==========
#if PL_CONFIG_HAS_BUZZER
	BUZ_Beep(300,1000);
#endif /* PL_CONFIG_HAS_BUZZER */
	TRG_SetTrigger(TRG_BLINK,0,blinkLED, NULL);
	//TRG1_AddTrigger(TRG_BLINK,0,blinkLED, NULL);

	// ========== [ loop ] ==========
	for(;;){
		KEY_scan();
		//KEY_scan();
		//EVNT_HandleEvent(APP_EventHandler, TRUE);
		//vTaskDelay(500/portTICK_RATE_MS);
		vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
	}
}

static void taskOne(void *pvParameters){
	BUZ_Beep(300,1000);
	for(;;){
		vTaskDelay(500/portTICK_RATE_MS);
		//CLS1_SendStr("a", CLS1_GetStdio()->stdOut);
	}
}

/* --------------------------------------------------
 * Zork
 * - ACHTUNG
 * 		- Dieser Task laeuft parallel zum Task Shell und verwendet die selbe Rescource!
 * 		- Moeglicher Ansatz: Sonderzeichen vor Befehl, damit Unterschied zwischen den Tasks (nicht implementiert)
 * - strcmp
 * 		- string compare bei einem char nicht noetig
 * --------------------------------------------------*/
static void taskZork(void *pvParameters){
	char stateGame = 0;
	uint8_t ch;
	//char buf [32];
	for(;;){
		vTaskDelay(50/portTICK_RATE_MS);

		/* ========== [ state machien - game ] ========== */
		if(stateGame == 0){			/* send information */

			CLS1_SendStr("press key 'y' to start \n", CLS1_GetStdio()->stdOut);
			stateGame += 1;
		}
		else if(stateGame == 1){	/* read input */
			//if(SW1_GetVal()==0){ 					/* start with button */
			if(CLS1_GetStdio()->keyPressed()>0){ 	/* start with shell, read key */
				//uint8_t ch;							// moved
				CLS1_GetStdio()->stdIn(&ch);
				//if(ch!='\0'){							// not used - optional safety check
				if(ch == 'y'){							// check key
					CLS1_GetStdio()->stdOut(ch);
					stateGame += 1;
				}else{
					ch = 0;
				}
			}
		}
		else if(stateGame == 2){	/* game */
			zork_config();
		 	run_zork_game();
			stateGame = 0;
		}
		else{
			/* error handling */
		}
		/* ========== [ CLS1 ] ========== */
		//CLS1_KeyPressed();		/* returns only, if something in the buffer */
		//CLS1_ReadChar(&a);		/* blocks, waits until button pressed */
		//CLS2_ReadLine();			/* blocks, read line until ('\r','\n') received */

		/* ========== [ graveyard  ] ==========
		 *  here lie my hopeless attempts
		*/
		//uint8_t a[48];
		//uint8_t b = 8;
		//UTIL1_Num8uToStr(buffer_value,sizeof(buffer_value),b);
		//uint8_t a;
		//char buf [32];
		//if(CLS1_KeyPressed()){ // doesn't work !
			//	CLS1_ReadChar(buf);
			//	stateGame += 1;
		//}
		//if(!strcmp(a,"b")){
		//if(strcmp(*(CLS1_GetStdio()),'a') == TRUE){
	}
}

/* --------------------------------------------------
 * APP
 * -------------------------------------------------- */
void APP_Start(void) {
#if PL_CONFIG_BOARD_IS_REMOTE
  /* need pull-up on UART Rx pin (PTC3) */
  //PORT_PDD_SetPinPullSelect(PORTC_BASE_PTR, 3, PORT_PDD_PULL_UP);
  //PORT_PDD_SetPinPullEnable(PORTC_BASE_PTR, 3, PORT_PDD_PULL_ENABLE);
#endif

  PL_Init();
  //RNETA_Init();// in Platform.c, REMOTE_Init() already;
  //RTOS_init(); // in PL_Init already
  //SEM_Init();

  APP_AdoptToHardware();
  __asm volatile("cpsie i"); /* enable interrupts */

  // ========== [ event ] ==========
  EVNT_SetEvent(EVNT_STARTUP);

  EVNT_HandleEvent(APP_EventHandler, TRUE);
  // ========== [ task ] ==========
  // ApplicationMallocFailedHook => increase heap size
  xTaskHandle taskHndl;

  // ----- | appTask | -----
  //<< create task
  BaseType_t res;
  res = xTaskCreate(
	appTask,						/* function */
	"appTask",						/* kernel awareness name */
	//configMINIMAL_STACK_SIZE+120,	/* stack */
	1000/sizeof(StackType_t),
	(void*) NULL, 					/* task parameter */
	tskIDLE_PRIORITY + 2, 				/* priority */
	&taskHndl 						/* handle */
  );//>> create task
  if (res!=pdPASS){ /* error handling */ }

//  // ----- | task 2| -----
//  if(xTaskCreate(
//	  taskOne,
//	  "taskOne",
//	  500/sizeof(StackType_t),
//	  (void*) NULL,
//	  tskIDLE_PRIORITY+2,
//	  &taskHndl
//  )!=pdPASS){
//	  /* error handling */
//  }

  // ----- | task Zork| -----
  //zorkTask(taskHndl);
  //---

  vTaskStartScheduler();		/* starts scheduler, creates IDLE task */
  //--wait in scheduler?


  // ==========  [ loop ] ==========
  for(;;){
	  //EVNT_HandleEvent(APP_EventHandler, TRUE);
	  //vTaskSuspendAll();		/* suspend all tasks */
	  //xTaskResumeAll();		/* resume all tasks */
  }
}

/* --------------------------------------------------
 * ZORK
 * platform: 	Robo V1
   ------------------------------------------------- */
void zorkTask(xTaskHandle taskHndl){
	 if(xTaskCreate(
		  taskZork,
		  "taskZork",
		  8000/sizeof(StackType_t),
		  (void*) NULL,
		  tskIDLE_PRIORITY+1,
		  &taskHndl
	  )!=pdPASS){
		 /*  error handling */
	  }
}
// --------------------------------------------------
// #23 DEBOUNCE
// platform: 	Robo V1
// -------------------------------------------------
void assignment23Queues(void){
	/* solution in file shell.c */
}

/* --------------------------------------------------
 * #19 freeRTOS
 * platform: 	Robo V1
 *
 * RTOS
 * - RTOS_Init() 	called in PL_Init();
// -------------------------------------------------
/* -------------------------------------------------
 * KERNEL CONTROL
 * - FreeRTOS API to control the kernel
 * - kernel states (init, running, suspended)
   ------------------------------------------------- */
//  vTaskStartScheduler();		/* starts scheduler, creates IDLE task */
//  vTaskEndScheduler();		/* stops scheduler, release kernel rescources, task resources (queues, semaphores) are not freed */
//  vTaskSuspendAll();			/* kernel suspending, from active to supsended state, intrrupts remain enabled, prevents context switch */
//  xTaskResumeAll();			/* kernel resuming, from suspended to active state */
//  taskYIELD();				/* passing control to one of the ready tasks, request context switch with software interrupt */
//  taskENTER_CRITICAL();		/* enter critical section, preemptive context switches cannot occur in CS */
//  taskEXIT_CRITICAL();		/* exit critical section */
//  taskDISABLE_INTERRUPTS();	/* macro */
//  taskENABLE_INTERRUPTS(); 	/* macro */
void assignment19frtos_task(void){
	  EVNT_SetEvent(EVNT_STARTUP);

	  // ========== [ task ] ==========
	  // ApplicationMallocFailedHook => increase heap size
	  xTaskHandle taskHndl;

	  //<< create task
	  BaseType_t res;
	  res = xTaskCreate(
		appTask,			/* function */
		"appTask",		/* kernel awareness name */
		//configMINIMAL_STACK_SIZE+120,	/* stack */
		500/sizeof(StackType_t),
		(void*) NULL, 		/* task parameter */
		tskIDLE_PRIORITY, 	/* priority */
		&taskHndl 			/* handle */
	  );//>> create task
	  if (res!=pdPASS){ /* error handling */ }

	// === [ task short version ] ===
	//  if (xTaskCreate(firstTask, "FirstTask", 500/sizeof(StackType_t), NULL, tskIDLE_PRIORITY+2, NULL) != pdPASS) {
	//     for(;;){} /* error case only, stay here! */
	//   }

	  vTaskStartScheduler();		/* starts scheduler, creates IDLE task */

	  // ==========  [ loop ] ==========
	  for(;;){
		  EVNT_HandleEvent(APP_EventHandler, TRUE);

	  }
}

// --------------------------------------------------
// #18 DEBOUNCE
// platform: 	Robo V1
// -------------------------------------------------
void assignment18debounce(void){
	EVNT_SetEvent(EVNT_STARTUP);
	for(;;){
		KEYDBNC_Process();	// Key - FSM
		EVNT_HandleEvent(APP_EventHandler, TRUE); // Event
	}
}

// --------------------------------------------------
// #17 TRIGGER
// platform: 	Robo V1
// -------------------------------------------------
void assignment17trigger(void){
	EVNT_SetEvent(EVNT_STARTUP);

	//BUZ_Init();			// not used !
	//BUZ_PlayTune(BUZ_TUNE_WELCOME);
	BUZ_Beep(300,1000);
	//TRG1_Init();		// not used !
	TRG_SetTrigger(TRG_BLINK,0,blinkLED, NULL); /* 	TRG_Blink defined in Trigger.h
	 	 	 	 	 	 	 	 	 	 	 	 	 blinkLED function in application */
	for(;;){
	  EVNT_HandleEvent(APP_EventHandler, TRUE);
	}
}

// --------------------------------------------------
// #16 CONSOLE - asynchrone serial
// platform: 	Robo V2
// autor: 		arbnor
// -------------------------------------------------
void assignment16consoleRoboV2(void){
	  char test[12]="Shell Test\n";
	  CLS1_SendStr(test , CLS1_GetStdio()->stdOut);
}

// --------------------------------------------------
// #16 CONSOLE - serial RTT
// platform: 	Robo V1
// -------------------------------------------------
void assignment16console(void){
	  //--- Events
	  EVNT_SetEvent(EVNT_STARTUP); // nescessary ?
	  //--- Init
	  RTT1_Init();		// real time terminal init

	 for(;;) {
		  EVNT_HandleEvent(APP_EventHandler, TRUE);

		  if(SW1_GetVal()==0){	// key pressed
			  EVNT_SetEvent(EVNT_STARTUP);

			  //--- buffer size and start
			  uint8_t buffer_write[48];
			  buffer_write[0] = '\0';	// start position, writes otherwise at the end of the buffer

			  //--- uint8 -> string
			  uint8_t buffer_value[48];
			  uint8_t b = 8;
			  UTIL1_Num8uToStr(buffer_value,sizeof(buffer_value),b);

			  //--- string
			  char str[]=" value ";

			  //--- write in buffer
			  UTIL1_strcat(buffer_write, sizeof(buffer_write), str);
			  UTIL1_strcat(buffer_write, sizeof(buffer_write), buffer_value);

			  //--- send
			  RTT1_WriteString(0, buffer_write);	// send buffer
			  RTT1_WriteString(0,"\n");				// newline
		  }
	 }
}

// --------------------------------------------------
// #5 KEYS
// platform: 	Robo V1
// -------------------------------------------------
void assignment15keys(void){
	EVNT_SetEvent(EVNT_STARTUP);
	RTT1_Init();
	for(;;){
		EVNT_HandleEvent(APP_EventHandler, TRUE);
		//--------------------
		// LED
		//--------------------
		#if PL_CONFIG_NOF_LEDS>=1
		//	  LED1_On();
		#endif
		#if PL_CONFIG_NOF_LEDS>=2
		//	  LED2_On();
		#endif
		#if PL_CONFIG_NOF_LEDS>=3
		//	  LED3_On();
		#endif
		//WAIT1_Waitms(500);
		//LED1_Off();
		//LED2_Off();
		//LED3_Off();
		//WAIT1_Waitms(500);
		//EVNT_SetEvent(EVNT_LED_HEARTBEAT);
		//--------------------
		// Keys
		//--------------------
		LED1_Off();
		if(SW1_GetVal()==0){	// key pressed
			//LED1_On();
			EVNT_SetEvent(EVNT_STARTUP);
		}
	}
}

