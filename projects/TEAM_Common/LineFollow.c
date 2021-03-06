/**
 * \file
 * \brief Implements line following of the robot
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * This is the implementation of the line following.
 */

#include "Platform.h"
#if PL_CONFIG_HAS_LINE_FOLLOW
#include "LineFollow.h"
#include "FRTOS1.h"
#include "CLS1.h"
#include "Shell.h"
#include "Motor.h"
#include "Reflectance.h"
#if PL_CONFIG_HAS_TURN
  #include "Turn.h"
#endif
#include "WAIT1.h"
#include "Pid.h"
#include "Drive.h"
#include "Shell.h"
#if PL_CONFIG_HAS_BUZZER
  #include "Buzzer.h"
#endif
#if PL_CONFIG_HAS_DRIVE
  #include "Drive.h"
#endif

#if PL_CONFIG_HAS_RADIO /*! \todo done*/
#include "RNet_App.h"
#endif

typedef enum {
  STATE_IDLE,              /* idle, not doing anything */
  STATE_FOLLOW_SEGMENT,    /* line following segment, going forward */
  STATE_TURN,              /* reached an intersection, turning around */
  STATE_SEARCH,				/* search line */
  STATE_FINISHED,          /* reached finish area */
  STATE_STOP               /* stop the engines */
} StateType;

/* added for competition */
typedef enum{
	STATE_TURN_LEFT,
	STATE_TURN_RIGHT
}StateTurnType;

/* task notification bits */
#define LF_START_FOLLOWING (1<<0)  /* start line following */
#define LF_STOP_FOLLOWING  (1<<1)  /* stop line following */

static volatile StateType LF_currState = STATE_IDLE;
static volatile StateTurnType LF_currStateTurn = STATE_TURN_LEFT;	/* added for competition */
static volatile uint16_t cntTurnAngle = 0;							/* added for competition */

static xTaskHandle LFTaskHandle;

void LF_StartFollowing(void) {
  (void)xTaskNotify(LFTaskHandle, LF_START_FOLLOWING, eSetBits);
}

void LF_StopFollowing(void) {
  (void)xTaskNotify(LFTaskHandle, LF_STOP_FOLLOWING, eSetBits);
}

void LF_StartStopFollowing(void) {
  if (LF_IsFollowing()) {
    (void)xTaskNotify(LFTaskHandle, LF_STOP_FOLLOWING, eSetBits);
  } else {
    (void)xTaskNotify(LFTaskHandle, LF_START_FOLLOWING, eSetBits);
  }
}

/* forward declaration */
static void StateMachine(void);

/*!
 * \brief follows a line segment.
 * \return Returns TRUE if still on line segment
 */
static bool FollowSegment(void) {
  uint16_t currLine;
  REF_LineKind currLineKind;

  currLine = REF_GetLineValue();
  currLineKind = REF_GetLineKind();
  if ((currLineKind==REF_LINE_STRAIGHT)||
		  (currLineKind==REF_LINE_LEFT)||
		  (currLineKind==REF_LINE_RIGHT)) {
    PID_Line(currLine, REF_MIDDLE_LINE_VALUE); /* move along the line */
    return TRUE;
  } else {
    return FALSE; /* intersection/change of direction or not on line any more */
  }
}

static void StateMachine(void) {
  REF_LineKind lineKind;

  switch (LF_currState) {
    case STATE_IDLE:
      break;

    case STATE_FOLLOW_SEGMENT:
      if (!FollowSegment()) {
        //SHELL_SendString((unsigned char*)"No line, stopped!\r\n");
        //LF_currState = STATE_STOP; /* stop if we do not have a line any more */
        //LF_currState = STATE_TURN;
    	  LF_currState = STATE_SEARCH;
      }
      break;

    case STATE_TURN:
       lineKind = REF_GetLineKind();
       if (lineKind==REF_LINE_FULL) {
    	 //LF_StartStopFollowing();
         LF_currState = STATE_FINISHED;
       } if (lineKind==REF_LINE_NONE) {
     	  /* turn left or right and set next turn*/
     	  if(LF_currStateTurn==STATE_TURN_LEFT){
     		  //TURN_Turn(TURN_STOP, NULL);
     		  //TURN_TurnAngle(130, NULL);
     		  //TURN_Turn(TURN_LEFT180, NULL);
     		  LF_currStateTurn = STATE_TURN_RIGHT;
     	  } else {
     		  //TURN_Turn(TURN_STOP, NULL);
     		  //TURN_TurnAngle(-130, NULL);
     		  //TURN_Turn(TURN_RIGHT180, NULL);
     		  LF_currStateTurn = STATE_TURN_LEFT;
     	  }
         DRV_SetMode(DRV_MODE_NONE); /* disable position mode */
         LF_currState = STATE_FOLLOW_SEGMENT;
       } else {
         LF_currState = STATE_STOP;
       }
       break;

    case STATE_SEARCH:
    	lineKind = REF_GetLineKind();
    	if (lineKind==REF_LINE_NONE) {
    		//int32_t turnValue;
		  /* turn left or right and set next turn*/
		  if(LF_currStateTurn==STATE_TURN_LEFT){
			  TURN_TurnAngle(-5, NULL);
			  cntTurnAngle = cntTurnAngle + 1;
			  //LF_currStateTurn = STATE_TURN_RIGHT;
		  } else {
			  TURN_TurnAngle(5, NULL);
			  cntTurnAngle = cntTurnAngle + 1;
			  //LF_currStateTurn = STATE_TURN_LEFT;
		  }
		 DRV_SetMode(DRV_MODE_NONE); /* disable position mode */
    	} else{
    		LF_currState = STATE_FOLLOW_SEGMENT;
    		if(cntTurnAngle >= 5){
    			if(LF_currStateTurn==STATE_TURN_LEFT){
    				LF_currStateTurn = STATE_TURN_RIGHT;
				}else
					LF_currStateTurn = STATE_TURN_LEFT;
    		}else{
    			/* change nothing */
    		}
    		cntTurnAngle = 0;
    	}
    	break;

    case STATE_FINISHED:
      SHELL_SendString("Finished!\r\n");
      LF_currState = STATE_STOP;
      break;

    case STATE_STOP:
#if PL_CONFIG_HAS_RADIO
      RNETA_SendSignal('C'); /*! \todo */
#endif
      SHELL_SendString("Stopped!\r\n");
      TURN_Turn(TURN_STOP, NULL);
      LF_currState = STATE_IDLE;
      break;
  } /* switch */
}

bool LF_IsFollowing(void) {
  return LF_currState!=STATE_IDLE;
}

static void LineTask (void *pvParameters) {
  uint32_t notifcationValue;

  (void)pvParameters; /* not used */
  for(;;) {
    (void)xTaskNotifyWait(0UL, LF_START_FOLLOWING|LF_STOP_FOLLOWING, &notifcationValue, 0); /* check flags */
    if (notifcationValue&LF_START_FOLLOWING) {
#if PL_CONFIG_HAS_RADIO
      RNETA_SendSignal('B'); /*! \todo */
#endif
      DRV_SetMode(DRV_MODE_NONE); /* disable any drive mode */
      PID_Start();
      LF_currState = STATE_FOLLOW_SEGMENT;
    }
    if (notifcationValue&LF_STOP_FOLLOWING) {
      LF_currState = STATE_STOP;
    }
    StateMachine();
    FRTOS1_vTaskDelay(5/portTICK_PERIOD_MS);
  }
}

static void LF_PrintHelp(const CLS1_StdIOType *io) {
  CLS1_SendHelpStr((unsigned char*)"line", (unsigned char*)"Group of line following commands\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  help|status", (unsigned char*)"Shows line help or status\r\n", io->stdOut);
  CLS1_SendHelpStr((unsigned char*)"  start|stop", (unsigned char*)"Starts or stops line following\r\n", io->stdOut);
}

static void LF_PrintStatus(const CLS1_StdIOType *io) {
  CLS1_SendStatusStr((unsigned char*)"line follow", (unsigned char*)"\r\n", io->stdOut);
  switch (LF_currState) {
    case STATE_IDLE: 
      CLS1_SendStatusStr((unsigned char*)"  state", (unsigned char*)"IDLE\r\n", io->stdOut);
      break;
    case STATE_FOLLOW_SEGMENT: 
      CLS1_SendStatusStr((unsigned char*)"  state", (unsigned char*)"FOLLOW_SEGMENT\r\n", io->stdOut);
      break;
    case STATE_STOP: 
      CLS1_SendStatusStr((unsigned char*)"  state", (unsigned char*)"STOP\r\n", io->stdOut);
      break;
    case STATE_TURN: 
      CLS1_SendStatusStr((unsigned char*)"  state", (unsigned char*)"TURN\r\n", io->stdOut);
      break;
    case STATE_FINISHED: 
      CLS1_SendStatusStr((unsigned char*)"  state", (unsigned char*)"FINISHED\r\n", io->stdOut);
      break;
    default: 
      CLS1_SendStatusStr((unsigned char*)"  state", (unsigned char*)"UNKNOWN\r\n", io->stdOut);
      break;
  } /* switch */
}

uint8_t LF_ParseCommand(const unsigned char *cmd, bool *handled, const CLS1_StdIOType *io) {
  uint8_t res = ERR_OK;

  if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_HELP)==0 || UTIL1_strcmp((char*)cmd, (char*)"line help")==0) {
    LF_PrintHelp(io);
    *handled = TRUE;
  } else if (UTIL1_strcmp((char*)cmd, (char*)CLS1_CMD_STATUS)==0 || UTIL1_strcmp((char*)cmd, (char*)"line status")==0) {
    LF_PrintStatus(io);
    *handled = TRUE;
  } else if (UTIL1_strcmp((char*)cmd, (char*)"line start")==0) {
    LF_StartFollowing();
    *handled = TRUE;
  } else if (UTIL1_strcmp((char*)cmd, (char*)"line stop")==0) {
    LF_StopFollowing();
    *handled = TRUE;
  }
  return res;
}

void LF_Deinit(void) {
  /* nothing needed */
}

void LF_Init(void) {
  LF_currState = STATE_IDLE;
  if (xTaskCreate(LineTask, "Line", 800/sizeof(StackType_t), NULL, tskIDLE_PRIORITY, &LFTaskHandle) != pdPASS) {
    for(;;){} /* error */
  }
}
#endif /* PL_CONFIG_HAS_LINE_FOLLOW */
