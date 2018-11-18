/**
 * \file
 * \brief Main application interface
 * \author Erich Styger, erich.styger@hslu.ch
 *
 * This provides the main application entry point.
 */

#ifndef SOURCES_FS2016_COMMON_APPLICATION_H_
#define SOURCES_FS2016_COMMON_APPLICATION_H_

#include "Platform.h"

#if PL_CONFIG_HAS_EVENTS
#include "Event.h"

void APP_EventHandler(EVNT_Handle event);
void KEY_scan(void);
#endif

void APP_Start(void);
void assignment15keys(void);
void assignment16console(void);
void assignment16consoleRoboV2 (void);
void assignment17trigger(void);
void assignment18debounce(void);
void assignment19frtos_task(void);


#endif /* SOURCES_FS2016_COMMON_APPLICATION_H_ */
