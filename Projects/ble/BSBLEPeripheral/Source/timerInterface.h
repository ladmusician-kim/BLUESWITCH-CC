#include "OSAL.h"

/*******************************************************************************
 * TYPEDEF
 */

/*******************************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define BS_TIMER_ID              HAL_TIMER_0
#define BS_OP_MODE               HAL_TIMER_MODE_CTC
#define BS_CHANNEL               HAL_TIMER_CHANNEL_SINGLE
#define BS_CHANNEL_MODE          HAL_TIMER_CH_MODE_OUTPUT_COMPARE
#define BS_INT_ENABLE            TRUE

/*********************************************************************
 * FUNCTIONS
 */
extern uint8 getTimerInterfaceTaskId();

/*
 * Task Initialization for the BLE Application
 */
extern void timerInterface_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 timerInterface_ProcessEvent( uint8 task_id, uint16 events );
