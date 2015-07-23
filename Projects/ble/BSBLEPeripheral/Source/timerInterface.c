#include "bcomdef.h"

#include "timerInterface.h"

#include "ioCC2540.h"
#include "OnBoard.h"

#include "hal_timer.h"

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void timerInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void bsHalTimerCB(uint8 timerId, uint8 channel, uint8 channelMode);
static void checkTimerStatus(uint8 status);

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 timerInterface_TaskID;   // Task ID for internal task/event processing

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

uint8 getTimerInterfaceTaskId() { return timerInterface_TaskID; }
 
static uint16 _status;

void timerInterface_Init( uint8 task_id )
{
  timerInterface_TaskID = task_id;
  /*
  (void) HalTimerInit();

  uint8 status = HalTimerConfig ( BS_TIMER_ID,
                                  BS_OP_MODE,
                                  BS_CHANNEL,
                                  BS_CHANNEL_MODE,
                                  BS_INT_ENABLE,
                                  bsHalTimerCB);
  _status = status;
  checkTimerStatus(status);

  HalTimerStart(BS_TIMER_ID, 128);
  */
}

static void checkTimerStatus(uint8 status) 
{
  _status = status;
  switch(status) 
  {
  case HAL_TIMER_OK:
    
    break;
  case HAL_TIMER_NOT_OK:
    
    break;
  case HAL_TIMER_PARAMS_ERROR:
    
    break;
  case HAL_TIMER_NOT_CONFIGURED:
    
    break;
  case HAL_TIMER_INVALID_ID:
    
    break;
  case HAL_TIMER_INVALID_CH_MODE:

    break;
  }
  return;
}

static uint16 count = 0;
static void bsHalTimerCB(uint8 timerId, uint8 channel, uint8 channelMode)
{
  if(count == 500) HalTimerStop(timerId);
  if(timerId == BS_TIMER_ID) {
    count++;
  }
  return;
}

uint16 timerInterface_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;
    
    if ( (pMsg = osal_msg_receive( timerInterface_TaskID )) != NULL )
    {
      timerInterface_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      
      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  return 0;
}

static void timerInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  default:
    // do nothing
    break;
  }
}
