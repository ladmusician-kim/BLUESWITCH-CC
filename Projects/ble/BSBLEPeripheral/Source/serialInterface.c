#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "BSGATTprofile.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "hal_uart.h"
#include "serialInterface.h"
#include "BSBLEPeripheral.h"

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg );

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 SEND_TO_ARDUINO_EVT_PERIOD = 20;

static uint8 serialInterface_TaskID;   // Task ID for internal task/event processing

static bridgeSerialParseState_t pktState = BRIDGE_SERIAL_STATE_LEN;
static uint16 pktLen = 0;

static SendingToArduinoData sendingToArudinoBuffer[BUFF_SIZE];
static uint8 firstQIndex = 0;
static uint8 lastQIndex = 0;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

uint8 getSerialInterfaceTaskId() { return serialInterface_TaskID; }

void SerialInterface_Init( uint8 task_id )
{
  serialInterface_TaskID = task_id;

  NPI_InitTransport(cSerialPacketParser);
  
  //osal_start_timerEx( serialInterface_TaskID, SEND_TO_ARDUINO_EVT, SEND_TO_ARDUINO_EVT_PERIOD );
}

static uint16 numBytes;
static uint8* pktData;

void cSerialPacketParser( uint8 port, uint8 events )
{
  (void) port;
  (void) events;

  uint8 done = FALSE;
  //uint8 numBytes = 0; 
  
  // Serial 데이터 리시브 테스트 부 
  /*
  numBytes = NPI_RxBufLen();
  if(numBytes != 0) {
    //uint8* pktData;
    pktData = (uint8*) osal_mem_alloc(numBytes);
    
    (void) NPI_ReadTransport(pktData, numBytes);
    
    BSProfile_SetParameter(BSPROFILE_CHAR1, numBytes, pktData);
    
    notifyCharateristicChanged(BSPROFILE_CHAR1);
    
    osal_mem_free(pktData);
  }
  */
  
  // Serial 데이터 리시브 구현 부
    
  numBytes = NPI_RxBufLen();

  while((numBytes > 0) && (!done)) {
    switch(pktState) {
    case BRIDGE_SERIAL_STATE_LEN:
      (void) NPI_ReadTransport((uint8*) &pktLen, 1);
      numBytes = numBytes - 1;
      pktState = BRIDGE_SERIAL_STATE_DATA;
      
    case BRIDGE_SERIAL_STATE_DATA:
      if(numBytes >= pktLen) {
        //uint8* pktData;
        pktData = (uint8*) osal_mem_alloc(pktLen);
          
        (void) NPI_ReadTransport(pktData, pktLen);
          
        BSProfile_SetParameter(BSPROFILE_CHAR1, pktLen, pktData);
        notifyCharateristicChanged(BSPROFILE_CHAR1);
          
        osal_mem_free(pktData);
       
        numBytes = numBytes - pktLen;
        pktState = BRIDGE_SERIAL_STATE_LEN;
      } else {
        done = TRUE;
      }
      break;
    }
  }
    
}

void EnqueueSendingToArduinoBuffer(SendingToArduinoData* data) {
  lastQIndex = (lastQIndex + 1) % BUFF_SIZE;
    
  if(firstQIndex == lastQIndex) {
    firstQIndex = (firstQIndex + 1) % BUFF_SIZE;
  }
  
  sendingToArudinoBuffer[lastQIndex] = *data;
  
  osal_mem_free(data->data);
  osal_mem_free(data);
}

SendingToArduinoData* DequeueSendingToArduinoBuffer( void ) {
  if(isQEmpty()) return NULL;
 
  firstQIndex = (firstQIndex + 1) % BUFF_SIZE;

  return (&(sendingToArudinoBuffer[firstQIndex]));
  
}

static uint16 sendBytes;
void SendToArduinoBuffer( void ) {
  SendingToArduinoData* temp = DequeueSendingToArduinoBuffer();
  
  if(temp != NULL) {
    //(void) NPI_WriteTransport(temp->data, temp->len);
    sendBytes = NPI_WriteTransport(temp->data, temp->len);
  }
  return;
}

uint16 SerialInterface_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;
    
    if ( (pMsg = osal_msg_receive( serialInterface_TaskID )) != NULL )
    {
      SerialInterface_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      
      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  if( events & SEND_TO_ARDUINO_EVT )
  {
    SendToArduinoBuffer();
    
    return (events ^ SEND_TO_ARDUINO_EVT);
  }
  
  /*
  if ( events & SEND_TO_ARDUINO_EVT )
  {
    // 현재 상태 확인
    if ( SEND_TO_ARDUINO_EVT_PERIOD )
    {
      osal_start_timerEx( serialInterface_TaskID, SEND_TO_ARDUINO_EVT, SEND_TO_ARDUINO_EVT_PERIOD );
    }
    
    SendToArduinoBuffer();
      
    return ( events ^ SEND_TO_ARDUINO_EVT );
  }
  */
  
  return 0;
}

static void SerialInterface_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  default:
    // do nothing
    break;
  }
}
