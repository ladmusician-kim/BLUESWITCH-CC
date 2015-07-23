#include "bcomdef.h"

#include "BSGATTprofile.h"
#include "parsingData.h"
#include "BSBLEPeripheral.h"

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void parsingData_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void writeDataInBoard(uint8 recvData);
//static uint8* getReadStatePacket();
//static uint8* getWriteStatePacket();
static uint8* getAbsenceStatePacket();
static uint8 getCurrentState();

static uint32 getEndMillis(uint8 endHours, uint8 endMinutes);
static uint32 getStartMillis(uint8 startHours, uint8 startMinutes);
static uint32 getNextDayMillis(uint32 endMillis);

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 parsingData_TaskID;   // Task ID for internal task/event processing

static uint8 absenceDataByte;
static uint32 lStartMillis;
static uint32 lEndMillis;
static uint32 lNextDayMillis;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

uint8 getParsingDataTaskId() { return parsingData_TaskID; }

void parsingData_Init( uint8 task_id )
{
  parsingData_TaskID = task_id;
}

bool parsingDataPacket( uint8* recvPacket, uint8 packetLength )
{
  uint8 dataLength = 0;
  
  if(recvPacket != NULL && packetLength != 0) 
  {
    if(packetLength == PACKET_LENGTH_NORMAL || packetLength == PACKET_LENGTH_ABSENCE) 
    {
      if(recvPacket[0] == STX) 
      {
        dataLength = recvPacket[1];
        
        if((dataLength + HEADER_LENGTH) == packetLength) 
        {
          //if(recvPacket[dataLength+2] == // CHECKSUM 확인 부분
          
          if(recvPacket[3 + dataLength] == ETX) 
          {
            if(recvPacket[2] == TYPE_SWITCH || recvPacket[2] == TYPE_WIDGET) 
            {
              uint8 recvData = recvPacket[3];
              
              if(check_bit(recvData, 7)) { // 보드에 데이터를 쓸 경우
                writeDataInBoard(recvData);
                
                uint8* responsePacket = getWriteStatePacket();
                BSProfile_SetParameter(BSPROFILE_CHAR1, PACKET_LENGTH_RESPONSE, responsePacket);
                notifyCharateristicChanged(BSPROFILE_CHAR1);
                osal_mem_free(responsePacket);
              } 
              else { // 보드의 데이터를 읽어 갈 경우
                uint8* responsePacket = getReadStatePacket();
                BSProfile_SetParameter(BSPROFILE_CHAR1, PACKET_LENGTH_RESPONSE, responsePacket);
                notifyCharateristicChanged(BSPROFILE_CHAR1);
                osal_mem_free(responsePacket);
              }
              return true;
            }
            else if(recvPacket[2] == TYPE_ALARM) 
            {
              uint8 currentState = getCurrentState();
              uint8 recvData = recvPacket[3];
              uint8 controlData = (currentState & recvData);
              writeDataInBoard(controlData);
              
              if(check_bit(recvData, 3)) { // 악마의 알람
                isSetDevilAlarm(true);
              }
              
              else {
                uint8* responsePacket = getWriteStatePacket();
                BSProfile_SetParameter(BSPROFILE_CHAR1, PACKET_LENGTH_RESPONSE, responsePacket);
                notifyCharateristicChanged(BSPROFILE_CHAR1);
                osal_mem_free(responsePacket);
              }
              
              return true;
            }
            else if(recvPacket[2] == TYPE_TIMER) 
            {
              uint8 currentState = getCurrentState();
              uint8 recvData = recvPacket[3];
              uint8 controlData = (currentState & recvData);
              writeDataInBoard(controlData);

              uint8* responsePacket = getWriteStatePacket();
              BSProfile_SetParameter(BSPROFILE_CHAR1, PACKET_LENGTH_RESPONSE, responsePacket);
              notifyCharateristicChanged(BSPROFILE_CHAR1);
              osal_mem_free(responsePacket);
              
              return true;
            }
            else if(recvPacket[2] == TYPE_ABSENCE) 
            {
              absenceDataByte = recvPacket[3];
              
              uint8 startHours = recvPacket[4];
              uint8 startMinutes = recvPacket[5];
              uint8 endHours = recvPacket[6];
              uint8 endMinutes = recvPacket[7];
              
              lStartMillis  = getStartMillis(startHours, startMinutes);
              lEndMillis  = getEndMillis(endHours, endMinutes);
              lNextDayMillis = getNextDayMillis(lEndMillis);
              
              if((absenceDataByte & 0x30) == ABSENCE_MODE_ON) { // 등록
                osal_set_event( parsingData_TaskID, EVT_ABSENCE_REGISTER );
                
                uint8* responsePacket = getWriteStatePacket();
                BSProfile_SetParameter(BSPROFILE_CHAR1, PACKET_LENGTH_RESPONSE, responsePacket);
                notifyCharateristicChanged(BSPROFILE_CHAR1);
                SetAbsenceMode(true);
                osal_mem_free(responsePacket);
                
                return true;
              }
              else if((absenceDataByte & 0x30) == ABSENCE_MODE_OFF) { // 해체
                osal_set_event( parsingData_TaskID, EVT_ABSENCE_UNREGISTER );
                
                uint8* responsePacket = getReadStatePacket();
                BSProfile_SetParameter(BSPROFILE_CHAR1, PACKET_LENGTH_RESPONSE, responsePacket);
                notifyCharateristicChanged(BSPROFILE_CHAR1);
                SetAbsenceMode(false);
                osal_mem_free(responsePacket);
                
                return true;
              }
              else if((absenceDataByte & 0x30) == ABSENCE_MODE_CHECK) { // 부재 모드 체크
                uint8* responsePacket = getAbsenceStatePacket();
                BSProfile_SetParameter(BSPROFILE_CHAR1, PACKET_LENGTH_RESPONSE, responsePacket);
                notifyCharateristicChanged(BSPROFILE_CHAR1);
                osal_mem_free(responsePacket);
                
                return true;
              }
              else if((absenceDataByte & 0x30) == ABSENCE_MODE_SYNC) { // 동기화
                
                return true;
              }
              
              return false;
            }
            else if(recvPacket[2] == TYPE_DIMMING) 
            {
              
              return true;
            }
          }
        }
      }
    }
  }
  return false;
}

static uint32 getNextDayMillis(uint32 endMillis) {
  uint32 nextDayMillis = 0;
  
  nextDayMillis = MILLIS_DAY - endMillis;
  
  return nextDayMillis;
}

static uint32 getStartMillis(uint8 startHours, uint8 startMinutes) {
  uint32 startMillis = 0;
  
  if(startHours == 0) startMillis = startMinutes * MILLIS_MINUTE;
  else {
    uint32 startHH = startHours * MILLIS_HOUR;
    uint32 startMM = startMinutes * MILLIS_MINUTE;
    startMillis = startHH + startMM;
  }
  return startMillis;
} 

static uint32 getEndMillis(uint8 endHours, uint8 endMinutes) {
  uint32 endMillis = 0;
  
  if(endHours == 0) endMillis = endMinutes * MILLIS_MINUTE;
  else {
    uint32 endHH = endHours * MILLIS_HOUR;
    uint32 endMM = endMinutes * MILLIS_MINUTE;
    endMillis = endHH + endMM;
  }
  return endMillis;
} 

static void writeDataInBoard(uint8 recvData) {
  if(check_bit(recvData, LIGHT_ONE)) {
    setLightOneOn();
    setLedOneOff();
  }
  else {
    setLightOneOff();
    setLedOneOn();
  }
  if(check_bit(recvData, LIGHT_TWO)) {
    setLightTwoOn();
    setLedTwoOff();
  }
  else {
    setLightTwoOff();
    setLedTwoOn();
  }
  if(check_bit(recvData, LIGHT_THREE)) {
    setLightThreeOn();
    setLedThreeOff();
  }
  else {
    setLightThreeOff();
    setLedThreeOn();
  }
}

static uint8 getCurrentState() {
  uint8 currentState = 0x00;
  
  if(isLightOneOn()) set_bit(currentState, LIGHT_ONE);
  if(isLightTwoOn()) set_bit(currentState, LIGHT_TWO);
  if(isLightThreeOn())  set_bit(currentState, LIGHT_THREE);
  
  return currentState;
}

static uint8* getAbsenceStatePacket() {
  uint8* responsePacket = osal_mem_alloc(PACKET_LENGTH_RESPONSE);
  
  responsePacket[0] = STX;
  responsePacket[1] = 0x00;
  set_bit(responsePacket[1], 7);
  if(GetAbsenceMode()) set_bit(responsePacket[1], 0);
  responsePacket[2] = ETX;
  
  return responsePacket;
}

uint8* getReadStatePacket() {
  uint8* responsePacket = osal_mem_alloc(PACKET_LENGTH_RESPONSE);
  
  responsePacket[0] = STX;
  responsePacket[1] = 0x00;
  if(isLightOneOn()) set_bit(responsePacket[1], LIGHT_ONE);
  if(isLightTwoOn()) set_bit(responsePacket[1], LIGHT_TWO);
  if(isLightThreeOn())  set_bit(responsePacket[1], LIGHT_THREE);
  responsePacket[2] = ETX;
  
  return responsePacket;
}

uint8* getWriteStatePacket() {
  uint8* responsePacket = osal_mem_alloc(PACKET_LENGTH_RESPONSE);
  
  responsePacket[0] = STX;
  responsePacket[1] = 0x00;
  set_bit(responsePacket[1], 7);
  if(isLightOneOn()) set_bit(responsePacket[1], LIGHT_ONE);
  if(isLightTwoOn()) set_bit(responsePacket[1], LIGHT_TWO);
  if(isLightThreeOn()) set_bit(responsePacket[1], LIGHT_THREE);
  responsePacket[1] = ( responsePacket[1] | ABSENCE_MODE_ON );
  responsePacket[2] = ETX;
  
  return responsePacket;
}

uint16 parsingData_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;
    
    if ( (pMsg = osal_msg_receive( parsingData_TaskID )) != NULL )
    {
      parsingData_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
      
      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }
    
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  if( events & EVT_ABSENCE_REGISTER ) 
  {
    osal_stop_timerEx( parsingData_TaskID, EVT_ABSENCE_LIGHT_ON );
    osal_stop_timerEx( parsingData_TaskID, EVT_ABSENCE_LIGHT_OFF );
    
    osal_start_timerEx( parsingData_TaskID, EVT_ABSENCE_LIGHT_ON, lStartMillis);
    
    return ( events ^ EVT_ABSENCE_REGISTER );
  }
  
  if( events & EVT_ABSENCE_UNREGISTER ) 
  {
    osal_stop_timerEx( parsingData_TaskID, EVT_ABSENCE_LIGHT_ON );
    osal_stop_timerEx( parsingData_TaskID, EVT_ABSENCE_LIGHT_OFF );
    
    return ( events ^ EVT_ABSENCE_UNREGISTER );
  }
  
  if( events & EVT_ABSENCE_LIGHT_ON )
  {
    writeDataInBoard(absenceDataByte);
    
    osal_stop_timerEx( parsingData_TaskID, EVT_ABSENCE_LIGHT_ON );
    osal_start_timerEx( parsingData_TaskID, EVT_ABSENCE_LIGHT_OFF, lEndMillis );
      
    return ( events ^ EVT_ABSENCE_LIGHT_ON );
  }
  
  if( events & EVT_ABSENCE_LIGHT_OFF )
  {
    writeDataInBoard((!absenceDataByte));
    
    osal_stop_timerEx( parsingData_TaskID, EVT_ABSENCE_LIGHT_OFF );
    osal_start_timerEx( parsingData_TaskID, EVT_ABSENCE_LIGHT_ON, lNextDayMillis );
    
    return ( events ^ EVT_ABSENCE_LIGHT_OFF );
  }
  
  return 0;
}

static void parsingData_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  default:
    // do nothing
    break;
  }
}
