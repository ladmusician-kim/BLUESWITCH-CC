#include "hal_uart.h"
#include "OSAL.h"
#include "npi.h"

/*******************************************************************************
 * TYPEDEF
 */

typedef struct _SendingToArduinoData {
  uint8* data;
  uint8 len;
} SendingToArduinoData;

typedef enum {
  BRIDGE_SERIAL_STATE_LEN,
  BRIDGE_SERIAL_STATE_DATA,
} bridgeSerialParseState_t;


/*******************************************************************************
 * MACROS
 */

#define SEND_TO_ARDUINO_EVT                       0x0001

#define isQEmpty()                        ( firstQIndex == lastQIndex )
#define BUFF_SIZE    10

/*********************************************************************
 * FUNCTIONS
 */

extern uint8 getSerialInterfaceTaskId();
void cSerialPacketParser( uint8 port, uint8 events );

extern void EnqueueSendingToArduinoBuffer(SendingToArduinoData* data);
SendingToArduinoData* DequeueSendingToArduinoBuffer();

/*
 * Task Initialization for the BLE Application
 */
extern void SerialInterface_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 SerialInterface_ProcessEvent( uint8 task_id, uint16 events );
