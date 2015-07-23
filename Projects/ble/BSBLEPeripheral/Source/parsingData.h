#include "ioCC2540.h"
#include "OSAL.h"


/*******************************************************************************
 * TYPEDEF
 */

/*******************************************************************************
 * MACROS
 */

#define set_bit(data, pos)              ((data) |= (0x1<<(pos)))
#define clear_bit(data, pos)            ((data) &= ~(0x1<<(pos)))
#define inver_bit(data, pos)            ((data) ^= (0x1<<(pos)))
#define check_bit(data, pos)            ((data) & (0x1<<(pos)))

#define isLightOneOn()                  (PORT_LIGHT_ONE != 0)
#define isLightTwoOn()                  (PORT_LIGHT_TWO != 0)
#define isLightThreeOn()                (PORT_LIGHT_THREE != 0)

#define setLightOneOn()                 (PORT_LIGHT_ONE = 1)
#define setLightTwoOn()                 (PORT_LIGHT_TWO = 1)
#define setLightThreeOn()               (PORT_LIGHT_THREE = 1)

#define setLightOneOff()                (PORT_LIGHT_ONE = 0)
#define setLightTwoOff()                (PORT_LIGHT_TWO = 0)
#define setLightThreeOff()              (PORT_LIGHT_THREE = 0)

#define setLedOneOn()                   (PORT_LED_ONE = 0)
#define setLedTwoOn()                   (PORT_LED_TWO = 0)
#define setLedThreeOn()                 (PORT_LED_THREE = 0)

#define setLedOneOff()                  (PORT_LED_ONE = 1)
#define setLedTwoOff()                  (PORT_LED_TWO = 1)
#define setLedThreeOff()                (PORT_LED_THREE = 1)


/*********************************************************************
 * CONSTANTS
 */

#define PACKET_LENGTH_RESPONSE          3
#define HEADER_LENGTH                   4
#define PACKET_LENGTH_NORMAL            6
#define PACKET_LENGTH_ABSENCE           10

#define PORT_LED_ONE                    P1_0
#define PORT_LED_TWO                    P1_1
#define PORT_LED_THREE                  P1_2

#define PORT_LIGHT_ONE                  P1_5
#define PORT_LIGHT_TWO                  P1_6
#define PORT_LIGHT_THREE                P1_7

#define PORT_INPUT_SWITCH_ONE           P0_4
#define PORT_INPUT_SWITCH_TWO           P0_5
#define PORT_INPUT_SWITCH_THREE         P0_6

#define LIGHT_ONE                       2
#define LIGHT_TWO                       1   
#define LIGHT_THREE                     0

#define ABSENCE_MODE_CHECK              0x00
#define ABSENCE_MODE_ON                 0x10
#define ABSENCE_MODE_OFF                0x20
#define ABSENCE_MODE_SYNC               0x30

#define TYPE_SWITCH                     0x01
#define TYPE_ALARM                      0x02
#define TYPE_TIMER                      0x04
#define TYPE_WIDGET                     0x08
#define TYPE_ABSENCE                    0x10
#define TYPE_DIMMING                    0x20

#define STX                             0xF0
#define ETX                             0xE0

#define EVT_ABSENCE_LIGHT_ON            0x01
#define EVT_ABSENCE_LIGHT_OFF           0x02
#define EVT_ABSENCE_REGISTER            0x04
#define EVT_ABSENCE_UNREGISTER          0x08

#define MILLIS_MINUTE                   60000
#define MILLIS_HOUR                     (MILLIS_MINUTE * 60)
#define MILLIS_DAY                      (MILLIS_HOUR * 60)
   
/*********************************************************************
 * FUNCTIONS
 */
extern uint8 getParsingDataTaskId();

extern bool parsingDataPacket( uint8* recvPacket , uint8 PacketLength );

/*
 * Task Initialization for the BLE Application
 */
extern void parsingData_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 parsingData_ProcessEvent( uint8 task_id, uint16 events );

extern uint8* getWriteStatePacket();

extern uint8* getReadStatePacket();
