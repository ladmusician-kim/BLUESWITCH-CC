/**************************************************************************************************
  Filename:       BSBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the BS BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "BSGATTprofile.h"

#include "peripheral.h"

#include "gapbondmgr.h"

#include "BSBLEPeripheral.h"
    
#if defined (SERIAL_INTERFACE)
  #include "serialInterface.h"
#endif

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

#include "hal_bs_key.h"
#include "parsingData.h"

/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * CONSTANTS
 */
   

// How often to perform periodic event
#define BBP_CHECK_PERIODIC_EVT_PERIOD                   3000
   
// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          480

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
//#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     80

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          100

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

#define INVALID_CONNHANDLE                            0xFFFF
  
#define TI_COMPANY_ID                                      0x000D

#define FIND_PHONE_EVT_TIMER                               3000
#define FIND_PHONE_END_EVT_TIMER                        5000
#define RESET_EVT_TIMER                                      10000
#define FIND_PHONE_INDEX                                      11


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
// Task ID for internal task/event processing
static uint8 BSBLEPeripheral_TaskID;   

static bool isOperatedDevilAlarm = false;
static bool isAbsenceMode = false;

static bool isResetEvent = false;
static bool isFindPhoneEvent = false;
static bool isFindingPhoneEvent = false;

//static bool is1BtnPressed = false;
//static bool is2BtnPressed = false;
static bool is3BtnPressed = false;
static bool is4BtnPressed = false;

static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "BS BLE Peripheral";

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x0c,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x42, // B
  0x4c, // L
  0x55, // U
  0x45, // E
  0x20, // blank
  0x53, // S
  0x57, // W
  0x49, // I
  0x54, // T
  0x43, // C
  0x48, // H

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100*7/8ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm          ->  Codia´Â ¿©±â 4·Î ¼³Á¤ÇÔ.
};


// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS, // 0x01
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED, // 0x6


  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( BSPROFILE_SERV_UUID ),
  HI_UINT16( BSPROFILE_SERV_UUID ),
  
  0x0C, // length of this data
   GAP_ADTYPE_MANUFACTURER_SPECIFIC, // Manufacturer specific data
   LO_UINT16( TI_COMPANY_ID ), // Manufacturer Code (registered number TBD)
   HI_UINT16( TI_COMPANY_ID ), // Manufacturer Code (registered number TBD)
   0, 0, 0, 0, 0, 0, 0, 0, // Custom uint64 value to be modified dynamically
   0xBB, // Custom UINT8 value to be modified dynamically

}; // advert µ¥ÀÌÅÍ ºÎºÐÀÌ Codia¶û ´Ù¸§.

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void BSBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void BSBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void BSProfileChangeCB(uint8 paramID, uint8* pData, uint8 pLength);
static void performCheckPeriodicTask( void );
static void initGapProfile( void );
static void initGattAttribute( void );
static void initPort( void );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t BSBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t BSBLEPeripheral_BondMgrCBs = // Codia¿¡´Â Bond Manager°¡ ¾øÀ½.
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// BS GATT Profile Callbacks
static BSProfileCBs_t BSBLEPeripheral_BSProfileCBs =
{
  BSProfileChangeCB    // Charactersitic value change callback
};


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      initGapProfile
 *
 * @brief   Initialization GAP stack.
 *
 * @param   none
 *
 * @return  none
 */

void isSetDevilAlarm(bool set) {
  isOperatedDevilAlarm = set;
}

extern void SetAbsenceMode(bool set) {
  isAbsenceMode = set;
}

extern bool GetAbsenceMode() {
  return isAbsenceMode;
}

static void initGapProfile( void ) {
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable = FALSE; // Codia¿¡ ¾øÀ½.

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable ); // Codia´Â ÃÊ±â°ªÀÌ false
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    // GAPROLE_ADVER_OFF_TIMEÀ» 0À¸·Î ¼³Á¤ÇÏ¸é advertisingÇÏÁö ¾Ê°í °ªÀ» false·Î ¹Ù²Þ.. Ãß ÈÄ¿¡ true·Î ¹Ù²Ù¸é
    // µ¿ÀÛÀ» ½ÇÇàÇÏ°Ô ÇÔ.

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData ); 
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData ); // Codia´Â ÁÖ¼® ÇØ³õÀ½.

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  } // Codia´Â º»µå ¸Å´ÏÀú ºÎºÐÀÌ ¾øÀ½.
}

/*********************************************************************
 * @fn      initGapProfile
 *
 * @brief   Initialization GATT Attributes.
 *
 * @param   none
 *
 * @return  none
 */

static void initGattAttribute( void ) {
  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  BSProfile_AddService( GATT_ALL_SERVICES );      // BS GATT Profile
  
  // Register callback with BSGATTprofile
  VOID BSProfile_RegisterAppCBs( &BSBLEPeripheral_BSProfileCBs );
}

/*********************************************************************
 * @fn      initPort
 *
 * @brief   Initialization cc2540 port.
 *
 * @param   none
 *
 * @return  none
 */

static void initPort( void ) {
  
  
  P0SEL = 0; // Configure Port 0 as GPIO ( 0 : General purpose I/O, 1 : Peripheral function )
  P1SEL = 0; // Configure Port 1 as GPIO ( 0 : General purpose I/O, 1 : Peripheral function )
  P2SEL = 0; // Configure Port 2 as GPIO ( 0 : General purpose I/O, 1 : Peripheral function )

  ////////////////////////////////////////////////////////////////////////
  // Register Direction
  // O : Output => one(1), I : Input => zero(0)
  //
  // P0_0 ~7
  // 7 6 5 4 3 2 1 0
  // O I I I O O I O  => 10001101 => 0x8D
  //
  // P0_0 - blank
  // P0_1 - SW1 (INPUT)
  // P0_2 - UART TX
  // P0_3 - UART RX
  // P0_4 - SW2 (INPUT)
  // P0_5 - SW3 (INPUT)
  // P0_6 - SW4 (INPUT)
  // P0_7 - blank
  //
  // P1_0 ~7
  // 7 6 5 4 3 2 1 0
  // O O O I IO O O O  => 11100111 => 0xE7
  //
  // P1_0 - LED1 (OUTPUT)
  // P1_1 - LED2 (OUTPUT)
  // P1_2 - LED3 (OUTPUT)
  // P1_3 - blank
  // P1_4 - blank
  // P1_5 - SW1 (OUTPUT)
  // P1_6 - SW2 (OUTPUT)
  // P1_7 - SW3 (OUTPUT)
  //
  // P2_0 ~7
  // 7 6 5 4 3 2 1 0
  // I I I O O O O O  => 00011111 => 0x1F
  //
  // P2_0 - blank
  // P2_1 - blank
  // P2_2 - DD
  // P2_3 - DC
  // P2_4 - 
  //
  ////////////////////////////////////////////////////////////////////////
  
  P0DIR |= 0x8D; 
  //P1DIR |= 0xFF;
  P1DIR = 0xE7;
  P2DIR |= 0x1F;

  ////////////////////////////////////////////////////////////////////////
  // O : Output => one(1), I : Input => zero(0)
  //
  // P0 => LED OUTPUT & BUTTON => 1À¸·Î ¼¼ÆÃ(LED ´ëºÎºÐÀÌ Ç®¾÷ÀúÇ×À¸·Î ÀÎÇØ 0ÀÌ µé¾î°¡¸é OnµÇ¹Ç·Î) => 0xF2
  // P1 => LED OUTPUT => 1À¸·Î ¼¼ÆÃ => 0xFF
  // P2 => ±âº» ¼³Á¤´ë·Î => 0À¸·Î ¼¼ÆÃ => 0x00
  //
  ////////////////////////////////////////////////////////////////////////
   
  //P0 = 0xF2; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  //P1 = 0xFF;   // All pins on port 1 to low
  //P1_0 = 0;
  //P1_1 = 0;
  //P1_2 = 1;
  //P1_3 = 1;
  //P1_4 = 1;
  //P1_5 = 0;
  //P1_6 = 0;
  //P1_7 = 0;
  //P2 = 0;   // All pins on port 2 to low
  
  PORT_LIGHT_ONE = 0;
  PORT_LIGHT_TWO = 0;  
  PORT_LIGHT_THREE = 0 ;
  
  PORT_LED_ONE = 0;
  PORT_LED_TWO = 0;
  PORT_LED_THREE = 0;
}

/*********************************************************************
 * @fn      BSBLEPeripheral_Init
 *
 * @brief   Initialization function for the BS BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */

void BSBLEPeripheral_Init( uint8 task_id )
{
  BSBLEPeripheral_TaskID = task_id;
  
  initGapProfile();
  initGattAttribute();
  initPort();  
  
  RegisterForKeys( BSBLEPeripheral_TaskID );
  
  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

  // Setup a delayed profile startup
  osal_set_event( BSBLEPeripheral_TaskID, BBP_START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      BSBLEPeripheral_ProcessEvent
 *
 * @brief   BS BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.w
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 BSBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( BSBLEPeripheral_TaskID )) != NULL )
    {
      BSBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & BBP_START_DEVICE_EVT )
  {
    //initGattAttribute();
    
    // Start the Device
    VOID GAPRole_StartDevice( &BSBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &BSBLEPeripheral_BondMgrCBs );
    
    //osal_start_timerEx( BSBLEPeripheral_TaskID, BBP_CHECK_PERIODIC_EVT, BBP_CHECK_PERIODIC_EVT_PERIOD );

    return ( events ^ BBP_START_DEVICE_EVT );
  }
  
  if( events & BBP_GAP_ADVERT_EVT ) 
  {
    Application_StartAdvertise(5000, DEFAULT_ADVERTISING_INTERVAL);
    return (events ^ BBP_GAP_ADVERT_EVT);
  }
  
  if( events & BBP_GAP_ADVERT_TIME_OUT_EVT ) 
  {
    uint8 temp;
    GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, (void *)&temp );
      
    if( temp == TRUE ){
      uint8 advertising_enable = FALSE;
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &advertising_enable );
    } 
    return (events ^ BBP_GAP_ADVERT_TIME_OUT_EVT);
  }
  
  if( events & BBP_GAP_DISCONNECT_EVT ) 
  {
    //GAPRole_TerminateConnection();
    osal_set_event( BSBLEPeripheral_TaskID, BBP_START_DEVICE_EVT );
    
    return (events ^ BBP_GAP_DISCONNECT_EVT);
  }
  
  if ( events & BBP_CHECK_PERIODIC_EVT )
  {
    // ÇöÀç »óÅÂ È®ÀÎ
    if ( BBP_CHECK_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( BSBLEPeripheral_TaskID, BBP_CHECK_PERIODIC_EVT, BBP_CHECK_PERIODIC_EVT_PERIOD );
    }
    
    performCheckPeriodicTask();
      
    return ( events ^ BBP_CHECK_PERIODIC_EVT );
  }
  
  if( events & BBP_RESET_EVT )
  { 
    if( isResetEvent ) 
      HAL_SYSTEM_RESET();
    return ( events ^ BBP_RESET_EVT );
  }
  
  if( events & BBP_FIND_PHONE_EVT )
  {
    if( isFindPhoneEvent ) {
      isFindingPhoneEvent = true;

      advertData[FIND_PHONE_INDEX] = 1;
      GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
      
      osal_start_timerEx( BSBLEPeripheral_TaskID, BBP_FIND_PHONE_END_EVT, FIND_PHONE_END_EVT_TIMER );
    }
    return ( events ^ BBP_FIND_PHONE_EVT );
  }
  
  if( events & BBP_FIND_PHONE_END_EVT )
  {
    if(isFindingPhoneEvent) {
      isFindingPhoneEvent = false;

      advertData[FIND_PHONE_INDEX] = 0;
      GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    }
    return ( events ^ BBP_FIND_PHONE_END_EVT );
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      performCheckPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the BBP__CHECK_PERIODIC_EVT
 *          OSAL event. In this example, After operation of this application is uncorrect,
 *          this application will boot again. and then this application initialize and operate again.
 *
 * @param   none
 *
 * @return  none
 */
static void performCheckPeriodicTask( void )
{
  //Confirm current application state
  static gaprole_States_t currentGapState = GAPROLE_INIT;
  
  GAPRole_GetParameter(GAPROLE_STATE, &currentGapState);
  
  if(currentGapState != GAPROLE_INIT) {
    return;
  } else {
    return;
  }
}

/*********************************************************************
 * @fn      BSBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void BSBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      BSBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
    break;
    
    default:
      // do nothing
    break;
  }
}

/*********************************************************************
 * @fn      BSPeripheral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void BSBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift; // Intentionally unreferenced parameter
  
  if ( keys & HAL_BS_KEY_BTN1 )
  {
    if(isLightOneOn()) {
      setLightOneOff();
      setLedOneOn();
    }
    else {
      setLightOneOn();
      setLedOneOff();
    }
  }
  
  if ( keys & HAL_BS_KEY_BTN2 )
  {
    if(isLightTwoOn()) {
      setLightTwoOff();
      setLedTwoOn();
    }
    else {
      setLightTwoOn();
      setLedTwoOff();
    }
  }
  
  if ( keys & HAL_BS_KEY_BTN3 )
  {
    is3BtnPressed = true;
    isFindPhoneEvent = true;
    
    if(!isFindingPhoneEvent)
      osal_start_timerEx( BSBLEPeripheral_TaskID, BBP_FIND_PHONE_EVT, FIND_PHONE_EVT_TIMER  );
    
    if(isLightThreeOn()) {
      setLightThreeOff();
      setLedThreeOn();
    }
    else {
      setLightThreeOn();
      setLedThreeOff();
    }
  }
  
  if( is3BtnPressed && !(keys & HAL_BS_KEY_BTN3) )
  {
    is3BtnPressed = false;
    isFindPhoneEvent = false;
    
    osal_stop_timerEx( BSBLEPeripheral_TaskID, BBP_FIND_PHONE_EVT );
  }
  
  if ( keys & HAL_BS_KEY_BTN4 )
  {
    if(isOperatedDevilAlarm) {
      uint8* responsePacket = getWriteStatePacket();
      BSProfile_SetParameter(BSPROFILE_CHAR1, PACKET_LENGTH_RESPONSE, responsePacket);
      notifyCharateristicChanged(BSPROFILE_CHAR1);
      osal_mem_free(responsePacket);
      
      isOperatedDevilAlarm = false;
    }
    
    is4BtnPressed = true;
    isResetEvent = true;
    
    osal_start_timerEx( BSBLEPeripheral_TaskID, BBP_RESET_EVT, RESET_EVT_TIMER );
    
    if(isLightThreeOn() && isLightTwoOn() && isLightOneOn()) {
      setLightOneOff();
      setLedOneOn();
      setLightTwoOff();
      setLedTwoOn();
      setLightThreeOff();
      setLedThreeOn();
    } else {
      setLightOneOn();
      setLedOneOff();
      setLightTwoOn();
      setLedTwoOff();
      setLightThreeOn();
      setLedThreeOff();
    }
  }
  
  if( is4BtnPressed && !(keys & HAL_BS_KEY_BTN4) )
  {
    is4BtnPressed = false;
    isResetEvent = false;
    
    osal_stop_timerEx( BSBLEPeripheral_TaskID, BBP_RESET_EVT );
  }
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */

static gaprole_States_t aa;

static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
  gapProfileState = newState;
  aa = newState;
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        isOperatedDevilAlarm = false;
        osal_set_event( BSBLEPeripheral_TaskID, BBP_GAP_ADVERT_EVT );
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        isOperatedDevilAlarm = false;
      }
      break;

    case GAPROLE_CONNECTED:
      {
        //BSProfile_AddService( GATT_ALL_SERVICES );      // BS GATT Profile

        //setCharacteristicNotification(true);
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      {
      }
      break;
      
    case GAPROLE_WAITING:
      {
        isOperatedDevilAlarm = false;
        /*
        if(bsBLEState == BLE_STATE_CONNECTED) {
          osal_set_event( BSBLEPeripheral_TaskID, BBP_GAP_DISCONNECT_EVT );
        } else {
          bsBLEState = BLE_STATE_WAITING;
        }
        */
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        isOperatedDevilAlarm = false;
        osal_set_event( BSBLEPeripheral_TaskID, BBP_GAP_DISCONNECT_EVT );
      }
      break;

    case GAPROLE_ERROR:
      {
        isOperatedDevilAlarm = false;
        osal_set_event( BSBLEPeripheral_TaskID, BBP_GAP_DISCONNECT_EVT );
      }
      break;

    default:
      {
      }
      break;
  }
  /*
  if(newState == GAPROLE_CONNECTED) 
    P1 = 0xfb; // LINK LED ON
  else
    P1 = 0xff; // LINK LED OFF
  */
  
  return;
  
}

/*********************************************************************
 * @fn      BSProfileChangeCB
 *
 * @brief   Callback from BSBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */

static void BSProfileChangeCB(uint8 paramID, uint8* pData, uint8 pLength)
{
  switch( paramID )
  {
    case BSPROFILE_CHAR1:
      {
        // create SendingData Structure & Enqueue
        
        // ¿©±â¼­ ¹ÞÀº µ¥ÀÌÅÍ¸¦ ¹Ù·Î ÆÄ½ÌÇÏ¸é µÇ°Ú´Ù.
        // ÆÄ½ÌÇÏ°í ³­ µÚ¿¡´Â notify È£ÃâÇØ¼­ ¹Ù·Î µ¥ÀÌÅÍ º¸³»¸é µÉ µí.\
       
        // ¾Æ·¡´Â characteristic value¸¦ ÀúÀåÇÏ°í Notify ÇÏ´Â ÇÔ¼ö
        // BSProfile_SetParameter(BSPROFILE_CHAR1, pktLen, pktData);
        // notifyCharateristicChanged(BSPROFILE_CHAR1);
        //uint8** ppData = &pData;
        
        parsingDataPacket(pData, pLength);
          
        /*
        SendingToArduinoData* temp = (SendingToArduinoData*) osal_mem_alloc(sizeof(SendingToArduinoData));
        temp->len = pLength;
        temp->data = osal_mem_alloc(pLength);
        osal_memcpy(temp->data, pData, pLength);
        
        EnqueueSendingToArduinoBuffer(temp);
      */
        // call osal event
        //osal_set_event( getSerialInterfaceTaskId(), SEND_TO_ARDUINO_EVT );
        break;
      }

    default:
      // should not reach here!
      break;
  }
}

uint8 Application_StartAdvertise(uint16 duration, uint16 interval)
{
  (void) duration;
  (void) interval;
  
  if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 status;    
         
      //TODO: Check if advertising parameters are legal
      
      //Set fast advertising interval for user-initiated connections
      //GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, interval );
      //GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, interval );
      //GAP_SetParamValue( TGAP_LIM_ADV_TIMEOUT, duration );
         
      // toggle GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &status );
      if (status == FALSE)
      {
        status = !status;
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &status ); 
        
        return SUCCESS;
      }
      
    }
  return FAILURE;
}

/*********************************************************************
*********************************************************************/
