/**************************************************************************************************
  Filename:       BSGATTprofile.c
  Revised:        $Date: 2013-05-06 13:33:47 -0700 (Mon, 06 May 2013) $
  Revision:       $Revision: 34153 $

  Description:    This file contains the BS GATT profile sample GATT service 
                  profile for use with the BLE sample application.

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
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "BSGATTprofile.h"

/*********************************************************************
 * MACROS
 */
   
/*********************************************************************
 * CONSTANTS
 */

#define INDEX_CHAR_ONE_VALUE 2
#define INDEX_CHAR_ONE_CONFIG 3
#define SERVAPP_NUM_ATTR_SUPPORTED        5

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// BS GATT Profile Service UUID: 0xFFF0
CONST uint8 BSProfileServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BSPROFILE_SERV_UUID), HI_UINT16(BSPROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8 BSProfilechar1UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(BSPROFILE_CHAR1_UUID), HI_UINT16(BSPROFILE_CHAR1_UUID)
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static BSProfileCBs_t *BSProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// BS Profile Service attribute
static CONST gattAttrType_t BSProfileService = { ATT_BT_UUID_SIZE, BSProfileServUUID };

// BS Profile Characteristic 1 Properties
static uint8 BSProfileChar1Props = GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// Characteristic 1 Value
static uint8 BSProfileChar1[MAX_LENGTH_CHARATERISTIC_VALUE] = "\0";
static uint16 BSProfileChar1Length = 0;

// BS Profile Characteristic 4 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t BSProfileChar1Config[GATT_MAX_NUM_CONN];

// BS Profile Characteristic 1 User Description
static uint8 BSProfileChar1UserDesp[MAX_LENGTH_CHARATERISTIC_VALUE] = "Characteristic 1\0";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t BSProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // BS Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&BSProfileService                /* pValue */
  },

    // Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &BSProfileChar1Props 
    },

      // Characteristic Value 1
      { 
        { ATT_BT_UUID_SIZE, BSProfilechar1UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8*) BSProfileChar1 
      },
      
      // Characteristic 1 configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)BSProfileChar1Config 
      },

      // Characteristic 1 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        BSProfileChar1UserDesp 
      },       
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 BSProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t BSProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void BSProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// BS Profile Service Callbacks
CONST gattServiceCBs_t BSProfileCBs =
{
  BSProfile_ReadAttrCB,  // Read callback function pointer
  BSProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      BSProfile_AddService
 *
 * @brief   Initializes the BS Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t BSProfile_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, BSProfileChar1Config );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( BSProfile_HandleConnStatusCB );  
  
  if ( services & BSPROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( BSProfileAttrTbl, 
                                          GATT_NUM_ATTRS( BSProfileAttrTbl ),
                                          &BSProfileCBs );
  }

  return ( status );
}


/*********************************************************************
 * @fn      BSProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t BSProfile_RegisterAppCBs( BSProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    BSProfile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}
  

void notifyCharateristicChanged( uint8 param ) {
  switch ( param )
  {
    case BSPROFILE_CHAR1:
      GATTServApp_ProcessCharCfg( BSProfileChar1Config, (uint8*) BSProfileChar1, FALSE,
                                    BSProfileAttrTbl, GATT_NUM_ATTRS( BSProfileAttrTbl ),
                                    INVALID_TASK_ID );
      break;
  }
}

/*********************************************************************
 * @fn      BSProfile_SetParameter
 *
 * @brief   Set a BS Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */

bStatus_t BSProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  
  switch ( param )
  {
    case BSPROFILE_CHAR1:
      osal_memset((uint8*) BSProfileChar1, 0, sizeof(BSProfileChar1));
      osal_memcpy((uint8*) BSProfileChar1, (uint8*) value, len);
      osal_memcpy(&BSProfileChar1Length, &len, sizeof(uint8));
      
      // See if Notification has been enabled
      //notifyCharateristicChanged(BSPROFILE_CHAR1);
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      BSProfile_GetParameter
 *
 * @brief   Get a BS Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t BSProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case BSPROFILE_CHAR1:
      osal_memcpy(value, (uint8*) &BSProfileChar1, BSProfileChar1Length);
      // *((uint8*)value) = BSProfileChar1;
      //*((uint8*)value) = BSProfileAttrTbl[INDEX_CHAR_ONE_VALUE].pValue;
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          BSProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 BSProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics 1 and 2 have read permissions
      // characteritisc 3 does not have read permissions; therefore it is not
      //   included here
      // characteristic 4 does not have read permissions, but because it
      //   can be sent as a notification, it is included here
      case BSPROFILE_CHAR1_UUID:
        *pLen = BSProfileChar1Length;
        osal_memcpy(pValue, pAttr->pValue, BSProfileChar1Length);
        //pValue[0] = *pAttr->pValue;
        break;
        
      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }
  return ( status );
}

/*********************************************************************
 * @fn      BSProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 *
 * @return  Success or Failure
 */
static bStatus_t BSProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset ) {
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;

  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) ) {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE ) {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid ) {
      case BSPROFILE_CHAR1_UUID:
        //Write the value
        if ( status == SUCCESS ) {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          
          BSProfile_SetParameter(BSPROFILE_CHAR1, len, pValue);
          BSProfileChar1Length = len;
          
          osal_memcpy(pCurValue, pValue, len);

          if(osal_memcmp(pValue, &BSProfileChar1, len)) {
            notifyApp = BSPROFILE_CHAR1;
          }
        }
        break;

      case GATT_CLIENT_CHAR_CFG_UUID:
        // when this function is called, connHandle = 0; pValue = 1; len = 2; offset=0;
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len, 
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
        
      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  } else {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && BSProfile_AppCBs && BSProfile_AppCBs->pfnBSProfileChange ) {
    BSProfile_AppCBs->pfnBSProfileChange( notifyApp , pValue , len);  
  }
  
  return ( status );
}

/*********************************************************************
 * @fn          BSProfile_HandleConnStatusCB
 *
 * @brief       BS Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */

static void BSProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) ) { 
      GATTServApp_InitCharCfg( connHandle, BSProfileChar1Config );
    } else if( changeType == LINKDB_STATUS_UPDATE_NEW ) { // when new link is established
      GATTServApp_WriteCharCfg ( connHandle, // configuration value is seted to notify
                                 BSProfileChar1Config, 
                                 GATT_CLIENT_CFG_NOTIFY);
      // if writing is correct, return success(0). else return failure(1)
    } else {
      GATTServApp_InitCharCfg( connHandle, BSProfileChar1Config );
    }
  }
}


/*********************************************************************
*********************************************************************/
