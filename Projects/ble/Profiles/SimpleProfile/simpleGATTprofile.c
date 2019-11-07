
/**************************************************************************************************
  Filename:       simpleGATTprofile.c
  Revised:        $Date: 2013-05-06 13:33:47 -0700 (Mon, 06 May 2013) $
  Revision:       $Revision: 34153 $

  Description:    This file contains the Simple GATT profile sample GATT service 
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
  PROVIDED �AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

#include "simpleGATTprofile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        25

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
#ifdef UUID_128

// Simple GATT Profile Service UUID: 0xFFF0
CONST uint8 simpleProfileServUUID[ATT_UUID_SIZE] =
{ 
  TI_UUID(SIMPLEPROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8 simpleProfilechar1UUID[ATT_UUID_SIZE] =
{ 
  TI_UUID(SIMPLEPROFILE_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFF2
CONST uint8 simpleProfilechar2UUID[ATT_UUID_SIZE] =
{ 
  TI_UUID(SIMPLEPROFILE_CHAR2_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8 simpleProfilechar3UUID[ATT_UUID_SIZE] =
{ 
  TI_UUID(SIMPLEPROFILE_CHAR3_UUID)
};

// Characteristic 4 UUID: 0xFFF4
CONST uint8 simpleProfilechar4UUID[ATT_UUID_SIZE] =
{ 
  TI_UUID(SIMPLEPROFILE_CHAR4_UUID)
};

// Characteristic 5 UUID: 0xFFF5
CONST uint8 SpParaport_charUUID[ATT_UUID_SIZE] =
{ 
  TI_UUID(SP_PARAPORT_CHAR_UUID)
};

// Characteristic bledataport UUID: 0xFFF6
CONST uint8 SpBledataport_charUUID[ATT_UUID_SIZE] =
{ 
  TI_UUID(SP_BLEDATAPORT_CHAR_UUID)
};

// Characteristic serialport UUID: 0xFFF7
CONST uint8 SpSerialport_charUUID[ATT_UUID_SIZE] =
{ 
  TI_UUID(SP_SERIALPORT_CHAR_UUID)
};//read/notification
#else
// Simple GATT Profile Service UUID: 0xFFF0
CONST uint8 simpleProfileServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_SERV_UUID), HI_UINT16(SIMPLEPROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8 simpleProfilechar1UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR1_UUID), HI_UINT16(SIMPLEPROFILE_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFF2
CONST uint8 simpleProfilechar2UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR2_UUID), HI_UINT16(SIMPLEPROFILE_CHAR2_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8 simpleProfilechar3UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR3_UUID), HI_UINT16(SIMPLEPROFILE_CHAR3_UUID)
};

// Characteristic 4 UUID: 0xFFF4
CONST uint8 simpleProfilechar4UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SIMPLEPROFILE_CHAR4_UUID), HI_UINT16(SIMPLEPROFILE_CHAR4_UUID)
};

// Characteristic ParaPort UUID: 0xFFF5
CONST uint8 SpParaport_charUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SP_PARAPORT_CHAR_UUID), HI_UINT16(SP_PARAPORT_CHAR_UUID)
};

// Characteristic bledataport UUID: 0xFFF6
CONST uint8 SpBledataport_charUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SP_BLEDATAPORT_CHAR_UUID), HI_UINT16(SP_BLEDATAPORT_CHAR_UUID)
};

// Characteristic serialport UUID: 0xFFF7
CONST uint8 SpSerialport_charUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(SP_SERIALPORT_CHAR_UUID), HI_UINT16(SP_SERIALPORT_CHAR_UUID)
};//read/notification
#endif
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static simpleProfileCBs_t *simpleProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Simple Profile Service attribute
#ifdef UUID_128
static CONST gattAttrType_t simpleProfileService = { ATT_UUID_SIZE, simpleProfileServUUID };
#else
static CONST gattAttrType_t simpleProfileService = { ATT_BT_UUID_SIZE, simpleProfileServUUID };
#endif

// Simple Profile Characteristic 1 Properties
static uint8 simpleProfileChar1Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 1 Value
static uint8 simpleProfileChar1 = 0;

// Simple Profile Characteristic 1 User Description
static uint8 simpleProfileChar1UserDesp[17] = "Characteristic 1\0";


// Simple Profile Characteristic 2 Properties
static uint8 simpleProfileChar2Props = GATT_PROP_READ ;

// Characteristic 2 Value
static uint8 simpleProfileChar2 = 0;

// Simple Profile Characteristic 2 User Description
static uint8 simpleProfileChar2UserDesp[17] = "Characteristic 2\0";


// Simple Profile Characteristic 3 Properties
static uint8 simpleProfileChar3Props = GATT_PROP_WRITE;

// Characteristic 3 Value
static uint8 simpleProfileChar3 = 0;

// Simple Profile Characteristic 3 User Description
static uint8 simpleProfileChar3UserDesp[17] = "Characteristic 3\0";


// Simple Profile Characteristic 4 Properties
static uint8 simpleProfileChar4Props = GATT_PROP_NOTIFY;

// Characteristic 4 Value
static uint8 simpleProfileChar4 = 0;

// Simple Profile Characteristic 4 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t simpleProfileChar4Config[GATT_MAX_NUM_CONN];
                                        
// Simple Profile Characteristic 4 User Description
static uint8 simpleProfileChar4UserDesp[17] = "Characteristic 4\0";

/************************��������ͨ��*********************************************/
// Simple Profile Characteristic paraport Properties
static uint8 SpParaportCharProps = GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// Characteristic paraport Value
static uint8 SpParaportChar[SP_PARAPORT_CHAR_LEN] = { 1, 2, 3, 4, 5 };

static gattCharCfg_t SpParaportCharConfig[GATT_MAX_NUM_CONN];

// Simple Profile Characteristic paraport User Description
static uint8 SpParaportCharUserDesp[14] = "Char Paraport\0";

/************************��������ͨ��*********************************************/
// Simple Profile Characteristic bledataport Properties
static uint8 SpBledataportCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic bledataport Value
static uint8 SpBledataportChar[SP_BLEDATAPORT_CHAR_LEN] = { 1, 2, 3, 4, 5 };


// Simple Profile Characteristic bledataport User Description
static uint8 SpBledataportCharUserDesp[17] = "Char Bledataport\0";

/************************��������ͨ��*********************************************/
// Simple Profile Characteristic serialport Properties
static uint8 SpSerialportCharProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic serialport Value
uint8 SpSerialportChar[SP_SERIALPORT_CHAR_LEN] = { 1, 2, 3, 4, 5 };

// Simple Profile Characteristic serialport Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t SpSerialportCharConfig[GATT_MAX_NUM_CONN];

// Simple Profile Characteristic serialport User Description
static uint8 SpSerialportCharUserDesp[16] = "Char Serialport\0";

//����ͨ�����ݳ���
uint8 CharBledataportLenth;
//����ͨ�����ݳ���
uint8 CharParaportLenth;

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t simpleProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // Simple Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&simpleProfileService            /* pValue */
  },

    // Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &simpleProfileChar1Props 
    },

      // Characteristic Value 1
      { 
#ifdef UUID_128
        { ATT_UUID_SIZE, simpleProfilechar1UUID },
#else
        { ATT_BT_UUID_SIZE, simpleProfilechar1UUID },
#endif
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &simpleProfileChar1 
      },

      // Characteristic 1 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar1UserDesp 
      },      

    // Characteristic 2 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &simpleProfileChar2Props 
    },

      // Characteristic Value 2
      { 
#ifdef UUID_128
        { ATT_UUID_SIZE, simpleProfilechar2UUID },
#else
        { ATT_BT_UUID_SIZE, simpleProfilechar2UUID },
#endif
        GATT_PERMIT_READ, 
        0, 
        &simpleProfileChar2 
      },

      // Characteristic 2 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar2UserDesp 
      },           
      
    // Characteristic 3 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &simpleProfileChar3Props 
    },

      // Characteristic Value 3
      { 
#ifdef UUID_128
        { ATT_UUID_SIZE, simpleProfilechar3UUID },
#else
        { ATT_BT_UUID_SIZE, simpleProfilechar3UUID },
#endif
        GATT_PERMIT_WRITE, 
        0, 
        &simpleProfileChar3 
      },

      // Characteristic 3 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar3UserDesp 
      },

    // Characteristic 4 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &simpleProfileChar4Props 
    },

      // Characteristic Value 4
      { 
#ifdef UUID_128
        { ATT_UUID_SIZE, simpleProfilechar4UUID },
#else
        { ATT_BT_UUID_SIZE, simpleProfilechar4UUID },
#endif
        0, 
        0, 
        &simpleProfileChar4 
      },

      // Characteristic 4 configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)simpleProfileChar4Config 
      },
      
      // Characteristic 4 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        simpleProfileChar4UserDesp 
      },
/************************��������ͨ��*********************************************/      
    // Characteristic paraport Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &SpParaportCharProps 
    },

      // Characteristic Value paraport
      { 
#ifdef UUID_128
        { ATT_UUID_SIZE, SpParaport_charUUID },
#else
        { ATT_BT_UUID_SIZE, SpParaport_charUUID },
#endif
        GATT_PERMIT_WRITE,
        0, 
        SpParaportChar 
      },

      // Characteristic serialport configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)SpParaportCharConfig 
      },
      
      // Characteristic paraport User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        SpParaportCharUserDesp 
      },
/************************��������ͨ��*********************************************/      
    // Characteristic bledataport Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &SpBledataportCharProps 
    },

      // Characteristic Value bledataport
      { 
#ifdef UUID_128
        { ATT_UUID_SIZE, SpBledataport_charUUID },
#else
        { ATT_BT_UUID_SIZE, SpBledataport_charUUID },
#endif
        GATT_PERMIT_READ|GATT_PERMIT_WRITE,
        0, 
        SpBledataportChar 
      },
      

      // Characteristic bledataport User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        SpBledataportCharUserDesp 
      },
/************************��������ͨ��*********************************************/      
      // Characteristic serialport Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &SpSerialportCharProps 
    },

      // Characteristic Value serialport
      { 
#ifdef UUID_128
        { ATT_UUID_SIZE, SpSerialport_charUUID },
#else
        { ATT_BT_UUID_SIZE, SpSerialport_charUUID },
#endif
        GATT_PERMIT_READ,
        0, 
        SpSerialportChar
      },

      // Characteristic serialport configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)SpSerialportCharConfig 
      },
      
      // Characteristic serialport User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        SpSerialportCharUserDesp 
      },

};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 simpleProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t simpleProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

static void simpleProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType );


/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
CONST gattServiceCBs_t simpleProfileCBs =
{
  simpleProfile_ReadAttrCB,  // Read callback function pointer
  simpleProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleProfile_AddService
 *
 * @brief   Initializes the Simple Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t SimpleProfile_AddService( uint32 services )
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, simpleProfileChar4Config );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, SpParaportCharConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, SpSerialportCharConfig );

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register( simpleProfile_HandleConnStatusCB );  
  
  if ( services & SIMPLEPROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( simpleProfileAttrTbl, 
                                          GATT_NUM_ATTRS( simpleProfileAttrTbl ),
                                          &simpleProfileCBs );
  }

  return ( status );
}


/*********************************************************************
 * @fn      SimpleProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t SimpleProfile_RegisterAppCBs( simpleProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    simpleProfile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}
  

/*********************************************************************
 * @fn      SimpleProfile_SetParameter
 *
 * @brief   Set a Simple Profile parameter.
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
bStatus_t SimpleProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case SIMPLEPROFILE_CHAR1:
      if ( len == sizeof ( uint8 ) ) 
      {
        simpleProfileChar1 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SIMPLEPROFILE_CHAR2:
      if ( len == sizeof ( uint8 ) ) 
      {
        simpleProfileChar2 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SIMPLEPROFILE_CHAR3:
      if ( len == sizeof ( uint8 ) ) 
      {
        simpleProfileChar3 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SIMPLEPROFILE_CHAR4:
      if ( len == sizeof ( uint8 ) ) 
      {
        simpleProfileChar4 = *((uint8*)value);
        
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( simpleProfileChar4Config, &simpleProfileChar4, FALSE,
                                    simpleProfileAttrTbl, GATT_NUM_ATTRS( simpleProfileAttrTbl ),
                                    INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    //��������ͨ��
    case SP_PARAPORT_CHAR:
      
      if ( (len <= SP_PARAPORT_CHAR_LEN) && (len != 0) ) 
      {
        
        VOID osal_memcpy( SpSerialportChar, value, SP_PARAPORT_CHAR_LEN );
        
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( SpParaportCharConfig, SpParaportChar, FALSE,
                                    simpleProfileAttrTbl, GATT_NUM_ATTRS( simpleProfileAttrTbl ),
                                    INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    //��������ͨ�� 
    case SP_BLEDATAPORT_CHAR:
      if ( len == SP_BLEDATAPORT_CHAR_LEN ) 
      {
        VOID osal_memcpy( SpBledataportChar, value, SP_BLEDATAPORT_CHAR_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
    //��������ͨ��  
    case SP_SERIALPORT_CHAR:
      if ( (len <= SP_SERIALPORT_CHAR_LEN) && (len != 0) ) 
      {
        
        VOID osal_memcpy( SpSerialportChar, value, SP_SERIALPORT_CHAR_LEN );
        
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( SpSerialportCharConfig, SpSerialportChar, FALSE,
                                    simpleProfileAttrTbl, GATT_NUM_ATTRS( simpleProfileAttrTbl ),
                                    INVALID_TASK_ID );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn      SimpleProfile_GetParameter
 *
 * @brief   Get a Simple Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t SimpleProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case SIMPLEPROFILE_CHAR1:
      *((uint8*)value) = simpleProfileChar1;
      break;

    case SIMPLEPROFILE_CHAR2:
      *((uint8*)value) = simpleProfileChar2;
      break;      

    case SIMPLEPROFILE_CHAR3:
      *((uint8*)value) = simpleProfileChar3;
      break;  

    case SIMPLEPROFILE_CHAR4:
      *((uint8*)value) = simpleProfileChar4;
      break;
//��������ͨ��
    case SP_PARAPORT_CHAR:
      VOID osal_memcpy( value, SpParaportChar, CharParaportLenth );
      break;      
//��������ͨ��      
    case SP_BLEDATAPORT_CHAR:
      VOID osal_memcpy( value, SpBledataportChar, SP_BLEDATAPORT_CHAR_LEN );
      break;  
//��������ͨ��      
    case SP_SERIALPORT_CHAR:
      VOID osal_memcpy( value, SpSerialportChar, SP_SERIALPORT_CHAR_LEN );
      break;  
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          simpleProfile_ReadAttrCB
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
static uint8 simpleProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
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
      case SIMPLEPROFILE_CHAR1_UUID:
      case SIMPLEPROFILE_CHAR2_UUID:
      case SIMPLEPROFILE_CHAR4_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;
      //��������ͨ��
      case SP_PARAPORT_CHAR_UUID:
        *pLen = SP_PARAPORT_CHAR_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, SP_PARAPORT_CHAR_LEN );
        break;
      //��������ͨ��  
      case SP_BLEDATAPORT_CHAR_UUID:
        *pLen = SP_BLEDATAPORT_CHAR_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, SP_BLEDATAPORT_CHAR_LEN );
        break;
      //��������ͨ��
      case SP_SERIALPORT_CHAR_UUID:
        *pLen = SP_SERIALPORT_CHAR_LEN;
        VOID osal_memcpy( pValue, pAttr->pValue, SP_SERIALPORT_CHAR_LEN );
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
 * @fn      simpleProfile_WriteAttrCB
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
static bStatus_t simpleProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  
  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case SIMPLEPROFILE_CHAR1_UUID:
      case SIMPLEPROFILE_CHAR3_UUID:

        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != 1 )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;        
          *pCurValue = pValue[0];

          if( pAttr->pValue == &simpleProfileChar1 )
          {
            notifyApp = SIMPLEPROFILE_CHAR1;        
          }
          else
          {
            notifyApp = SIMPLEPROFILE_CHAR3;           
          }
        }      
        break;
        
      //��������ͨ�� 
      case SP_PARAPORT_CHAR_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len <= SP_PARAPORT_CHAR_LEN )
          {
           // status = ATT_ERR_INVALID_VALUE_SIZE;
            CharParaportLenth= len;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }

        //Write the value
        if ( status == SUCCESS )
        {
	        VOID osal_memcpy( pAttr->pValue, pValue, SP_PARAPORT_CHAR_LEN );
          notifyApp = SP_PARAPORT_CHAR;
        }
             
        break;
      //��������ͨ�� 
      case SP_BLEDATAPORT_CHAR_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len <= SP_BLEDATAPORT_CHAR_LEN )
          {
           // status = ATT_ERR_INVALID_VALUE_SIZE;
            CharBledataportLenth= len;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }

        //Write the value
        if ( status == SUCCESS )
        {
	        VOID osal_memcpy( pAttr->pValue, pValue, SP_BLEDATAPORT_CHAR_LEN );
          notifyApp = SP_BLEDATAPORT_CHAR;
        }
             
        break;
      //��������ͨ��  
      case SP_SERIALPORT_CHAR_UUID:
        //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != SP_SERIALPORT_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }

        //Write the value
        if ( status == SUCCESS )
        {
	  VOID osal_memcpy( pAttr->pValue, pValue, SP_SERIALPORT_CHAR_LEN );
          notifyApp = SP_SERIALPORT_CHAR;
        }
             
        break;

      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
        
      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    const uint8 uuid[ATT_UUID_SIZE] = 
    { 
    	//Ti�ⲿ��д���д����Ѿ�������
      TI_UUID(BUILD_UINT16( pAttr->type.uuid[12], pAttr->type.uuid[13]))
    };
    
    if ( osal_memcmp(uuid, simpleProfilechar1UUID, ATT_UUID_SIZE) || osal_memcmp(uuid, simpleProfilechar3UUID, ATT_UUID_SIZE))
    {
      //Validate the value
      // Make sure it's not a blob oper
      if ( offset == 0 )
      {
        if ( len != 1 )
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }
        
      //Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;        
        *pCurValue = pValue[0];

        if( pAttr->pValue == &simpleProfileChar1 )
        {
          notifyApp = SIMPLEPROFILE_CHAR1;        
        }
        else
        {
          notifyApp = SIMPLEPROFILE_CHAR3;           
        }
      }

      else
      {      
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
      }
    }
    //5ͨ��
    if ( osal_memcmp(uuid, SpParaport_charUUID, ATT_UUID_SIZE))
   	{
   		  //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len <= SP_PARAPORT_CHAR_LEN )
          {
           // status = ATT_ERR_INVALID_VALUE_SIZE;
            CharParaportLenth= len;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }

        //Write the value
        if ( status == SUCCESS )
        {
	        VOID osal_memcpy( pAttr->pValue, pValue, SP_PARAPORT_CHAR_LEN );
          notifyApp = SP_PARAPORT_CHAR;
        }
   	}
   	//6ͨ��
    if ( osal_memcmp(uuid, SpBledataport_charUUID, ATT_UUID_SIZE))
   	{
   		  //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len <= SP_BLEDATAPORT_CHAR_LEN )
          {
           // status = ATT_ERR_INVALID_VALUE_SIZE;
            CharBledataportLenth= len;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }

        //Write the value
        if ( status == SUCCESS )
        {
	        VOID osal_memcpy( pAttr->pValue, pValue, SP_BLEDATAPORT_CHAR_LEN );
          notifyApp = SP_BLEDATAPORT_CHAR;
        }
   	}
   	//7ͨ��
    if ( osal_memcmp(uuid, SpSerialport_charUUID, ATT_UUID_SIZE))
   	{
   		  //Validate the value
        // Make sure it's not a blob oper
        if ( offset == 0 )
        {
          if ( len != SP_SERIALPORT_CHAR_LEN )
          {
            status = ATT_ERR_INVALID_VALUE_SIZE;
          }
        }
        else
        {
          status = ATT_ERR_ATTR_NOT_LONG;
        }

        //Write the value
        if ( status == SUCCESS )
        {
	        VOID osal_memcpy( pAttr->pValue, pValue, SP_SERIALPORT_CHAR_LEN );
          notifyApp = SP_SERIALPORT_CHAR;
        }
   	}
  }
  
  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && simpleProfile_AppCBs && simpleProfile_AppCBs->pfnSimpleProfileChange )
  {
    simpleProfile_AppCBs->pfnSimpleProfileChange( notifyApp );  
  }
  
  return ( status ); 

} 
  
  
  
  
/*********************************************************************
 * @fn          simpleProfile_HandleConnStatusCB
 *
 * @brief       Simple Profile link status change handler function.
 *
 * @param       connHandle - connection handle
 * @param       changeType - type of change
 *
 * @return      none
 */
static void simpleProfile_HandleConnStatusCB( uint16 connHandle, uint8 changeType )
{ 
  // Make sure this is not loopback connection
  if ( connHandle != LOOPBACK_CONNHANDLE )
  {
    // Reset Client Char Config if connection has dropped
    if ( ( changeType == LINKDB_STATUS_UPDATE_REMOVED )      ||
         ( ( changeType == LINKDB_STATUS_UPDATE_STATEFLAGS ) && 
           ( !linkDB_Up( connHandle ) ) ) )
    { 
      GATTServApp_InitCharCfg( connHandle, simpleProfileChar4Config );
    }
  }
}


/*********************************************************************
*********************************************************************/

