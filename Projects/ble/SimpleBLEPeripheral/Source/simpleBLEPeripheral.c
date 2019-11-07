/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
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
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"

#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#include "peripheral.h"

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif




   
#include "serial.h"
#include "string.h"
#include "simpleGATTprofile.h"
#include "SerialCMD.h"
#include "Key.h"  


#include "battservice.h" 


#include "osal_snv.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   1000

// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL          160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT         1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
uint8 txPower[2] = {0} ;
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
extern uint8 CharBledataportLenth;
extern uint8 CharParaportLenth;
uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;

uint8 battLevel[5];

uint8 advState=0;

// GAP - SCAN RSP data (max size = 31 bytes)
uint8 DefaultScanRspData[30] =
{
   // length of this data
  0x0a,   
  // complete name
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x65,   // 'e'
  0x72,   // 'r'
  0x69,   // 'i'
  0x61,   // 'a'
  0x6c,   // 'l'
  0x43,   // 'C'
  0x6f,   // 'o'
  0x6d    // 'm'  
/*
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
 */
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
#if defined UUID_128
  0x11,
  GAP_ADTYPE_128BIT_MORE,
  TI_UUID(SIMPLEPROFILE_SERV_UUID),
#else
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),
#endif
};

// GAP GATT Attributes
uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "SerialCom";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );

static void GUA_Key_Process(void);     //按键处理函数

//passkey1密码回调函数
void ProcessPasscodeCB(uint8 *deviceAddr,uint16 connectionHandle,uint8 uiInputs,uint8 uiOutputs );
static void ProcessPairStateCB( uint16 connHandle, uint8 state, uint8 status );
//passkey1密码回调函数

#if defined( CC2540_MINIDK )
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );
#endif

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

//passkey1
static uint8 gPairStatus=0;
/*用来管理当前的状态，如果密码不正确，立即取消连接，0表示未配对，1表示已配对*/
//passkey1

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
   ProcessPasscodeCB,                      // 密码回调
   ProcessPairStateCB                      // 绑定状态回调
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
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
void SimpleBLEPeripheral_Init( uint8 task_id )
{
  //设置发射功率
osal_snv_read(BLE_NVID_POWER_LEAVEL,1,txPower);
if(txPower[0] == '\0' )
{
   txPower[0] = 3;
   HCI_EXT_SetTxPowerCmd (2);
   osal_snv_write(BLE_NVID_POWER_LEAVEL,1,txPower);
      
}else{
       if(txPower[0] == 1)
       {
         HCI_EXT_SetTxPowerCmd (0); 
       }
       if(txPower[0] == 2)
       {
         HCI_EXT_SetTxPowerCmd (1);        
       }
       if(txPower[0] == 3)
       {
         HCI_EXT_SetTxPowerCmd (2);        
       }
}
  simpleBLEPeripheral_TaskID = task_id;
#if defined ADD_CC2590
  HCI_EXT_ExtendRfRangeCmd();
  TXPOWER = 0xF1;
  //设置接收灵敏度
  HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);
#else
  P1SEL &= ~0x0E;
  P1DIR |= 0x0E;
  P1 |= 0;
#endif
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );

  //按键初始化  
  #if defined(GPIO_2_0)
   Key_Init();  
   RegisterForKey(simpleBLEPeripheral_TaskID, SBP_KEY_CHECK_PROCESS_EVT);  
  #endif
  //更新波特率并启动串口
  update_BPS();
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( POWER_SAVING )
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

	#if defined(GPIO_2_0)
     if(!(P2 & 0x01)) //若检测P2.0
     {  
       initial_advertising_enable = TRUE;     
       PICTL &= ~(1 << 3);  //上升沿触发     
     }
    #endif 
    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    //更新蓝牙名称
    Init_NAME();
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    //更新连接间隔
    update_newCONI();
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
  
    
  // Set the GAP Characteristics
  //GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    //更新广播间隔
    update_newADVI();
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
#if defined PASS_KEY
    uint8 pairMode = GAPBOND_PAIRING_MODE_INITIATE;
#else
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
#endif
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = FALSE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
    uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SP_PARAPORT_CHAR_LEN] = { 1, 2, 3, 4, 5 };
    uint8 charValue6[SP_BLEDATAPORT_CHAR_LEN] = { 1, 2, 3, 4, 5 };
    uint8 charValue7[SP_SERIALPORT_CHAR_LEN] = { 1, 2, 3, 4, 5 };
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SP_PARAPORT_CHAR, SP_PARAPORT_CHAR_LEN, charValue5 );
    SimpleProfile_SetParameter( SP_BLEDATAPORT_CHAR, SP_BLEDATAPORT_CHAR_LEN, charValue6 );
    SimpleProfile_SetParameter( SP_SERIALPORT_CHAR, SP_SERIALPORT_CHAR_LEN, charValue7 );
  }
  //初始化IO口

  P0SEL &= ~ 0xC3; //P0.0 P0.1 P0.6 P0.7 Configure as GPIO 1100 0011
  P1SEL &= ~0xF1; // P1 1111 0001 ,P11\P12\P13 PA CONTROL
  P2SEL = 0; // Configure Port 2 as GPIO
  P0DIR &= ~0xC3; //P0.0~P0.7初始化全部设置成输出口
  P1DIR |= 0xF1; //P1 1111 0001,to output
  P2DIR = 0x1E; // (P2.1-P2.4)设置成输出口  P2.0（广播使能脚）设置为输入口 ---0001 1110
  P0 = 0x30; //P0.5初始化高电平，其余口低电平---0010 0000 
  P1 &= ~0xB1;    //1011 0001
  P2 &= ~0x1E;   // All pins on port 2 to low
  P1 |= 0x41;   //P1.0 P1.6设置为高电平 其余口设置输出低电平 ---0100 0001 
    

#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
#endif // FEATURE_OAD

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
                                                                                           //修改波特率要注释掉这句话否则打印错误很多
 // HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

}
// check-----
void check_P0_4(void)
{
    uint8 current_adv_enabled_status=0xff;
    uint8 new_adv_enabled_status=0xff;

      if(!(P0_4&0x01))  
      {
		  if(advState==1)
		  	return ;
    	  GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );
          if( current_adv_enabled_status == FALSE)
          {
            new_adv_enabled_status = TRUE;
            GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
            osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
		  }  
      }
      else
      {
      	  if(advState==0)
				return ;
          {
          	
			GAPRole_TerminateConnection();
            new_adv_enabled_status = FALSE;
            GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
			
            new_adv_enabled_status = TRUE;
			
            GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
		  }
      }
}


/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

#if defined(GPIO_0_4)

  int new_adv_enabled_status =true;
  int current_adv_enabled_status;
  if(!(P0_4 & 0x01)&&(advState!=1))
  {
	GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );
	if(current_adv_enabled_status==false)
	{
     	GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
        osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
	}
  }

#endif

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;
    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_EVT )
  {
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }
    // Perform periodic application task
    performPeriodicTask();
	
    check_P0_4();
    return (events ^ SBP_PERIODIC_EVT);
  } 
  //串口数据处理及命令解析
  if ( events & SBP_UART_EVT )  //UART RXBUFF process
  {
    uart_remain_len = Rx_q - SerialRxBuff;
    //减去偏移地址
    uart_remain_len -= offs_RxBuf;

    //解析命令
    if(Uart_Command(SerialRxBuff,SERIAL_PARA_CMD) != STATUS_CMD_ERR)
    {
      Rx_q = SerialRxBuff;
      uart_remain_len =0;
      memset(SerialRxBuff,0,200);
      return (events ^ SBP_UART_EVT);
    }

    //连续判断4次，一个连接间隔最多发4个包
     if(uart_remain_len >= 20)
    {
      SendNotification(SP_SERIAL_HANDLE,20);
    } 
    if(uart_remain_len >= 20)
    {
      SendNotification(SP_SERIAL_HANDLE,20);
    } 
    if(uart_remain_len >= 20)
    {
     SendNotification(SP_SERIAL_HANDLE,20);
    }
    if (uart_remain_len>20) 
    {
      SendNotification(SP_SERIAL_HANDLE,20);
      if (uart_remain_len>0) 
      { 
        osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_UART_EVT, 40 );
      }
    }
    else if (uart_remain_len>0) 
    {
      SendNotification(SP_SERIAL_HANDLE,uart_remain_len);
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;
      memset(SerialRxBuff,0,200);     
    }
    return (events ^ SBP_UART_EVT);
  }
  
  //按键处理事件
if ( events & SBP_KEY_CHECK_PROCESS_EVT )  
{  
  //防止抖动，确定是按键  
 // if(Key_Check_Pin() == KEY_PRESS)    
  {  
    //按键处理函数  
    #if defined(GPIO_2_0)
    	GUA_Key_Process();
    #endif
  }      
    
  return (events ^ SBP_KEY_CHECK_PROCESS_EVT);  
}
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
  #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
  #endif // #if defined( CC2540_MINIDK )

  default:
    // do nothing
    break;
  }
}

#if defined( CC2540_MINIDK )
/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
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
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 SK_Keys = 0;

  VOID shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_SW_1 )
  {
    SK_Keys |= SK_KEY_LEFT;
  }

  if ( keys & HAL_KEY_SW_2 )
  {

    SK_Keys |= SK_KEY_RIGHT;

    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    // Note:  If PLUS_BROADCASTER is define this condition is ignored and
    //        Device may advertise during connections as well. 
#ifndef PLUS_BROADCASTER  
    if( gapProfileState != GAPROLE_CONNECTED )
    {
#endif // PLUS_BROADCASTER
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
#ifndef PLUS_BROADCASTER
    }
#endif // PLUS_BROADCASTER
  }

  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  SK_SetParameter( SK_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}
#endif // #if defined( CC2540_MINIDK )

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
#ifdef PLUS_BROADCASTER
  static uint8 first_conn_flag = 0;
#endif // PLUS_BROADCASTER
  
  
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
          HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;
      
    //广播状态  未连接
    case GAPROLE_ADVERTISING:
      {
        #if defined MODULE_STATUS
        NPI_WriteTransport("Advertising=OK\r\n",16);
        #endif
        P1_6 = 1;
		P1_0 = 1;
		advState = 0;
/*****************************************************************************/
//因为出现修改完发射功率，然后连接后出现功率升高现象所以这里重新配置功率
//设置发射功率
        
osal_snv_read(BLE_NVID_POWER_LEAVEL,1,txPower);
if(txPower[0] == '\0' )
{
   txPower[0] = 3;
   HCI_EXT_SetTxPowerCmd (2);
   osal_snv_write(BLE_NVID_POWER_LEAVEL,1,txPower);
      
}else{
       if(txPower[0] == 1)
       {
         HCI_EXT_SetTxPowerCmd (0); 
       }
       if(txPower[0] == 2)
       {
         HCI_EXT_SetTxPowerCmd (1);        
       }
       if(txPower[0] == 3)
       {
         HCI_EXT_SetTxPowerCmd (2);        
       }
      }
/*****************************************************************************/        
      }
      break;
    //连接状态   已连接
    case GAPROLE_CONNECTED:
      {        
       #if defined MODULE_STATUS
       NPI_WriteTransport("Connected=OK\r\n",14);
       #endif
       P1_6 = 0;
	   P1_0 = 0;
	   advState = 1;
      }
      break;

    case GAPROLE_CONNECTED_ADV:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;  
    //待机状态 ，未连接
    case GAPROLE_WAITING:
      {
       #if defined MODULE_STATUS
        NPI_WriteTransport("Standby=OK\r\n",12);
       #endif
       P1_6 = 1;
	   P1_0 = 1;	   
	   advState = 0;
/*****************************************************************************/
//因为出现修改完发射功率，然后连接后出现功率升高现象所以这里重新配置功率
//设置发射功率
        
osal_snv_read(BLE_NVID_POWER_LEAVEL,1,txPower);
if(txPower[0] == '\0' )
{
   txPower[0] = 3;
   HCI_EXT_SetTxPowerCmd (2);
   osal_snv_write(BLE_NVID_POWER_LEAVEL,1,txPower);
      
}else{
       if(txPower[0] == 1)
       {
         HCI_EXT_SetTxPowerCmd (0); 
       }
       if(txPower[0] == 2)
       {
         HCI_EXT_SetTxPowerCmd (1);        
       }
       if(txPower[0] == 3)
       {
         HCI_EXT_SetTxPowerCmd (2);        
       }
      }
/*****************************************************************************/
      }
      break;
    //超时，未连接
    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if defined MODULE_STATUS
         NPI_WriteTransport("Standby=OK\r\n",12);
        #endif
        P1_6 = 1;
		P1_0 = 1;		
		advState = 0;
/*****************************************************************************/
//因为出现修改完发射功率，然后连接后出现功率升高现象所以这里重新配置功率
//设置发射功率
        
osal_snv_read(BLE_NVID_POWER_LEAVEL,1,txPower);
if(txPower[0] == '\0' )
{
   txPower[0] = 3;
   HCI_EXT_SetTxPowerCmd (2);
   osal_snv_write(BLE_NVID_POWER_LEAVEL,1,txPower);
      
}else{
       if(txPower[0] == 1)
       {
         HCI_EXT_SetTxPowerCmd (0); 
       }
       if(txPower[0] == 2)
       {
         HCI_EXT_SetTxPowerCmd (1);        
       }
       if(txPower[0] == 3)
       {
         HCI_EXT_SetTxPowerCmd (2);        
       }
      }
/*****************************************************************************/
      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{
  uint8 valueToCopy;
  uint8 stat;

  // Call to retrieve the value of the third characteristic in the profile
  stat = SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &valueToCopy);

  if( stat == SUCCESS )
  {
    /*
     * Call to set that value of the fourth characteristic in the profile. Note
     * that if notifications of the fourth characteristic have been enabled by
     * a GATT client device, then a notification will be sent every time this
     * function is called.
     */
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof(uint8), &valueToCopy);
  }
  
}

/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue;
  uint8 BlePortValue[20] = {0};
  uint8 ParaPortValue[20] = {0};
  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;
     //FFF5 手机参数设置通道 
    case SP_PARAPORT_CHAR:
      memset(ParaPortValue,0,20);
      SimpleProfile_GetParameter( SP_PARAPORT_CHAR, ParaPortValue );
      if(Uart_Command(ParaPortValue,BLE_PARA_CMD) != STATUS_CMD_ERR)
      {
        memset(ParaPortValue,0,20);
      }
      break;
      //FFF6 手机数据发送通道
     case SP_BLEDATAPORT_CHAR:
      SimpleProfile_GetParameter( SP_BLEDATAPORT_CHAR, BlePortValue );
      NPI_WriteTransport(BlePortValue,CharBledataportLenth);
      break;

    default:
      // should not reach here!
      break;
  }
}
//此处该函数屏蔽了，所以不能调用
//#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
//#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

/*********************************************************************
 *  函数名：SendNotification
 *  功能：发送通知数据
 *
 */
void SendNotification(uint16 handle,uint8 noti_len)
{
  //连接上发送通知
  if( gapProfileState == GAPROLE_CONNECTED )
  {
    attHandleValueNoti_t pNoti;
    uint8 *p = pNoti.value;      
    for (uint8 i=0;i<noti_len;i++) 
    {
      *p++=SerialRxBuff[i+offs_RxBuf];
    }  
    pNoti.len = noti_len;
    pNoti.handle = handle;
    GATT_Notification(0,&pNoti,false);
  }
  uart_remain_len-=noti_len;
  offs_RxBuf+=noti_len; 
}


//******************************************************************************        
//name:             GUA_Key_Process        
//introduce:        按键处理函数       
//parameter:        none       
//return:           none               
//******************************************************************************    
static void GUA_Key_Process(void)  
{  
  /*
      HAL_ENTER_ISR();
      #ifdef POWER_SAVING
      CLEAR_SLEEP_MODE();
      #endif
      HAL_EXIT_ISR();
*/
        
        if(PICTL & 0x08)   //判断是下降沿
        {
          //改为上升沿
          PICTL &= ~(1 << 3);  
        }
        else
        {
         PICTL |= (1 << 3);  //下降沿触发
        }
		
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status获取当前广播状态
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE)
      {
        new_adv_enabled_status = TRUE;
        //change the GAP advertisement status to opposite of current status
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
        osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
      }
      else if( current_adv_enabled_status == TRUE)
      {
        new_adv_enabled_status = FALSE;
        //change the GAP advertisement status to opposite of current status
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
        osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT );
      }
      
} 

/*********************************************************************
 * @fn      ProcessPasscodeCB
 *
 * @brief   密码回调函数
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void ProcessPasscodeCB(uint8 *deviceAddr,uint16 connectionHandle,uint8 uiInputs,uint8 uiOutputs )
{
    uint32 passcode = 0;
    uint8 MY_PASSKEY[6]={0};
    osal_snv_read(BLE_NVID_PASSKEY_PARA,6,MY_PASSKEY);
    //判断PASSKEY是否为空
    if(MY_PASSKEY[0]!='\0')
    {
      passcode = ((uint32)MY_PASSKEY[0]-48)*100000+((uint32)MY_PASSKEY[1]-48)*10000+
        ((uint32)MY_PASSKEY[2]-48)*1000+((uint32)MY_PASSKEY[3]-48)*100+((uint32)MY_PASSKEY[4]-48)*10
         +((uint32)MY_PASSKEY[5]-48);
    }
    else
    {
      //默认PASSKEY 000000
      passcode = 123456;
    }
    // Send passcode response
    GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
}
/*********************************************************************
 * @fn      ProcessPairStateCB
 *
 * @brief   密码回掉函数
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
//绑定过程中的状态管理，在这里可以设置标志位，当密码不正确时不允许连接。
static void ProcessPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )/*主机发起连接，会进入开始绑定状态*/
  {
    gPairStatus = 0;
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )/*当主机提交密码后，会进入完成*/
  { 
    if ( status == SUCCESS )
    {
      gPairStatus = 1; /*密码正确*/
    }
    else
    {
      /*密码不正确，或者先前已经绑定*/
      if(status == 8)
      {
        gPairStatus = 1;/*已绑定*/
      }
      else
      {
        gPairStatus = 0;//密码不正确
      }
    }
    //判断配对结果，如果不正确立刻停止连接。
    if(gapProfileState == GAPROLE_CONNECTED && gPairStatus !=1){
      GAPRole_TerminateConnection();
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      HalLcdWriteString( "Bonding success", HAL_LCD_LINE_1 );
    }
  }
}

/*********************************************************************
*********************************************************************/


