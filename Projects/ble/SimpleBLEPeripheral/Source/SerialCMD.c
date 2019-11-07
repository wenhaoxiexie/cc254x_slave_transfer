/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "linkdb.h"
#include "gatt.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "hal_dma.h"
#include "hal_uart.h"
#include "hal_flash.h"
#include "string.h"
#include "stdlib.h"
#include "hci.h"
#include "hal_adc.h"
#include "OSAL_PwrMgr.h"
#include "NPI.h"
#include "simpleBLEPeripheral.h"
#include "SerialCMD.h"
#include "hal_board_cfg.h"
#include "serial.h"
#include "osal_snv.h"
#include "simpleGATTprofile.h"


static uint8 SerialCMD_TaskID;
uint8 newbps_len;
extern uint8 attDeviceName[GAP_DEVICE_NAME_LEN];
uint8 new_name_len;                                //更新的蓝牙名称长度
uint8 scanRspData[31];
extern uint8 DefaultScanRspData[30];
static uint8 SoftwareVersion[15]="RL254X-V1.83\r\n";

extern char *bdAddr2Str( uint8 *pAddr );
/*********************************************************************
 * 函数名称：SerialCMD_Init
 * 功能：初始化任务SerialCMD
 * 返回：无
 */
void SerialCMD_Init( uint8 task_id )
{
  SerialCMD_TaskID = task_id;
}

/*********************************************************************
 * 函数名称：SerialCMD_ProcessEvent
 * 功能：SerialCMD任务事件处理
 * 返回：未处理事件
 */
uint16 SerialCMD_ProcessEvent( uint8 task_id, uint16 events )
{
  VOID task_id;
  if ( events & SYS_EVENT_MSG )
  {
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  if ( events & SC_CMD_BPS_EVT)
  {
    //更新波特率并启动串口
    update_BPS();
    return (events ^ SC_CMD_BPS_EVT);
  }
  if ( events & SC_CMD_NAME_EVT)
  {
    //更新蓝牙名称
    update_NEWNAME();
    return (events ^ SC_CMD_NAME_EVT );
  }
  if( events & SC_CMD_RST_EVT )
  {
    //复位模块
    SystemReset();
  }
  
  if( events & SC_CMD_CONI_EVT )
  {
    //更新连接间隔
    update_newCONI();
    return (events ^ SC_CMD_CONI_EVT );
  }
  
  if( events & SC_CMD_ADVI_EVT )
  {
    //更新广播间隔
    update_newADVI();
  } 
  return 0;
}
 

/***********************************************************************************
***********************************************************************************/





/***********************************************************************************
 * 函数名称：Uart_Command
 * 功能：分析串口数据，解析命令
 * 返回：命令状态
 */
uint8 Uart_Command(uint8 *p,uint8 typecommand)
{
  uint8 i=0;
  uint8 * OffData_Buff;
  OffData_Buff=p;
  if(!osal_memcmp(OffData_Buff, "AT+", 3) &&  !osal_memcmp(OffData_Buff, "CNM=", 4) )
  {
    return STATUS_CMD_ERR;
  }
  else
  {
   if(osal_memcmp(OffData_Buff, "CNM=", 4))
   {
    OffData_Buff+=4; 
    Uart_CommandService(STATUS_CMD_NAME,OffData_Buff,typecommand);
   }
   else{
   
         OffData_Buff+=3;
         while(((*OffData_Buff) != '='))
         {
             if((*OffData_Buff)=='\0')
             {
               *OffData_Buff = '=';
              }
              else
              {
                OffData_Buff++;
                i++;
              }
         }
             OffData_Buff-=i;
              i--;
    /*******************判断命令类型********************************************/
    //测试串口
    if(osal_memcmp(OffData_Buff,"TEST",4))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_CMD_TEST,OffData_Buff,typecommand);
      return STATUS_CMD_TEST;
    }

    //读取串口波特率
    if(osal_memcmp(OffData_Buff,"BPS?",4))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_FD_BPS,OffData_Buff,typecommand);
      return STATUS_FD_BPS;
    }

    //修改串口波特率
    if(osal_memcmp(OffData_Buff, "BPS",3))
    {
      OffData_Buff+=4;
      Uart_CommandService(STATUS_CMD_BPS,OffData_Buff,typecommand);
      return STATUS_CMD_BPS;
    }

    //模块复位
    if(osal_memcmp(OffData_Buff, "RST",3))
    {  
      OffData_Buff+=4;
      Uart_CommandService(STATUS_CMD_RST,OffData_Buff,typecommand);
      return STATUS_CMD_RST;
    }
    //读取蓝牙名称
    if(osal_memcmp(OffData_Buff, "NAME?",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_FD_NAME,OffData_Buff,typecommand);
      return STATUS_FD_NAME;
    }
    
    //修改蓝牙名称
    if(osal_memcmp(OffData_Buff, "NAME",4) )
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_CMD_NAME,OffData_Buff,typecommand); 
      return STATUS_CMD_NAME;
    }
    
    //查询蓝牙广播间隔
    if(osal_memcmp(OffData_Buff,"ADVI?",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_FD_ADVI,OffData_Buff,typecommand);
      return STATUS_FD_ADVI;
    }
    
    //修改蓝牙广播间隔
    if(osal_memcmp(OffData_Buff,"ADVI",4))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_CMD_ADVI,OffData_Buff,typecommand);
      return STATUS_CMD_ADVI;
    }

    //查询蓝牙连接间隔
    if(osal_memcmp(OffData_Buff,"CONI?",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_FD_CONI,OffData_Buff,typecommand);
      return STATUS_FD_CONI;
    }
    
   //修改从机蓝牙连接间隔
    if(osal_memcmp(OffData_Buff,"CONI",4))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_CMD_CONI,OffData_Buff,typecommand);
      return STATUS_CMD_CONI;
    } 
   
   //恢复出厂设置，清楚flash中存储参数
    if(osal_memcmp(OffData_Buff,"RENEW",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_CMD_RENEW,OffData_Buff,typecommand);
      return STATUS_CMD_RENEW;
    }
   
   //读取蓝牙地址
    if(osal_memcmp(OffData_Buff, "MAC?",4))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_FD_MAC,OffData_Buff,typecommand);
      return STATUS_FD_MAC;
    }  
    
   //读取软件版本号
    if(osal_memcmp(OffData_Buff,"VER",3))
    {
      OffData_Buff+=4;
      Uart_CommandService(STATUS_FD_VERSION,OffData_Buff,typecommand);
      return STATUS_FD_VERSION;
    } 
/******************************V1.83版本增加的命令******************************/   
     //修改蓝牙配对密码
    if(osal_memcmp(OffData_Buff,"PASSKEY",7))
    {
      OffData_Buff+=8;
      Uart_CommandService(STATUS_CMD_PASSKEY,OffData_Buff,typecommand);
      return STATUS_CMD_PASSKEY;
    }
    
    //修改发射功率掉电保存
    if(osal_memcmp(OffData_Buff,"POWER",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_CMD_POWER,OffData_Buff,typecommand);
      return STATUS_CMD_POWER;
    }
    
    //查询发射功率
    if(osal_memcmp(OffData_Buff,"POW?",4))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_FD_POW,OffData_Buff,typecommand);
      return STATUS_FD_POW;
    }
    
    //查询蓝牙状态
    if(osal_memcmp(OffData_Buff,"STATU",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_FD_STATU,OffData_Buff,typecommand);
      return STATUS_FD_STATU;
    }  
  
    //开启广播
    if(osal_memcmp(OffData_Buff,"ADVON",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_CMD_ADVON,OffData_Buff,typecommand);
      return STATUS_CMD_ADVON;
    }       

    //关闭广播
    if(osal_memcmp(OffData_Buff,"ADVOFF",6))
    {
      OffData_Buff+=7;
      Uart_CommandService(STATUS_CMD_ADVOFF,OffData_Buff,typecommand);
      return STATUS_CMD_ADVOFF;
    }    
    

         }
      }
   return STATUS_CMD_ERR;
  }



/*********************************************************************
 * 函数名称：Uart_CommandService
 * 功能：分析串口命令，调用处理函数
 * 返回：
 */
void Uart_CommandService(uint8 Command , uint8 *DataBuff , uint8 typecommand)
{  
  switch(Command)
  {
    case STATUS_CMD_TEST:
      CMD_Test(typecommand);                        //测试串口是否正常
    break;
 
    case STATUS_FD_BPS:
      CMD_Find_BPS(typecommand);                    //查询波特率
      break;
  
    case STATUS_CMD_BPS:
      CMD_Modify_BPS( DataBuff,typecommand );       //修改波特率
      break;
      
    case STATUS_CMD_RST:
      CMD_SystemReset(typecommand);                 //模块复位
      break;
      
    case STATUS_FD_NAME:
      CMD_Find_name(typecommand);                  //查询蓝牙名称
      break;
      
    case STATUS_CMD_NAME:
      CMD_Modify_name( DataBuff,typecommand );     //修改蓝牙名称
      break; 
      
    case STATUS_FD_ADVI:
      CMD_Find_ADVI( typecommand );               //查询广播间隔
      break;
      
    case STATUS_CMD_ADVI:
      CMD_Modify_ADVI( DataBuff,typecommand );    //修改广播间隔
      break; 
     
    case STATUS_FD_CONI:
      CMD_Find_CONI( typecommand );               //查询连接间隔
      break; 
      
    case STATUS_CMD_CONI:
      CMD_Modify_CONI( DataBuff,typecommand );    //修改连接间隔
      break; 
    
    case STATUS_CMD_RENEW:
      CMD_ParaRenew( typecommand );              //参数恢复出厂设置
      break;  
      
    case STATUS_FD_MAC:
      CMD_Find_MAC(typecommand);                 //查询MAC地址
      break;
      
    case STATUS_FD_VERSION:
      CMD_Find_Version(typecommand);             //查询串口透传版本  
      break;
      
    case STATUS_CMD_PASSKEY:
      CMD_PASSKEY( DataBuff,typecommand);        //修改配对密码
      break;
      
    case STATUS_CMD_POWER:
      CMD_Modify_POWER(DataBuff,typecommand );   //修改发射功率
      break; 
      
    case STATUS_FD_POW:
      CMD_Find_POW(typecommand );               //查询发射功率
      break; 
      
    case STATUS_FD_STATU:
      CMD_Find_STATU(typecommand );             //查询蓝牙状态
      break;
 
    case STATUS_CMD_ADVON:
      CMD_Modify_ADVON(typecommand );//开启广播
      break;  
      
    case STATUS_CMD_ADVOFF:
      CMD_Modify_ADVOFF(typecommand );//关闭广播
      break;   
      
    default:
      break;
  }
}





/***********************************************************************************
***********************************************************************************/




  
/*******************************************************************
*      函数名称：update_BPS
*      功能：串口波特率更新
*/
void update_BPS()
{
  static uint8 my_baudRate;
  uint8 SC_BR[7]={0};
  osal_snv_read(BLE_NVID_BPS_PARA,6,SC_BR);
    //判断波特率，有变化就更新
  if(SC_BR[0]!='\0')
  { 
    if(osal_memcmp(SC_BR,"4800",4))
      my_baudRate=HAL_UART_BR_4800;
      
    if(osal_memcmp(SC_BR,"9600",4))
      my_baudRate=HAL_UART_BR_9600;
    
    if(osal_memcmp(SC_BR,"19200",5))
      my_baudRate=HAL_UART_BR_19200;
    
    if(osal_memcmp(SC_BR,"38400",5))
      my_baudRate=HAL_UART_BR_38400;
    
    if(osal_memcmp(SC_BR,"57600",5))
      my_baudRate=HAL_UART_BR_57600;
    
    if(osal_memcmp(SC_BR,"115200",6))
      my_baudRate=HAL_UART_BR_115200;
    
    if(osal_memcmp(SC_BR,"230400",6))
      my_baudRate=HAL_UART_BR_230400;
  }
  else
  {
     //默认波特率115200
    my_baudRate=HAL_UART_BR_9600;  // HAL_UART_BR_115200
  }
  SC_InitTransport(NpiSerialCallback,my_baudRate); 
}

/*******************************************************************
*      函数名称：update_NEWNAME
*      功能：蓝牙名称更新
*/
uint8 update_NEWNAME()
{
  static uint8 status;
  uint8 n=0;
  uint8 SC_Name[26]={0};
  osal_snv_read(BLE_NVID_NAME_PARA,new_name_len,SC_Name);
  if(SC_Name[0]!= 0x00)
  {
    while(n < new_name_len )
    {
      scanRspData[n+2]=SC_Name[n];
      n++;
    }
    scanRspData[0]=n+1;
    scanRspData[1]=GAP_ADTYPE_LOCAL_NAME_COMPLETE;
    status = GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, (n+2), scanRspData );
    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, SC_Name );
    new_name_len = 0;//更新完蓝牙名称，之前蓝牙名称长度清零
  }
  else
  {
    status = GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( DefaultScanRspData ), DefaultScanRspData );
    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
  }
  return status;
}

/*******************************************************************
*      函数名称：Init_NAME
*      功能：蓝牙名称初始化
*/
uint8 Init_NAME()
{
  static uint8 status;
  uint8 n=0;
  uint8 Init_Name[26]={0};
  uint8 init_name_len = 0;
  osal_snv_read(BLE_NVID_NAME_LEN,1,&init_name_len);
  osal_snv_read(BLE_NVID_NAME_PARA,init_name_len,Init_Name);
  if(Init_Name[0]!= 0x00)
  {
    while(n < init_name_len )
    {
      scanRspData[n+2]=Init_Name[n];
      n++;
    }
    scanRspData[0]=n+1;
    scanRspData[1]=GAP_ADTYPE_LOCAL_NAME_COMPLETE;
    status = GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, (n+2), scanRspData );
    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, Init_Name );
  }
  else
  {
    status = GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( DefaultScanRspData ), DefaultScanRspData );
    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );
  }
  return status;
}

/****************************************************************
*      函数名称：update_newCONI 4位
*      功能：更新蓝牙连接间隔
*      返回：AT+CONI=OK 修改成功
*      4/5  代表 1ms
*/
uint8 update_newCONI()
{
  uint8 SC_CONI[5]={0};
  //默认蓝牙间隔
  uint16 default_min_interval= 16;//默认连接间隔20ms   16
  uint16 default_max_interval= 16;
  uint16 new_min_interval,new_max_interval;
  osal_snv_read(BLE_NVID_CONI_PARA,4,SC_CONI);
  if( SC_CONI[0] != '\0' )
  {
    uint16 x = (SC_CONI[0]-48)*1000+(SC_CONI[1]-48)*100+(SC_CONI[2]-48)*10+(SC_CONI[3]-48);
    new_min_interval = (uint16)(x/1.25);
    new_max_interval = new_min_interval;
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &new_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &new_max_interval );
    return SUCCESS;
  }
  else
  {
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &default_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &default_max_interval );
    return FALSE;
  }
}

/****************************************************************
*      函数名称：update_newADVI
*      功能：更新新的蓝牙间隔
*      返回：SUCCESS 修改成功
*/
uint8 update_newADVI()
{
  uint8 SC_ADVI[6]={0};
  //默认设置
  uint16 default_advInt=160;//默认100ms
  uint16 new_advInt;
  osal_snv_read(BLE_NVID_ADVI_PARA,5,SC_ADVI);
  if( SC_ADVI[0] != '\0' )
  {
    uint16 x = (SC_ADVI[0]-48)*10000+(SC_ADVI[1]-48)*1000+(SC_ADVI[2]-48)*100+(SC_ADVI[3]-48)*10+(SC_ADVI[4]-48);
    new_advInt = (uint16)(x/0.625);
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, new_advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, new_advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, new_advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, new_advInt );
    return SUCCESS;
  }
  else
  {
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, default_advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, default_advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, default_advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, default_advInt );
    return FALSE;
  }
  
}


/*******************************************************************
*      函数名称：update_Passkey
*      功能：串口波特率更新
*/
void update_Passkey()
{
  uint32 my_passkey;
  uint8 MY_PASSKEY[7]={0};
 
  osal_snv_read(BLE_NVID_PASSKEY_PARA,6,MY_PASSKEY);
  //判断PASSKEY是否为空
  if(MY_PASSKEY[0]!='\0')
  {
      my_passkey = (MY_PASSKEY[0]-48)*100000+(MY_PASSKEY[1]-48)*10000+
        (MY_PASSKEY[2]-48)*1000+(MY_PASSKEY[3]-48)*100+(MY_PASSKEY[4]-48)*10
         +(MY_PASSKEY[5]-48);
  }
  else
  {
      //默认PASSKEY 000000
      my_passkey = 000000;
  }
  GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &my_passkey );
}


/***********************************************************************************
***********************************************************************************/






/*******************************************************************************
*修改相关参数指令，掉电保存
*/
/*******************************************************************************
*      函数名称：CMD_Test
*      功能：测试命令通道是否正常回应
*      CMD：AT+TEST
*      回应：OK
*/
void CMD_Test(uint8 typecommand)
{
  if(typecommand)
  {
    VOID osal_memcpy(SerialRxBuff,"OK\r\n",4);
    SendNotification(SP_PARA_HANDLE,4);
    Rx_q = SerialRxBuff;
    offs_RxBuf=0;
    memset(SerialRxBuff,0,200);  
  }
  else
  {
    HalUARTWrite(HAL_UART_PORT_0,"OK\r\n",4);
  }
}

/****************************************************************
*      函数名称：CMD_Find_BPS
*      功能：查询串口接收波特率
*      CMD：AT+BPS?
*      返回：AT+BPS=Current_BPS
*/
static uint8 CMD_Find_BPS(uint8 typecommand)
{
  uint8 SC_BR[7]={0};
  osal_snv_read(BLE_NVID_BPS_PARA,6,SC_BR);
  
  //解决波特率设为9600时出现？的问题2016年6月3日17:14:45
  if(SC_BR[0] == '9')
  {
   for(uint8 i = 0; i < 7;i++)
   {
     SC_BR[i] = 0;
   }
  osal_snv_read(BLE_NVID_BPS_PARA,4,SC_BR);
  }
  //解决波特率设为9600时出现？的问题2016年6月3日17:14:45 
    if(SC_BR[0] == '4')
  {
   for(uint8 i = 0; i < 7;i++)
   {
     SC_BR[i] = 0;
   }
  osal_snv_read(BLE_NVID_BPS_PARA,4,SC_BR);
  }
  
  if(SC_BR[0] != '\0')
  {
    if(typecommand)
    {
      osal_memcpy(SerialRxBuff,"OK+BPS:",7);
      osal_memcpy(&SerialRxBuff[7],SC_BR,osal_strlen((char*)SC_BR));
      osal_memcpy(&SerialRxBuff[7+osal_strlen((char*)SC_BR)],"\r\n",2);
      SendNotification(SP_PARA_HANDLE,7+2+osal_strlen((char*)SC_BR));
      Rx_q=SerialRxBuff;
      offs_RxBuf=0;
    }
    else
    {
      osal_memcpy(SerialRxBuff,"OK+BPS:",7);
      osal_memcpy(&SerialRxBuff[7],SC_BR,osal_strlen((char*)SC_BR));
      osal_memcpy(&SerialRxBuff[7+osal_strlen((char*)SC_BR)],"\r\n",2);
      NPI_WriteTransport(SerialRxBuff,7+2+osal_strlen((char*)SC_BR));
    }
  }
  else 
  {
     //默认波特率115200
    if(typecommand)
    {
      VOID osal_memcpy(SerialRxBuff,"OK+BPS:115200\r\n",15);
      SendNotification(SP_PARA_HANDLE,15);
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;
    }
    else
    {
      HalUARTWrite(HAL_UART_PORT_0,"OK+BPS:115200\r\n",15);
    }
  }
  memset(SerialRxBuff,0,200);
  return 0;
}

/*****************************************************************************
*      函数名称：CMD_Modify_BPS
*      功能：串口接收波特率改写
*      CMD：AT+BPS=NEWBPS
*      掉电保存
*/
void CMD_Modify_BPS(uint8 *newbps,uint8 typecommand)
{
  newbps_len = osal_strlen((char*)newbps);
  uint8 SC_BR[7]={0};
  osal_snv_read(BLE_NVID_BPS_PARA,6,SC_BR);
  offs_RxBuf=0; 
  if(
      osal_memcmp("4800",newbps,4)||
      osal_memcmp("9600",newbps,4) || osal_memcmp("19200",newbps,5) ||
      osal_memcmp("38400",newbps,5) || osal_memcmp("57600",newbps,5) || 
      osal_memcmp("115200",newbps,6) || osal_memcmp("230400",newbps,6) )
  {
      //波特率正确且有改动开始写flash
      if(!osal_memcmp(SC_BR,newbps,osal_strlen((char*)newbps)))
      {
        osal_snv_write(BLE_NVID_BPS_PARA,osal_strlen((char*)newbps),newbps);
      }
         
            if(typecommand)
           {
             //手机直驱模式应答命令到手机
             osal_memcpy(SerialRxBuff,"OK+BPS:",7);
             osal_memcpy(&SerialRxBuff[7],newbps,osal_strlen((char*)newbps));
             osal_memcpy(&SerialRxBuff[7+osal_strlen((char*)newbps)],"\r\n",2);
             SendNotification(SP_PARA_HANDLE,7+2+osal_strlen((char*)newbps));
             Rx_q = SerialRxBuff;
             offs_RxBuf=0;
            }
              else
                   {
                     //应答命令到串口
                     osal_memcpy(SerialRxBuff,"OK+BPS:",7);
                     osal_memcpy(&SerialRxBuff[7],newbps,osal_strlen((char*)newbps));
                     osal_memcpy(&SerialRxBuff[7+osal_strlen((char*)newbps)],"\r\n",2);
                     NPI_WriteTransport(SerialRxBuff,7+2+osal_strlen((char*)newbps));
                    }
  osal_start_timerEx( SerialCMD_TaskID, SC_CMD_BPS_EVT,50);
  }
  else
  {
    if(typecommand)
    {
      //手机直驱模式应答命令到手机
      VOID osal_memcpy(SerialRxBuff,"AT+BPS=ERR\r\n",12);
      SendNotification(SP_PARA_HANDLE,12);
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;
    }
    else
    {
      //应答命令到串口
      HalUARTWrite(HAL_UART_PORT_0,"AT+BPS=ERR\r\n",12);
    }
  }
  memset(SerialRxBuff,0,200);
}

/****************************************************************
*      函数名称：CMD_SystemReset
*      功能：模块复位
*      CMD：AT+RST
*      返回：OK+RST 复位成功
*/
void CMD_SystemReset(uint8 typecommand)
{
  offs_RxBuf=0;
  osal_set_event( SerialCMD_TaskID, SC_CMD_RST_EVT );
  if(typecommand)
  {
    VOID osal_memcpy(SerialRxBuff,"OK+RST\r\n",8);
    SendNotification(SP_PARA_HANDLE,8);
    Rx_q = SerialRxBuff;
    offs_RxBuf=0;
  } 
  else 
  {  
    HalUARTWrite(HAL_UART_PORT_0,"OK+RST\r\n",8);
  }
  memset(SerialRxBuff,0,200);
}

/****************************************************************
*      函数名称：CMD_Find_name
*      功能：查询串口接收波特率
*      CMD：AT+NAME?
*      返回：AT+NAME=Current_NAME
*/
static uint8 CMD_Find_name(uint8 typecommand)
{
  uint8 name_len;
  uint8 SC_Name[26]={0};
  uint8 current_name[26]={0};
  uint8 n=0;
  osal_snv_read(BLE_NVID_NAME_LEN,1,&name_len);
  osal_snv_read(BLE_NVID_NAME_PARA,name_len,SC_Name);
  if(SC_Name[0] != '\0')
  {
    while(SC_Name[n] != '\0')
    {
      current_name[n]=SC_Name[n];
      n++;
    }
    if(typecommand)
    {
      osal_memcpy(SerialRxBuff,"OK+NAME:",8);
      osal_memcpy(&SerialRxBuff[8],current_name,osal_strlen((char*)current_name));
      osal_memcpy(&SerialRxBuff[8+osal_strlen((char*)current_name)],"\r\n",2);
      SendNotification(SP_PARA_HANDLE,8+2+osal_strlen((char*)current_name));
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;
      //memset(SerialRxBuff,0,200);
    }
    else
    {
      osal_memcpy(SerialRxBuff,"OK+NAME:",8);
      osal_memcpy(&SerialRxBuff[8],current_name,osal_strlen((char*)current_name));
      osal_memcpy(&SerialRxBuff[8+osal_strlen((char*)current_name)],"\r\n",2);
      NPI_WriteTransport(SerialRxBuff,8+2+osal_strlen((char*)current_name));//串口输出蓝牙名称
    }
  }
  else
  {
    if(typecommand)
    {
      osal_memcpy(SerialRxBuff,"OK+NAME:",8);
      osal_memcpy(&SerialRxBuff[8],attDeviceName,osal_strlen((char*)attDeviceName));
      osal_memcpy(&SerialRxBuff[8+osal_strlen((char*)attDeviceName)],"\r\n",2);
      SendNotification(SP_PARA_HANDLE,8+2+osal_strlen((char*)attDeviceName));
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;
      //memset(SerialRxBuff,0,200);
    }
    else
    {
      osal_memcpy(SerialRxBuff,"OK+NAME:",8);
      osal_memcpy(&SerialRxBuff[8],attDeviceName,osal_strlen((char*)attDeviceName));
      osal_memcpy(&SerialRxBuff[8+osal_strlen((char*)attDeviceName)],"\r\n",2);
      NPI_WriteTransport(SerialRxBuff,8+2+osal_strlen((char*)attDeviceName));
    }
  }
  memset(SerialRxBuff,0,200);
  return 0;
}


/*******************************************************************************
*      函数名称：CMD_Modify_name 
*      功能：串口接收蓝牙名称改写
*      CMD：AT+RENAME=NEWNAME
*      掉电保存
*/
void CMD_Modify_name(uint8 *newname,uint8 typecommand)
{
  new_name_len = osal_strlen((char*)newname);                                   //获取新名称长度
  osal_snv_write(BLE_NVID_NAME_LEN,1,&new_name_len);
  offs_RxBuf=0;

  //新蓝牙名称最大长度26字节，输出错误信息
  if(new_name_len>26)
  {
    if(typecommand)
    { 
      VOID osal_memcpy(SerialRxBuff,"AT+NAME=ERR\r\n",13);
      SendNotification(SP_PARA_HANDLE,13);
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;

    }
    else
    {
      HalUARTWrite(HAL_UART_PORT_0,"AT+NAME=ERR\r\n",13);
    }
  }
  else if(new_name_len>0)
  {
      osal_snv_write(BLE_NVID_NAME_PARA,new_name_len,(uint8*)newname);
      
      osal_start_timerEx( SerialCMD_TaskID, SC_CMD_NAME_EVT,50);

    if(typecommand)
    {
      if(osal_memcmp(newname - 4,"CNM=",4))
      {
      osal_memcpy(SerialRxBuff,"CNM:",4);
      osal_memcpy(&SerialRxBuff[4],newname,new_name_len);
      osal_memcpy(&SerialRxBuff[4+new_name_len],"\r\n",2);
      SendNotification(SP_PARA_HANDLE,4+2+new_name_len);
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;
      }
      else
      {
      osal_memcpy(SerialRxBuff,"OK+NAME:",8);
      osal_memcpy(&SerialRxBuff[8],newname,new_name_len);
      osal_memcpy(&SerialRxBuff[8+new_name_len],"\r\n",2);
      SendNotification(SP_PARA_HANDLE,8+2+new_name_len);
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;
      };

    }
    else
    {
       if(osal_memcmp(SerialRxBuff,"CNM=",4))
      {
      osal_memcpy(SerialRxBuff,"CNM:",4);
      osal_memcpy(&SerialRxBuff[4],newname,new_name_len);
      osal_memcpy(&SerialRxBuff[4+new_name_len],"\r\n",2);
      NPI_WriteTransport(SerialRxBuff,4+2+new_name_len);//串口输出蓝牙名称
      }else{
      osal_memcpy(SerialRxBuff,"OK+NAME:",8);
      osal_memcpy(&SerialRxBuff[8],newname,new_name_len);
      osal_memcpy(&SerialRxBuff[8+new_name_len],"\r\n",2);
      NPI_WriteTransport(SerialRxBuff,8+2+new_name_len);//串口输出蓝牙名称
      }
    }
  }
  memset(SerialRxBuff,0,200);
}


/****************************************************************
*      函数名称：CMD_Find_ADVI
*      功能：查询广播间隔
*      CMD：AT+ADVI?
*      返回：OK+CONI:para查询成功
*/
void CMD_Find_ADVI(uint8 typecommand)
{
  uint8 SC_ADVI[6]={0};
  osal_snv_read(BLE_NVID_ADVI_PARA,5,SC_ADVI);
  if(SC_ADVI[0] != '\0')
  {
    if(typecommand)
    {
      osal_memcpy(SerialRxBuff,"OK+ADVI:",8);
      osal_memcpy(&SerialRxBuff[8],SC_ADVI,5);
      osal_memcpy(&SerialRxBuff[13],"\r\n",2);
      SendNotification(SP_PARA_HANDLE,15);
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;
    }
    else
    {
      osal_memcpy(SerialRxBuff,"OK+ADVI:",8);
      osal_memcpy(&SerialRxBuff[8],SC_ADVI,5);
      osal_memcpy(&SerialRxBuff[13],"\r\n",2);
      NPI_WriteTransport(SerialRxBuff,15);
    }
  }
  else
  {
     if(typecommand)
    {
      osal_memcpy(SerialRxBuff,"OK+ADVI:00100\r\n",15);
      SendNotification(SP_PARA_HANDLE,15);
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;
    }
    else
    {
         HalUARTWrite(HAL_UART_PORT_0,"OK+ADVI:00100\r\n",15);//默认广播间隔100ms
    }
  }
  memset(SerialRxBuff,0,200);
}



/****************************************************************
*      函数名称：CMD_Modify_ADVI
*      功能：修改连接间隔para 5 位数
*      CMD：AT+ADVI
*      返回：AT+ADVI=OK 修改成功
*/
//Advertising Interval can be any amount of time between 20ms and 10.24s
void CMD_Modify_ADVI( uint8 *new_advi,uint8 typecommand )
{
  uint8 SC_ADVI[6]={0};//存储当前ADVI
  offs_RxBuf=0;
  uint16 tem;
  if( osal_strlen((char*)new_advi) == 5 )
  {
    //输入ms，转化为参数  0.625 = 5/8   8/5代表1ms
    tem=(uint16)(atoi((char*)new_advi)/0.625);
    if(tem>=32 && tem<=16384)
    {
      osal_snv_read(BLE_NVID_ADVI_PARA,5,SC_ADVI);
      if(!osal_memcmp(new_advi,SC_ADVI,5))
      {
        osal_snv_write(BLE_NVID_ADVI_PARA,5,new_advi);
        osal_set_event( SerialCMD_TaskID, SC_CMD_ADVI_EVT);//启动更新ADVI事件,         重启模块后生效
      }
      if(typecommand)
      {
        osal_memcpy(SerialRxBuff,"OK+ADVI:",8);
        osal_memcpy(&SerialRxBuff[8],new_advi,osal_strlen((char*)new_advi));
        osal_memcpy(&SerialRxBuff[8+osal_strlen((char*)new_advi)],"\r\n",2);
        SendNotification(SP_PARA_HANDLE,8+2+osal_strlen((char*)new_advi));
        Rx_q = SerialRxBuff;
        offs_RxBuf=0;
      }
      else
      {
        osal_memcpy(SerialRxBuff,"OK+ADVI:",8);
        osal_memcpy(&SerialRxBuff[8],new_advi,osal_strlen((char*)new_advi));
        osal_memcpy(&SerialRxBuff[8+osal_strlen((char*)new_advi)],"\r\n",2);
        NPI_WriteTransport(SerialRxBuff,8+2+osal_strlen((char*)new_advi));
      }
    }
    else
    {   
      if(typecommand)
      {
        VOID osal_memcpy(SerialRxBuff,"AT+ADVI=ERR\r\n",13);
        SendNotification(SP_PARA_HANDLE,13);
        Rx_q = SerialRxBuff;
        offs_RxBuf=0;
      }
      else
      {
        HalUARTWrite(HAL_UART_PORT_0,"AT+ADVI=ERR\r\n",13);
      }
    }
  }
  else       
  {
    if(typecommand)
      {
        VOID osal_memcpy(SerialRxBuff,"AT+ADVI=ERR\r\n",13);
        SendNotification(SP_PARA_HANDLE,12);
        Rx_q = SerialRxBuff;
        offs_RxBuf=0;
      }
      else
      {
        HalUARTWrite(HAL_UART_PORT_0,"AT+ADVI=ERR\r\n",13);
      }
  }
  memset(SerialRxBuff,0,200);
}

/****************************************************************
*      函数名称：CMD_Find_CONI
*      功能：查询连接间隔
*      CMD：AT+CONI?
*      返回：OK+CONI:para查询成功
*/
void CMD_Find_CONI(uint8 typecommand)
{
  uint8 SC_CONI[5]={0};
  osal_snv_read(BLE_NVID_CONI_PARA,4,SC_CONI);
  if(SC_CONI[0] != '\0')
  {
    if(typecommand)
    {
      osal_memcpy(SerialRxBuff,"OK+CONI:",8);
      osal_memcpy(&SerialRxBuff[8],SC_CONI,4);
      osal_memcpy(&SerialRxBuff[12],"\r\n",2);
      SendNotification(SP_PARA_HANDLE,14);
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;
    }
    else
    {
      osal_memcpy(SerialRxBuff,"OK+CONI:",8);
      osal_memcpy(&SerialRxBuff[8],SC_CONI,4);
      osal_memcpy(&SerialRxBuff[12],"\r\n",2);
      NPI_WriteTransport(SerialRxBuff,14);
    }
  }
  else
  {
     if(typecommand)
    {
      osal_memcpy(SerialRxBuff,"OK+CONI:0020\r\n",14);
      SendNotification(SP_PARA_HANDLE,14);
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;
    }
    else
    {
    HalUARTWrite(HAL_UART_PORT_0,"OK+CONI:0020\r\n",14);//默认连接间隔20ms
    }
  }
  memset(SerialRxBuff,0,200);
}


/****************************************************************
*      函数名称：CMD_Modify_CONI
*      功能：修改连接间隔
*      CMD：AT+CONI
*      返回：AT+CONI=OK 修改成功
*/
//Connection Interval- multiple of 1.25ms in range of   7.5ms and 4.0s
void CMD_Modify_CONI( uint8 *new_coni,uint8 typecommand )
{
  uint8 SC_CONI[5]={0};//存储当前CONI
  offs_RxBuf=0;
  uint16 tem;
  if( osal_strlen((char*)new_coni) == 4 )
  {
    //输入ms，转化为参数
    tem=(uint16)(atoi((char*)new_coni)/1.25);
    if(tem>=6 && tem<=3200)
    {
      osal_snv_read(BLE_NVID_CONI_PARA,4,SC_CONI);
      if(!osal_memcmp(new_coni,SC_CONI,4))
      {
        osal_snv_write(BLE_NVID_CONI_PARA,4,new_coni);
        osal_set_event( SerialCMD_TaskID, SC_CMD_CONI_EVT);//启动更新CONI事件
      }
      if(typecommand)
      {
        osal_memcpy(SerialRxBuff,"OK+CONI:",8);
        osal_memcpy(&SerialRxBuff[8],new_coni,osal_strlen((char*)new_coni));
        osal_memcpy(&SerialRxBuff[8+osal_strlen((char*)new_coni)],"\r\n",2);
        SendNotification(SP_PARA_HANDLE,8+2+osal_strlen((char*)new_coni));
        Rx_q = SerialRxBuff;
        offs_RxBuf=0;
      }
      else
      {
        osal_memcpy(SerialRxBuff,"OK+CONI:",8);
        osal_memcpy(&SerialRxBuff[8],new_coni,osal_strlen((char*)new_coni));
        osal_memcpy(&SerialRxBuff[8+osal_strlen((char*)new_coni)],"\r\n",2);
        NPI_WriteTransport(SerialRxBuff,8+2+osal_strlen((char*)new_coni));
      }
    }
    else
    {   
      if(typecommand)
      {
        VOID osal_memcpy(SerialRxBuff,"AT+CONI=ERR\r\n",13);
        SendNotification(SP_PARA_HANDLE,13);
        Rx_q = SerialRxBuff;
        offs_RxBuf=0;
      }
      else
      {
        HalUARTWrite(HAL_UART_PORT_0,"AT+CONI=ERR\r\n",13);
      }
    }
  }
  else       
  {
    if(typecommand)
    {
      VOID osal_memcpy(SerialRxBuff,"AT+CONI=ERR\r\n",13);
      SendNotification(SP_PARA_HANDLE,12);
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;
    }
    else
    {
      HalUARTWrite(HAL_UART_PORT_0,"AT+CONI=ERR\r\n",13);
    }
  }
}


/****************************************************************
*      函数名称：CMD_ParaRenew
*      功能：模块恢复出厂设置
*      CMD：AT+RENEW
*      返回：OK+RENEW 
*/
void CMD_ParaRenew(uint8 typecommand)
{
  offs_RxBuf=0;
  uint8 Para_name_len = 9;
  static uint8 init_name[]= "SerialCom";
  static uint8 init_bps[]= "9600";
  static uint8 init_advi[]= "00100";//广播间隔100
  static uint8 init_coni[]= "0020";//连接间隔20
  static uint8 init_passcode[]="000000";
  static uint8 powerleavel[2] = {0};
  
  osal_snv_write(BLE_NVID_NAME_PARA,9,init_name);                         //修改名称时要重点注意此项   
  osal_snv_write(BLE_NVID_BPS_PARA,6,init_bps);
  osal_snv_write(BLE_NVID_ADVI_PARA,5,init_advi);
  osal_snv_write(BLE_NVID_CONI_PARA,4,init_coni);
  
   //恢复flash中名称的长度2016年3月18日15:16:44
  osal_snv_write(BLE_NVID_NAME_LEN,1,&Para_name_len);                   //修改名称时要重点注意此项
  osal_start_timerEx( SerialCMD_TaskID, SC_CMD_RST_EVT,50);
  osal_snv_write(BLE_NVID_POWER_LEAVEL,1,powerleavel);                  //功率恢复出厂设置
  osal_snv_write(BLE_NVID_PASSKEY_PARA,6,init_passcode);                //密码恢复出厂设置 
  if(typecommand)
  {
    VOID osal_memcpy(SerialRxBuff,"OK+RENEW\r\n",10);
    SendNotification(SP_PARA_HANDLE,10);
    Rx_q = SerialRxBuff;
    offs_RxBuf=0;
  } 
  else  
  {
    HalUARTWrite(HAL_UART_PORT_0,"OK+RENEW\r\n",10);
  }
  memset(SerialRxBuff,0,200);
}

/****************************************************************
*      函数名称：CMD_Find_MAC
*      功能：查询蓝牙模块MAC地址
*      CMD：AT+MAC=?
*      返回：OK+MAC:Current_MAC
*/
void CMD_Find_MAC(uint8 typecommand)
{
  uint8 DeviceMAC[6 + 1];
  uint8 *MacAddrPtr = (uint8*)(P_INFOPAGE+HAL_INFOP_IEEE_OSET);
  DeviceMAC[0] = MacAddrPtr[0];
  DeviceMAC[1] = MacAddrPtr[1];
  DeviceMAC[2] = MacAddrPtr[2];
  DeviceMAC[3] = MacAddrPtr[3];
  DeviceMAC[4] = MacAddrPtr[4];
  DeviceMAC[5] = MacAddrPtr[5];
  if(typecommand)
  {
   // VOID osal_memcpy(SerialRxBuff,DeviceMAC,6);
    VOID osal_memcpy(SerialRxBuff,"OK+MAC:",7);
    VOID osal_memcpy(SerialRxBuff + 7,(unsigned char*)bdAddr2Str( DeviceMAC ) + 2,12);
    //VOID osal_memcpy(SerialRxBuff + 18,"\r\n",2);
    SendNotification(SP_PARA_HANDLE,19);
    Rx_q = SerialRxBuff;
    offs_RxBuf=0;
    memset(SerialRxBuff,0,200);
  }
  else
  {
  //输出MAC到串口
  HalUARTWrite(HAL_UART_PORT_0,"OK+MAC:",7);
  NPI_WriteTransport((unsigned char*)bdAddr2Str( DeviceMAC ) + 2,12);
  HalUARTWrite(HAL_UART_PORT_0,"\r\n",2);
  }
}


/*****************************************************************************
*      函数名称：CMD_Find_Version
*      功能：
*      CMD：AT+VER
*      回应：OK:XXX
*/
void CMD_Find_Version(uint8 typecommand)
{
  if(typecommand)
  {
    VOID osal_memcpy(SerialRxBuff,"VER:",4);
    VOID osal_memcpy(&SerialRxBuff[4],SoftwareVersion,osal_strlen((char*)SoftwareVersion));
    SendNotification(SP_PARA_HANDLE,4+osal_strlen((char*)SoftwareVersion));
    Rx_q = SerialRxBuff;
    offs_RxBuf=0;
    memset(SerialRxBuff,0,200);
  }
  else
  {
    HalUARTWrite(HAL_UART_PORT_0,"OK+VER:",7);
    NPI_WriteTransport(SoftwareVersion,13);
  }
}


/*      函数名称：CMD_PASSKEY
*      功能：修改PASSKEY并存储
*      CMD：AT+PASSKEY = xxxxxx （六位）
*      返回：
*/
void CMD_PASSKEY(uint8 *new_passkey,uint8 typecommand)
{
    uint8 SC_PASSKEY[7]={0};//存储当前CONI
    uint8 SC_passkey[7]={0};
    offs_RxBuf=0;
    uint32 tem;
    osal_memcpy(SC_PASSKEY,new_passkey,6);
    //输入字符串转化为参数
    tem = (SC_PASSKEY[0]-48)*100000+(SC_PASSKEY[1]-48)*10000+
    (SC_PASSKEY[2]-48)*1000+(SC_PASSKEY[3]-48)*100+(SC_PASSKEY[4]-48)*10
    +(SC_PASSKEY[5]-48);
    
    //无符号整数没有负数，其值恒为正数所以和0比较无意义弹出警告。
   // if(tem>=0 && tem<=999999)
    
     if(tem<=999999)
    {
       osal_snv_read(BLE_NVID_PASSKEY_PARA,6,SC_passkey);
       if(!osal_memcmp(new_passkey,SC_passkey,6))
       {
         osal_snv_write(BLE_NVID_PASSKEY_PARA,6,new_passkey);
         
         tem = 0;
       }
       if(typecommand)
       {
          osal_memcpy(SerialRxBuff,"OK+PASSKEY:",11);
          osal_memcpy(&SerialRxBuff[11],new_passkey,osal_strlen((char*)new_passkey));
          osal_memcpy(&SerialRxBuff[11+osal_strlen((char*)new_passkey)],"\r\n",2);
          SendNotification(SP_PARA_HANDLE,11+2+osal_strlen((char*)new_passkey));
          Rx_q = SerialRxBuff;
          offs_RxBuf=0;
       }
       else
       {
          osal_memcpy(SerialRxBuff,"OK+PASSKEY:",11);
          osal_memcpy(&SerialRxBuff[11],new_passkey,osal_strlen((char*)new_passkey));
          osal_memcpy(&SerialRxBuff[11+osal_strlen((char*)new_passkey)],"\r\n",2);
          NPI_WriteTransport(SerialRxBuff,11+2+osal_strlen((char*)new_passkey));
       }
    }
    else
    {   
       if(typecommand)
       {
          VOID osal_memcpy(SerialRxBuff,"AT+PASSKEY=ERR\r\n",16);
          SendNotification(SP_PARA_HANDLE,16);
          Rx_q = SerialRxBuff;
          offs_RxBuf=0;
       }
       else
       {
          HalUARTWrite(HAL_UART_PORT_0,"AT+PASSKEY=ERR\r\n",16);
       }
    }
    osal_start_timerEx( SerialCMD_TaskID, SC_CMD_RST_EVT,50);  //50ms后自动重启
    memset(SerialRxBuff,0,200);
}

/****************************************************************
*      函数名称：CMD_Modify_POWER
*      功能：修改发射功率
*      返回：void
*/
void CMD_Modify_POWER(uint8 *new_spwr,uint8 typecommand )
{
    uint8 leavel[2] = {0};
    uint8 save[2] = {0};
     if(osal_memcmp(new_spwr,"-23",3))
    {
     leavel[0] = 0;
     save[0] = 1;
     osal_snv_write(BLE_NVID_POWER_LEAVEL,1,save);
     HCI_EXT_SetTxPowerCmd(leavel[0]);
     if(typecommand)
     {
        VOID osal_memcpy(SerialRxBuff,"OK:-23DB\r\n",10);
        SendNotification(SP_PARA_HANDLE,10);
        Rx_q = SerialRxBuff;
        offs_RxBuf=0;
        memset(SerialRxBuff,0,200);  
     }
      else
     {
       HalUARTWrite(HAL_UART_PORT_0,"OK:-23DB\r\n",10);
     }  
    }
    
    else if(osal_memcmp(new_spwr,"-6",2))
    {
     leavel[0] = 1;
     save[0] = 2;     
     osal_snv_write(BLE_NVID_POWER_LEAVEL,1,save);
     HCI_EXT_SetTxPowerCmd(leavel[0]);
     
     if(typecommand)
     {
        VOID osal_memcpy(SerialRxBuff,"OK:-6DB\r\n",9);
        SendNotification(SP_PARA_HANDLE,9);
        Rx_q = SerialRxBuff;
        offs_RxBuf=0;
        memset(SerialRxBuff,0,200);  
     }
     else
     {
       HalUARTWrite(HAL_UART_PORT_0,"OK:-6DB\r\n",9);
     }    
    }
    
     else if(osal_memcmp(new_spwr,"0",1))
    {
     leavel[0] = 2;
     save[0] = 3; 
     osal_snv_write(BLE_NVID_POWER_LEAVEL,1,save);
     HCI_EXT_SetTxPowerCmd(leavel[0]);  
     if(typecommand)
     {
        VOID osal_memcpy(SerialRxBuff,"OK:0DB\r\n",8);
        SendNotification(SP_PARA_HANDLE,8);
        Rx_q = SerialRxBuff;
        offs_RxBuf=0;
        memset(SerialRxBuff,0,200);  
     }
     else
     {
       HalUARTWrite(HAL_UART_PORT_0,"OK:0DB\r\n",8);
     } 
    }
      else
      {
         if(typecommand)
          {
            VOID osal_memcpy(SerialRxBuff,"AT+SPWR=ERR\r\n",13);
            SendNotification(SP_PARA_HANDLE,13);
            Rx_q = SerialRxBuff;
            offs_RxBuf=0;
          }
          else
          {
             HalUARTWrite(HAL_UART_PORT_0,"AT+SPWR=ERR\r\n",13);
          }
       }
    
}


/****************************************************************
*      函数名称：CMD_Find_POW
*      功能：查询发射功率
*      返回：void
*/
 void  CMD_Find_POW(uint8 typecommand )
 {
   uint8 txPower1[2] = {0};
   osal_snv_read(BLE_NVID_POWER_LEAVEL,1,txPower1);
   if(typecommand)
   {
      if(txPower1[0] == 0x01)
      {
        VOID osal_memcpy(SerialRxBuff,"PWR:-23DB\r\n",11);
        SendNotification(SP_PARA_HANDLE,11);
        Rx_q = SerialRxBuff;
        offs_RxBuf=0;
        memset(SerialRxBuff,0,200);  
      }
      if(txPower1[0] == 0x02)
      {
        VOID osal_memcpy(SerialRxBuff,"PWR:-6DB\r\n",10);
        SendNotification(SP_PARA_HANDLE,10);
        Rx_q = SerialRxBuff;
        offs_RxBuf=0;
        memset(SerialRxBuff,0,200);  
      }
      if(txPower1[0] == 0x03)
      {
        VOID osal_memcpy(SerialRxBuff,"PWR:0DB\r\n",9);
        SendNotification(SP_PARA_HANDLE,9);
        Rx_q = SerialRxBuff;
        offs_RxBuf=0;
        memset(SerialRxBuff,0,200);  
      }      
   }
   else{
         if(txPower1[0] == 0x01)
         {
           HalUARTWrite(HAL_UART_PORT_0,"PWR:-23DB\r\n",11);
         }
         if(txPower1[0] == 0x02)
         {
           HalUARTWrite(HAL_UART_PORT_0,"PWR:-6DB\r\n",10);         
         }
         if(txPower1[0] == 0x03)
         {
           HalUARTWrite(HAL_UART_PORT_0,"PWR:0DB\r\n",9);         
         }
        }
 }
/****************************************************************
*      函数名称：CMD_Find_STATU
*      功能：查询蓝牙状态
*      返回：void
*/
void CMD_Find_STATU(uint8 typecommand )
{
   uint8 current_adv_enabled_status;
  //Find the current GAP advertisement status获取当前广播状态
  GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );
  if(current_adv_enabled_status == FALSE)
  {
        if(P1_6 == 0)
        {
         //连接完成
         if(typecommand)
         {
            VOID osal_memcpy(SerialRxBuff,"CONNECTCOMPETE\r\n",16);
            SendNotification(SP_PARA_HANDLE,16);
            Rx_q = SerialRxBuff;
            offs_RxBuf=0;
            memset(SerialRxBuff,0,200);  
         }else{
                 HalUARTWrite(HAL_UART_PORT_0,"CONNECTCOMPETE\r\n",16);          
               } 
          
        }
          else
         {
          //广播关闭
          if(typecommand)
         {
            VOID osal_memcpy(SerialRxBuff,"ADVERTISING OFF\r\n",17);
            SendNotification(SP_PARA_HANDLE,16);
            Rx_q = SerialRxBuff;
            offs_RxBuf=0;
            memset(SerialRxBuff,0,200);  
         }else{
                 HalUARTWrite(HAL_UART_PORT_0,"ADVERTISING OFF\r\n",17);          
               } 
         }
  }
  else
  {
   //连接断开 广播开启
    if(typecommand)
    {
      VOID osal_memcpy(SerialRxBuff,"ADVERTISING ON\r\n",16);
      SendNotification(SP_PARA_HANDLE,16);
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;
      memset(SerialRxBuff,0,200);  
     }else{
            HalUARTWrite(HAL_UART_PORT_0,"ADVERTISING ON\r\n",16);          
           } 
  }
}
/****************************************************************
*      函数名称：CMD_Modify_ADVON
*      功能：开启广播
*      返回：void
*/
void CMD_Modify_ADVON(uint8 typecommand )
{
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status获取当前广播状态
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
        //change the GAP advertisement status to opposite of current status
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
      }
      
      if(typecommand)
      {
          VOID osal_memcpy(SerialRxBuff,"ADVERTISING ON",14);
          SendNotification(SP_PARA_HANDLE,14);
          Rx_q = SerialRxBuff;
          offs_RxBuf=0;
      }
      else
      {
          HalUARTWrite(HAL_UART_PORT_0,"ADVERTISING ON",14);         
      }
}



/****************************************************************
*      函数名称：CMD_Modify_ADVOFF
*      功能：关闭广播
*      返回：void
*/
void CMD_Modify_ADVOFF(uint8 typecommand )
{
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status获取当前广播状态
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == TRUE )
      {
        new_adv_enabled_status = FALSE;
        //change the GAP advertisement status to opposite of current status
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
      }
      
      if(typecommand)
      {
          VOID osal_memcpy(SerialRxBuff,"ADVERTISING OFF",15);
          SendNotification(SP_PARA_HANDLE,15);
          Rx_q = SerialRxBuff;
          offs_RxBuf=0;
      }
      else
      {
          HalUARTWrite(HAL_UART_PORT_0,"ADVERTISING OFF",15);         
      }
}
