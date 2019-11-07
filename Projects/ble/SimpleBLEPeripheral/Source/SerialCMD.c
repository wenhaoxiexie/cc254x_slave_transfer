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
uint8 new_name_len;                                //���µ��������Ƴ���
uint8 scanRspData[31];
extern uint8 DefaultScanRspData[30];
static uint8 SoftwareVersion[15]="RL254X-V1.83\r\n";

extern char *bdAddr2Str( uint8 *pAddr );
/*********************************************************************
 * �������ƣ�SerialCMD_Init
 * ���ܣ���ʼ������SerialCMD
 * ���أ���
 */
void SerialCMD_Init( uint8 task_id )
{
  SerialCMD_TaskID = task_id;
}

/*********************************************************************
 * �������ƣ�SerialCMD_ProcessEvent
 * ���ܣ�SerialCMD�����¼�����
 * ���أ�δ�����¼�
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
    //���²����ʲ���������
    update_BPS();
    return (events ^ SC_CMD_BPS_EVT);
  }
  if ( events & SC_CMD_NAME_EVT)
  {
    //������������
    update_NEWNAME();
    return (events ^ SC_CMD_NAME_EVT );
  }
  if( events & SC_CMD_RST_EVT )
  {
    //��λģ��
    SystemReset();
  }
  
  if( events & SC_CMD_CONI_EVT )
  {
    //�������Ӽ��
    update_newCONI();
    return (events ^ SC_CMD_CONI_EVT );
  }
  
  if( events & SC_CMD_ADVI_EVT )
  {
    //���¹㲥���
    update_newADVI();
  } 
  return 0;
}
 

/***********************************************************************************
***********************************************************************************/





/***********************************************************************************
 * �������ƣ�Uart_Command
 * ���ܣ������������ݣ���������
 * ���أ�����״̬
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
    /*******************�ж���������********************************************/
    //���Դ���
    if(osal_memcmp(OffData_Buff,"TEST",4))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_CMD_TEST,OffData_Buff,typecommand);
      return STATUS_CMD_TEST;
    }

    //��ȡ���ڲ�����
    if(osal_memcmp(OffData_Buff,"BPS?",4))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_FD_BPS,OffData_Buff,typecommand);
      return STATUS_FD_BPS;
    }

    //�޸Ĵ��ڲ�����
    if(osal_memcmp(OffData_Buff, "BPS",3))
    {
      OffData_Buff+=4;
      Uart_CommandService(STATUS_CMD_BPS,OffData_Buff,typecommand);
      return STATUS_CMD_BPS;
    }

    //ģ�鸴λ
    if(osal_memcmp(OffData_Buff, "RST",3))
    {  
      OffData_Buff+=4;
      Uart_CommandService(STATUS_CMD_RST,OffData_Buff,typecommand);
      return STATUS_CMD_RST;
    }
    //��ȡ��������
    if(osal_memcmp(OffData_Buff, "NAME?",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_FD_NAME,OffData_Buff,typecommand);
      return STATUS_FD_NAME;
    }
    
    //�޸���������
    if(osal_memcmp(OffData_Buff, "NAME",4) )
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_CMD_NAME,OffData_Buff,typecommand); 
      return STATUS_CMD_NAME;
    }
    
    //��ѯ�����㲥���
    if(osal_memcmp(OffData_Buff,"ADVI?",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_FD_ADVI,OffData_Buff,typecommand);
      return STATUS_FD_ADVI;
    }
    
    //�޸������㲥���
    if(osal_memcmp(OffData_Buff,"ADVI",4))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_CMD_ADVI,OffData_Buff,typecommand);
      return STATUS_CMD_ADVI;
    }

    //��ѯ�������Ӽ��
    if(osal_memcmp(OffData_Buff,"CONI?",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_FD_CONI,OffData_Buff,typecommand);
      return STATUS_FD_CONI;
    }
    
   //�޸Ĵӻ��������Ӽ��
    if(osal_memcmp(OffData_Buff,"CONI",4))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_CMD_CONI,OffData_Buff,typecommand);
      return STATUS_CMD_CONI;
    } 
   
   //�ָ��������ã����flash�д洢����
    if(osal_memcmp(OffData_Buff,"RENEW",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_CMD_RENEW,OffData_Buff,typecommand);
      return STATUS_CMD_RENEW;
    }
   
   //��ȡ������ַ
    if(osal_memcmp(OffData_Buff, "MAC?",4))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_FD_MAC,OffData_Buff,typecommand);
      return STATUS_FD_MAC;
    }  
    
   //��ȡ����汾��
    if(osal_memcmp(OffData_Buff,"VER",3))
    {
      OffData_Buff+=4;
      Uart_CommandService(STATUS_FD_VERSION,OffData_Buff,typecommand);
      return STATUS_FD_VERSION;
    } 
/******************************V1.83�汾���ӵ�����******************************/   
     //�޸������������
    if(osal_memcmp(OffData_Buff,"PASSKEY",7))
    {
      OffData_Buff+=8;
      Uart_CommandService(STATUS_CMD_PASSKEY,OffData_Buff,typecommand);
      return STATUS_CMD_PASSKEY;
    }
    
    //�޸ķ��书�ʵ��籣��
    if(osal_memcmp(OffData_Buff,"POWER",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_CMD_POWER,OffData_Buff,typecommand);
      return STATUS_CMD_POWER;
    }
    
    //��ѯ���书��
    if(osal_memcmp(OffData_Buff,"POW?",4))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_FD_POW,OffData_Buff,typecommand);
      return STATUS_FD_POW;
    }
    
    //��ѯ����״̬
    if(osal_memcmp(OffData_Buff,"STATU",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_FD_STATU,OffData_Buff,typecommand);
      return STATUS_FD_STATU;
    }  
  
    //�����㲥
    if(osal_memcmp(OffData_Buff,"ADVON",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_CMD_ADVON,OffData_Buff,typecommand);
      return STATUS_CMD_ADVON;
    }       

    //�رչ㲥
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
 * �������ƣ�Uart_CommandService
 * ���ܣ���������������ô�����
 * ���أ�
 */
void Uart_CommandService(uint8 Command , uint8 *DataBuff , uint8 typecommand)
{  
  switch(Command)
  {
    case STATUS_CMD_TEST:
      CMD_Test(typecommand);                        //���Դ����Ƿ�����
    break;
 
    case STATUS_FD_BPS:
      CMD_Find_BPS(typecommand);                    //��ѯ������
      break;
  
    case STATUS_CMD_BPS:
      CMD_Modify_BPS( DataBuff,typecommand );       //�޸Ĳ�����
      break;
      
    case STATUS_CMD_RST:
      CMD_SystemReset(typecommand);                 //ģ�鸴λ
      break;
      
    case STATUS_FD_NAME:
      CMD_Find_name(typecommand);                  //��ѯ��������
      break;
      
    case STATUS_CMD_NAME:
      CMD_Modify_name( DataBuff,typecommand );     //�޸���������
      break; 
      
    case STATUS_FD_ADVI:
      CMD_Find_ADVI( typecommand );               //��ѯ�㲥���
      break;
      
    case STATUS_CMD_ADVI:
      CMD_Modify_ADVI( DataBuff,typecommand );    //�޸Ĺ㲥���
      break; 
     
    case STATUS_FD_CONI:
      CMD_Find_CONI( typecommand );               //��ѯ���Ӽ��
      break; 
      
    case STATUS_CMD_CONI:
      CMD_Modify_CONI( DataBuff,typecommand );    //�޸����Ӽ��
      break; 
    
    case STATUS_CMD_RENEW:
      CMD_ParaRenew( typecommand );              //�����ָ���������
      break;  
      
    case STATUS_FD_MAC:
      CMD_Find_MAC(typecommand);                 //��ѯMAC��ַ
      break;
      
    case STATUS_FD_VERSION:
      CMD_Find_Version(typecommand);             //��ѯ����͸���汾  
      break;
      
    case STATUS_CMD_PASSKEY:
      CMD_PASSKEY( DataBuff,typecommand);        //�޸��������
      break;
      
    case STATUS_CMD_POWER:
      CMD_Modify_POWER(DataBuff,typecommand );   //�޸ķ��书��
      break; 
      
    case STATUS_FD_POW:
      CMD_Find_POW(typecommand );               //��ѯ���书��
      break; 
      
    case STATUS_FD_STATU:
      CMD_Find_STATU(typecommand );             //��ѯ����״̬
      break;
 
    case STATUS_CMD_ADVON:
      CMD_Modify_ADVON(typecommand );//�����㲥
      break;  
      
    case STATUS_CMD_ADVOFF:
      CMD_Modify_ADVOFF(typecommand );//�رչ㲥
      break;   
      
    default:
      break;
  }
}





/***********************************************************************************
***********************************************************************************/




  
/*******************************************************************
*      �������ƣ�update_BPS
*      ���ܣ����ڲ����ʸ���
*/
void update_BPS()
{
  static uint8 my_baudRate;
  uint8 SC_BR[7]={0};
  osal_snv_read(BLE_NVID_BPS_PARA,6,SC_BR);
    //�жϲ����ʣ��б仯�͸���
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
     //Ĭ�ϲ�����115200
    my_baudRate=HAL_UART_BR_9600;  // HAL_UART_BR_115200
  }
  SC_InitTransport(NpiSerialCallback,my_baudRate); 
}

/*******************************************************************
*      �������ƣ�update_NEWNAME
*      ���ܣ��������Ƹ���
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
    new_name_len = 0;//�������������ƣ�֮ǰ�������Ƴ�������
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
*      �������ƣ�Init_NAME
*      ���ܣ��������Ƴ�ʼ��
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
*      �������ƣ�update_newCONI 4λ
*      ���ܣ������������Ӽ��
*      ���أ�AT+CONI=OK �޸ĳɹ�
*      4/5  ���� 1ms
*/
uint8 update_newCONI()
{
  uint8 SC_CONI[5]={0};
  //Ĭ���������
  uint16 default_min_interval= 16;//Ĭ�����Ӽ��20ms   16
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
*      �������ƣ�update_newADVI
*      ���ܣ������µ��������
*      ���أ�SUCCESS �޸ĳɹ�
*/
uint8 update_newADVI()
{
  uint8 SC_ADVI[6]={0};
  //Ĭ������
  uint16 default_advInt=160;//Ĭ��100ms
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
*      �������ƣ�update_Passkey
*      ���ܣ����ڲ����ʸ���
*/
void update_Passkey()
{
  uint32 my_passkey;
  uint8 MY_PASSKEY[7]={0};
 
  osal_snv_read(BLE_NVID_PASSKEY_PARA,6,MY_PASSKEY);
  //�ж�PASSKEY�Ƿ�Ϊ��
  if(MY_PASSKEY[0]!='\0')
  {
      my_passkey = (MY_PASSKEY[0]-48)*100000+(MY_PASSKEY[1]-48)*10000+
        (MY_PASSKEY[2]-48)*1000+(MY_PASSKEY[3]-48)*100+(MY_PASSKEY[4]-48)*10
         +(MY_PASSKEY[5]-48);
  }
  else
  {
      //Ĭ��PASSKEY 000000
      my_passkey = 000000;
  }
  GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &my_passkey );
}


/***********************************************************************************
***********************************************************************************/






/*******************************************************************************
*�޸���ز���ָ����籣��
*/
/*******************************************************************************
*      �������ƣ�CMD_Test
*      ���ܣ���������ͨ���Ƿ�������Ӧ
*      CMD��AT+TEST
*      ��Ӧ��OK
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
*      �������ƣ�CMD_Find_BPS
*      ���ܣ���ѯ���ڽ��ղ�����
*      CMD��AT+BPS?
*      ���أ�AT+BPS=Current_BPS
*/
static uint8 CMD_Find_BPS(uint8 typecommand)
{
  uint8 SC_BR[7]={0};
  osal_snv_read(BLE_NVID_BPS_PARA,6,SC_BR);
  
  //�����������Ϊ9600ʱ���֣�������2016��6��3��17:14:45
  if(SC_BR[0] == '9')
  {
   for(uint8 i = 0; i < 7;i++)
   {
     SC_BR[i] = 0;
   }
  osal_snv_read(BLE_NVID_BPS_PARA,4,SC_BR);
  }
  //�����������Ϊ9600ʱ���֣�������2016��6��3��17:14:45 
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
     //Ĭ�ϲ�����115200
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
*      �������ƣ�CMD_Modify_BPS
*      ���ܣ����ڽ��ղ����ʸ�д
*      CMD��AT+BPS=NEWBPS
*      ���籣��
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
      //��������ȷ���иĶ���ʼдflash
      if(!osal_memcmp(SC_BR,newbps,osal_strlen((char*)newbps)))
      {
        osal_snv_write(BLE_NVID_BPS_PARA,osal_strlen((char*)newbps),newbps);
      }
         
            if(typecommand)
           {
             //�ֻ�ֱ��ģʽӦ������ֻ�
             osal_memcpy(SerialRxBuff,"OK+BPS:",7);
             osal_memcpy(&SerialRxBuff[7],newbps,osal_strlen((char*)newbps));
             osal_memcpy(&SerialRxBuff[7+osal_strlen((char*)newbps)],"\r\n",2);
             SendNotification(SP_PARA_HANDLE,7+2+osal_strlen((char*)newbps));
             Rx_q = SerialRxBuff;
             offs_RxBuf=0;
            }
              else
                   {
                     //Ӧ���������
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
      //�ֻ�ֱ��ģʽӦ������ֻ�
      VOID osal_memcpy(SerialRxBuff,"AT+BPS=ERR\r\n",12);
      SendNotification(SP_PARA_HANDLE,12);
      Rx_q = SerialRxBuff;
      offs_RxBuf=0;
    }
    else
    {
      //Ӧ���������
      HalUARTWrite(HAL_UART_PORT_0,"AT+BPS=ERR\r\n",12);
    }
  }
  memset(SerialRxBuff,0,200);
}

/****************************************************************
*      �������ƣ�CMD_SystemReset
*      ���ܣ�ģ�鸴λ
*      CMD��AT+RST
*      ���أ�OK+RST ��λ�ɹ�
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
*      �������ƣ�CMD_Find_name
*      ���ܣ���ѯ���ڽ��ղ�����
*      CMD��AT+NAME?
*      ���أ�AT+NAME=Current_NAME
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
      NPI_WriteTransport(SerialRxBuff,8+2+osal_strlen((char*)current_name));//���������������
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
*      �������ƣ�CMD_Modify_name 
*      ���ܣ����ڽ����������Ƹ�д
*      CMD��AT+RENAME=NEWNAME
*      ���籣��
*/
void CMD_Modify_name(uint8 *newname,uint8 typecommand)
{
  new_name_len = osal_strlen((char*)newname);                                   //��ȡ�����Ƴ���
  osal_snv_write(BLE_NVID_NAME_LEN,1,&new_name_len);
  offs_RxBuf=0;

  //������������󳤶�26�ֽڣ����������Ϣ
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
      NPI_WriteTransport(SerialRxBuff,4+2+new_name_len);//���������������
      }else{
      osal_memcpy(SerialRxBuff,"OK+NAME:",8);
      osal_memcpy(&SerialRxBuff[8],newname,new_name_len);
      osal_memcpy(&SerialRxBuff[8+new_name_len],"\r\n",2);
      NPI_WriteTransport(SerialRxBuff,8+2+new_name_len);//���������������
      }
    }
  }
  memset(SerialRxBuff,0,200);
}


/****************************************************************
*      �������ƣ�CMD_Find_ADVI
*      ���ܣ���ѯ�㲥���
*      CMD��AT+ADVI?
*      ���أ�OK+CONI:para��ѯ�ɹ�
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
         HalUARTWrite(HAL_UART_PORT_0,"OK+ADVI:00100\r\n",15);//Ĭ�Ϲ㲥���100ms
    }
  }
  memset(SerialRxBuff,0,200);
}



/****************************************************************
*      �������ƣ�CMD_Modify_ADVI
*      ���ܣ��޸����Ӽ��para 5 λ��
*      CMD��AT+ADVI
*      ���أ�AT+ADVI=OK �޸ĳɹ�
*/
//Advertising Interval can be any amount of time between 20ms and 10.24s
void CMD_Modify_ADVI( uint8 *new_advi,uint8 typecommand )
{
  uint8 SC_ADVI[6]={0};//�洢��ǰADVI
  offs_RxBuf=0;
  uint16 tem;
  if( osal_strlen((char*)new_advi) == 5 )
  {
    //����ms��ת��Ϊ����  0.625 = 5/8   8/5����1ms
    tem=(uint16)(atoi((char*)new_advi)/0.625);
    if(tem>=32 && tem<=16384)
    {
      osal_snv_read(BLE_NVID_ADVI_PARA,5,SC_ADVI);
      if(!osal_memcmp(new_advi,SC_ADVI,5))
      {
        osal_snv_write(BLE_NVID_ADVI_PARA,5,new_advi);
        osal_set_event( SerialCMD_TaskID, SC_CMD_ADVI_EVT);//��������ADVI�¼�,         ����ģ�����Ч
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
*      �������ƣ�CMD_Find_CONI
*      ���ܣ���ѯ���Ӽ��
*      CMD��AT+CONI?
*      ���أ�OK+CONI:para��ѯ�ɹ�
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
    HalUARTWrite(HAL_UART_PORT_0,"OK+CONI:0020\r\n",14);//Ĭ�����Ӽ��20ms
    }
  }
  memset(SerialRxBuff,0,200);
}


/****************************************************************
*      �������ƣ�CMD_Modify_CONI
*      ���ܣ��޸����Ӽ��
*      CMD��AT+CONI
*      ���أ�AT+CONI=OK �޸ĳɹ�
*/
//Connection Interval- multiple of 1.25ms in range of   7.5ms and 4.0s
void CMD_Modify_CONI( uint8 *new_coni,uint8 typecommand )
{
  uint8 SC_CONI[5]={0};//�洢��ǰCONI
  offs_RxBuf=0;
  uint16 tem;
  if( osal_strlen((char*)new_coni) == 4 )
  {
    //����ms��ת��Ϊ����
    tem=(uint16)(atoi((char*)new_coni)/1.25);
    if(tem>=6 && tem<=3200)
    {
      osal_snv_read(BLE_NVID_CONI_PARA,4,SC_CONI);
      if(!osal_memcmp(new_coni,SC_CONI,4))
      {
        osal_snv_write(BLE_NVID_CONI_PARA,4,new_coni);
        osal_set_event( SerialCMD_TaskID, SC_CMD_CONI_EVT);//��������CONI�¼�
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
*      �������ƣ�CMD_ParaRenew
*      ���ܣ�ģ��ָ���������
*      CMD��AT+RENEW
*      ���أ�OK+RENEW 
*/
void CMD_ParaRenew(uint8 typecommand)
{
  offs_RxBuf=0;
  uint8 Para_name_len = 9;
  static uint8 init_name[]= "SerialCom";
  static uint8 init_bps[]= "9600";
  static uint8 init_advi[]= "00100";//�㲥���100
  static uint8 init_coni[]= "0020";//���Ӽ��20
  static uint8 init_passcode[]="000000";
  static uint8 powerleavel[2] = {0};
  
  osal_snv_write(BLE_NVID_NAME_PARA,9,init_name);                         //�޸�����ʱҪ�ص�ע�����   
  osal_snv_write(BLE_NVID_BPS_PARA,6,init_bps);
  osal_snv_write(BLE_NVID_ADVI_PARA,5,init_advi);
  osal_snv_write(BLE_NVID_CONI_PARA,4,init_coni);
  
   //�ָ�flash�����Ƶĳ���2016��3��18��15:16:44
  osal_snv_write(BLE_NVID_NAME_LEN,1,&Para_name_len);                   //�޸�����ʱҪ�ص�ע�����
  osal_start_timerEx( SerialCMD_TaskID, SC_CMD_RST_EVT,50);
  osal_snv_write(BLE_NVID_POWER_LEAVEL,1,powerleavel);                  //���ʻָ���������
  osal_snv_write(BLE_NVID_PASSKEY_PARA,6,init_passcode);                //����ָ��������� 
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
*      �������ƣ�CMD_Find_MAC
*      ���ܣ���ѯ����ģ��MAC��ַ
*      CMD��AT+MAC=?
*      ���أ�OK+MAC:Current_MAC
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
  //���MAC������
  HalUARTWrite(HAL_UART_PORT_0,"OK+MAC:",7);
  NPI_WriteTransport((unsigned char*)bdAddr2Str( DeviceMAC ) + 2,12);
  HalUARTWrite(HAL_UART_PORT_0,"\r\n",2);
  }
}


/*****************************************************************************
*      �������ƣ�CMD_Find_Version
*      ���ܣ�
*      CMD��AT+VER
*      ��Ӧ��OK:XXX
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


/*      �������ƣ�CMD_PASSKEY
*      ���ܣ��޸�PASSKEY���洢
*      CMD��AT+PASSKEY = xxxxxx ����λ��
*      ���أ�
*/
void CMD_PASSKEY(uint8 *new_passkey,uint8 typecommand)
{
    uint8 SC_PASSKEY[7]={0};//�洢��ǰCONI
    uint8 SC_passkey[7]={0};
    offs_RxBuf=0;
    uint32 tem;
    osal_memcpy(SC_PASSKEY,new_passkey,6);
    //�����ַ���ת��Ϊ����
    tem = (SC_PASSKEY[0]-48)*100000+(SC_PASSKEY[1]-48)*10000+
    (SC_PASSKEY[2]-48)*1000+(SC_PASSKEY[3]-48)*100+(SC_PASSKEY[4]-48)*10
    +(SC_PASSKEY[5]-48);
    
    //�޷�������û�и�������ֵ��Ϊ�������Ժ�0�Ƚ������嵯�����档
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
    osal_start_timerEx( SerialCMD_TaskID, SC_CMD_RST_EVT,50);  //50ms���Զ�����
    memset(SerialRxBuff,0,200);
}

/****************************************************************
*      �������ƣ�CMD_Modify_POWER
*      ���ܣ��޸ķ��书��
*      ���أ�void
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
*      �������ƣ�CMD_Find_POW
*      ���ܣ���ѯ���书��
*      ���أ�void
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
*      �������ƣ�CMD_Find_STATU
*      ���ܣ���ѯ����״̬
*      ���أ�void
*/
void CMD_Find_STATU(uint8 typecommand )
{
   uint8 current_adv_enabled_status;
  //Find the current GAP advertisement status��ȡ��ǰ�㲥״̬
  GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );
  if(current_adv_enabled_status == FALSE)
  {
        if(P1_6 == 0)
        {
         //�������
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
          //�㲥�ر�
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
   //���ӶϿ� �㲥����
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
*      �������ƣ�CMD_Modify_ADVON
*      ���ܣ������㲥
*      ���أ�void
*/
void CMD_Modify_ADVON(uint8 typecommand )
{
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status��ȡ��ǰ�㲥״̬
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
*      �������ƣ�CMD_Modify_ADVOFF
*      ���ܣ��رչ㲥
*      ���أ�void
*/
void CMD_Modify_ADVOFF(uint8 typecommand )
{
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status��ȡ��ǰ�㲥״̬
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
