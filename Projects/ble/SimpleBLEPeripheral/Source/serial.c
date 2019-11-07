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
//#include "SerialCMD.h"
#include "hal_board_cfg.h"
/*********************************************************************
 * CONSTANTS
 */
#define DEFAULT_UARTTIMEOUT_PERIOD           5
/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SerialRxBuff[200]; //���ڽ������ݻ���
uint8 *Rx_q = SerialRxBuff;//ָ�򴮿ڻ�����
uint8 uart_remain_len;//��������ʣ�೤��
uint8 offs_RxBuf =0;
uint8 uarting =0;
extern uint8 simpleBLEPeripheral_TaskID;

/*********************************************************************
 * �������ƣ�SC_InitTransport
 * ���ܣ����յ�ǰ���ò����ʳ�ʼ������
 * �������ص���������ǰ������
 * ���أ���
 */
void SC_InitTransport( npiCBack_t npiCBack,uint8 current_BD)
{
  halUARTCfg_t SC_uartConfig;

  // configure UART
  SC_uartConfig.configured           = TRUE;
  SC_uartConfig.baudRate             = current_BD;
  SC_uartConfig.flowControl          = FALSE;
  SC_uartConfig.flowControlThreshold = 48;
  SC_uartConfig.rx.maxBufSize        = 128;
  SC_uartConfig.tx.maxBufSize        = 128;
  SC_uartConfig.idleTimeout          = 60;
  SC_uartConfig.intEnable            = TRUE;
  SC_uartConfig.parity               = 1;
  SC_uartConfig.callBackFunc         = (halUARTCBack_t)npiCBack;

  // start UART
  // Note: Assumes no issue opening UART port.
  (void)HalUARTOpen( NPI_UART_PORT, &SC_uartConfig );

  return;
}
/*********************************************************************
 * �������ƣ�NpiSerialCallback
 * ���ܣ����ڻص����������ڻ�����������ʱ����
 * ���������ںţ������¼�
 * ���أ���
 */
void NpiSerialCallback(uint8 port, uint8 event)
{ 
  (void)port;
   uint16 bufLen;
   if( port!= HAL_UART_PORT_0 ) return;//it should be my port
   bufLen = NPI_RxBufLen();
   if(offs_RxBuf>=200)
   { 
     Rx_q = SerialRxBuff;
     offs_RxBuf =0;
     memset(SerialRxBuff,0,200);
   }
   if ((Rx_q-SerialRxBuff+bufLen)>sizeof(SerialRxBuff)) 
   {
     uint8 len=sizeof(SerialRxBuff)-(Rx_q-SerialRxBuff);  
     HalUARTRead(HAL_UART_PORT_0, Rx_q, len);
     Rx_q += len;
     osal_set_event( simpleBLEPeripheral_TaskID, SBP_UART_EVT);
   }
   else 
   {
     HalUARTRead(HAL_UART_PORT_0, Rx_q, bufLen); //s_RxBuf
     Rx_q+=bufLen;
     osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_UART_EVT, DEFAULT_UARTTIMEOUT_PERIOD );
   }
   if (event==HAL_UART_TX_EMPTY) uarting=0;    
}