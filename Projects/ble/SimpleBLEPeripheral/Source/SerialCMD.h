/*********************************************************************
 * INCLUDES
 */
#include "NPI.h"


/*********************************************************************
 * CONSTANTS
 */
// SerialCmd Task Events
#define SC_CMD_BPS_EVT                               0x0001
#define SC_CMD_NAME_EVT                              0x0002
#define SC_CMD_RST_EVT                               0x0004
#define SC_CMD_CONI_EVT                              0x0008
#define SC_CMD_ADVI_EVT                              0x0010




//CMD Œª÷√
#define SERIAL_PARA_CMD                               0
#define BLE_PARA_CMD                                  1









/*********************************************************************
 * FUNCTIONS
 */
extern void SerialCMD_Init( uint8 task_id );
extern uint16 SerialCMD_ProcessEvent( uint8 task_id, uint16 events );


void update_BPS();
uint8 update_NEWNAME();
uint8 update_newCONI();
uint8 update_newADVI();
uint8 Init_NAME();
uint8 Uart_Command(uint8 *p,uint8 typecommand);
void Uart_CommandService(uint8 Command , uint8 *DataBuff , uint8 typecommand);
void CMD_Test(uint8 typecommand);
static uint8 CMD_Find_BPS(uint8 typecommand);
void CMD_Modify_BPS(uint8 *newbps,uint8 typecommand);
void CMD_SystemReset(uint8 typecommand);
static uint8 CMD_Find_name(uint8 typecommand);
void CMD_Modify_name(uint8 *newname,uint8 typecommand);
void CMD_Find_ADVI(uint8 typecommand);
void CMD_Modify_ADVI( uint8 *new_advi,uint8 typecommand );
void CMD_Find_CONI(uint8 typecommand);
void CMD_Modify_CONI( uint8 *new_coni,uint8 typecommand );
void CMD_ParaRenew(uint8 typecommand);
void CMD_Find_MAC(uint8 typecommand);
void CMD_Find_Version(uint8 typecommand);
void update_Passkey();
void CMD_PASSKEY(uint8 *new_passkey,uint8 typecommand);
void CMD_Modify_POWER(uint8 *new_spwr,uint8 typecommand );
void  CMD_Find_POW(uint8 typecommand );
void CMD_Find_STATU(uint8 typecommand );
void CMD_Modify_ADVON(uint8 typecommand );
void CMD_Modify_ADVOFF(uint8 typecommand );
enum cmd_status
{
  STATUS_CMD_ERR,
  STATUS_CMD_NAME,
  STATUS_CMD_TEST,
  STATUS_FD_BPS,
  STATUS_CMD_BPS,
  STATUS_CMD_RST,
  STATUS_FD_NAME,
  STATUS_FD_ADVI,
  STATUS_CMD_ADVI,
  STATUS_FD_CONI,
  STATUS_CMD_CONI,
  STATUS_CMD_RENEW,
  STATUS_FD_MAC,
  STATUS_FD_VERSION,
  STATUS_CMD_PASSKEY,
  STATUS_CMD_POWER,
  STATUS_FD_POW,
  STATUS_FD_STATU,
  STATUS_CMD_ADVON,
  STATUS_CMD_ADVOFF,
};