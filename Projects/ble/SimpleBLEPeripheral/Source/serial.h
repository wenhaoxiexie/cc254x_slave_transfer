/*********************************************************************
 * EXTERNAL VARIABLES DEFINE
 */
#ifdef __cplusplus
extern "C"
{
#endif
  
extern uint8 SerialRxBuff[200]; //���ڽ������ݻ���
extern uint8 *Rx_q;//ָ�򴮿ڻ�����
extern uint8 uart_remain_len;//��������ʣ�೤��
extern uint8 offs_RxBuf;
extern uint8 uarting;
/*********************************************************************
 * FUNCTIONS
 */
void NpiSerialCallback(uint8 port, uint8 event);
extern void SC_InitTransport( npiCBack_t npiCBack,uint8 current_BD);