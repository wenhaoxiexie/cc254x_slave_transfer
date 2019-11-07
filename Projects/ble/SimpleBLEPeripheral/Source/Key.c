//******************************************************************************            
//name:         Key.c          
//introduce:    ��������        
//******************************************************************************   
#include <ioCC2541.h>     
#include "Key.h"    
    
/*********************�궨��************************/    
//ע��ʱʹ�õĺ�    
#define NO_TASK_ID                      0xFF            //û��ע��ʱ������id    
#define NO_EVEN_ID                      0x0000          //û��ע��ʱ���¼�id    
    
//�ж�����ʱʹ�õĺ�    
#define KEY_DEBOUNCE_VALUE  10          //����ʱ��10ms    
    
#ifndef false    
#define false 0    
#endif    
    
#ifndef true    
#define true 1    
#endif    

 
typedef signed   char   int8;     //!< Signed 8 bit integer    
typedef unsigned char   uint8;    //!< Unsigned 8 bit integer    
    
typedef signed   short  int16;    //!< Signed 16 bit integer    
typedef unsigned short  uint16;   //!< Unsigned 16 bit integer    
    
typedef signed   long   int32;    //!< Signed 32 bit integer    
typedef unsigned long   uint32;   //!< Unsigned 32 bit integer    

    
/*********************�ڲ�����************************/    
static U8 registeredKeyTaskID = NO_TASK_ID;    
static U16 registeredKeyEvenID = NO_EVEN_ID;    
    
    
/*********************��������************************/    
extern uint8 osal_start_timerEx( uint8 task_id, uint16 event_id, uint32 timeout_value );    
    
    
//******************************************************************************      
//name:             Key_Init      
//introduce:        ������ʼ��    
//parameter:        none     
//return:           none            
//******************************************************************************    
void Key_Init(void)    
{     
    P2SEL &= ~(1 << 0); //P2.0����ΪIO��    
    P2DIR &= ~(1 << 0); //P2.0����Ϊ����    
    P2INP &= ~(1 << 0); //P2.0��������ģʽ        
    P2INP &= ~(1 << 7); //P2����    
    P2_0 = 1;           //P2.0����    
        
    P2IFG &= ~(1 << 0); //��ʼ��P2.0�жϱ�־λ    
    PICTL |= (1 << 3);  //�½��ش���     
    P2IEN |= (1 << 0);  //ʹ��P2.0�ж�      
    IEN2 |= (1 << 1);   //����P���ж�; 

}    
    
//******************************************************************************      
//name:             RegisterForKey     
//introduce:        ע������š������¼���    
//parameter:        task_id������id    
//                  even_id���¼�id    
//return:           true��ע��ɹ�    
//                  flase��ע�᲻�ɹ�                
//******************************************************************************    
U8 RegisterForKey(U8 task_id, U16 even_id)    
{    
  // Allow only the first task    
  if ( registeredKeyTaskID == NO_TASK_ID )    
  {    
    registeredKeyTaskID = task_id;    
  }    
  else    
    return ( false );    
      
      
  // Allow only the first even    
  if ( registeredKeyEvenID == NO_EVEN_ID )    
  {    
    registeredKeyEvenID = even_id;    
  }    
  else    
    return ( false );    
      
      
  return ( true );      
}    
    
//******************************************************************************      
//name:             Key_Check_Pin      
//introduce:        �������ߵ͵�ƽ״̬    
//parameter:        none     
//return:           KEY_LOOSEN����ʱ�ް�������    
//                  KEY_PRESS����ʱ��������               
//******************************************************************************    
U8 Key_Check_Pin(void)    
{    
  if(P2 & (1 << 0))    
  {    
    return KEY_LOOSEN;    
  }    
  else    
  {    
    return KEY_PRESS;     
  }    
}    
 
//******************************************************************************      
//name:             P2_ISR      
//introduce:        P2���ж����     
//parameter:        none     
//return:           none                
//******************************************************************************
#if defined(GPIO_2_0)
#pragma vector = P2INT_VECTOR        
__interrupt void P2_ISR(void)     
{  

 //   if(Key_Check_Pin() == KEY_PRESS)     
 //   {    
      osal_start_timerEx(registeredKeyTaskID, registeredKeyEvenID, KEY_DEBOUNCE_VALUE);    
  //  } 

     P2IFG = 0;       //���жϱ�־     
     P2IF = 0;        //���жϱ�־    
  
}

#endif

