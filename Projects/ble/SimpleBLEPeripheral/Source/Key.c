//******************************************************************************            
//name:         Key.c          
//introduce:    按键驱动        
//******************************************************************************   
#include <ioCC2541.h>     
#include "Key.h"    
    
/*********************宏定义************************/    
//注册时使用的宏    
#define NO_TASK_ID                      0xFF            //没有注册时的任务id    
#define NO_EVEN_ID                      0x0000          //没有注册时的事件id    
    
//中断消抖时使用的宏    
#define KEY_DEBOUNCE_VALUE  10          //消抖时间10ms    
    
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

    
/*********************内部变量************************/    
static U8 registeredKeyTaskID = NO_TASK_ID;    
static U16 registeredKeyEvenID = NO_EVEN_ID;    
    
    
/*********************函数声明************************/    
extern uint8 osal_start_timerEx( uint8 task_id, uint16 event_id, uint32 timeout_value );    
    
    
//******************************************************************************      
//name:             Key_Init      
//introduce:        按键初始化    
//parameter:        none     
//return:           none            
//******************************************************************************    
void Key_Init(void)    
{     
    P2SEL &= ~(1 << 0); //P2.0设置为IO口    
    P2DIR &= ~(1 << 0); //P2.0设置为输入    
    P2INP &= ~(1 << 0); //P2.0上拉下拉模式        
    P2INP &= ~(1 << 7); //P2上拉    
    P2_0 = 1;           //P2.0拉高    
        
    P2IFG &= ~(1 << 0); //初始化P2.0中断标志位    
    PICTL |= (1 << 3);  //下降沿触发     
    P2IEN |= (1 << 0);  //使能P2.0中断      
    IEN2 |= (1 << 1);   //允许P口中断; 

}    
    
//******************************************************************************      
//name:             RegisterForKey     
//introduce:        注册任务号、处理事件号    
//parameter:        task_id：任务id    
//                  even_id：事件id    
//return:           true：注册成功    
//                  flase：注册不成功                
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
//introduce:        按键检测高低电平状态    
//parameter:        none     
//return:           KEY_LOOSEN：此时无按键按下    
//                  KEY_PRESS：此时按键按下               
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
//introduce:        P2的中断入口     
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

     P2IFG = 0;       //清中断标志     
     P2IF = 0;        //清中断标志    
  
}

#endif

