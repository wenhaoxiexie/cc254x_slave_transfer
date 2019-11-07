//******************************************************************************            
//name:         Key.h          
//introduce:    按键驱动的头文件        
//author:       甜甜的大香瓜          
//changetime:   2016.04.14        
//email:        897503845@qq.com        
//******************************************************************************   
#ifndef KEY_H  
#define KEY_H  
  
/*********************宏定义************************/  
#ifndef U8  
typedef unsigned char U8;  
#endif  
  
#ifndef U16  
typedef unsigned short U16;  
#endif  
  
//检测io口状态时使用的宏  
#define KEY_LOOSEN                      0x01              
#define KEY_PRESS                       0x00  
  
  
/*********************函数声明************************/  
extern void Key_Init(void);  
extern U8 RegisterForKey(U8 task_id, U16 even_id);  
extern U8 Key_Check_Pin(void);  
  
#endif  
