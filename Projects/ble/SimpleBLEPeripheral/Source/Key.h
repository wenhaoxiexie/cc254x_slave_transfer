//******************************************************************************            
//name:         Key.h          
//introduce:    ����������ͷ�ļ�        
//author:       ����Ĵ����          
//changetime:   2016.04.14        
//email:        897503845@qq.com        
//******************************************************************************   
#ifndef KEY_H  
#define KEY_H  
  
/*********************�궨��************************/  
#ifndef U8  
typedef unsigned char U8;  
#endif  
  
#ifndef U16  
typedef unsigned short U16;  
#endif  
  
//���io��״̬ʱʹ�õĺ�  
#define KEY_LOOSEN                      0x01              
#define KEY_PRESS                       0x00  
  
  
/*********************��������************************/  
extern void Key_Init(void);  
extern U8 RegisterForKey(U8 task_id, U16 even_id);  
extern U8 Key_Check_Pin(void);  
  
#endif  
