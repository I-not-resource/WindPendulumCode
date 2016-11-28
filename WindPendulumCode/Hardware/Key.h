#ifndef __KEY_H_
#define __KEY_H_
#include "Sys.h"
#if __KEY_ENABLE

#define POW_KEY PAout(8)    //电源维持开关
#define WK_UP   PAin(0)
#define BEEP    PFout(6)

#define KEYNUM   33         //电压一个阶级的范围
#define KEYAREA  15         //为了方便计算，上调数值

#define OFF 0
#define ON  1

#define Base1_Fuction()  BF1_Show()
#define Base2_Fuction()  BF2_Show()
#define Base3_Fuction()  BF3_Show()
#define Base4_Fuction()  BF4_Show()

#define Improve1_Function()  IF1_Show()
#define Improve2_Function()  IF2_Show()
#define Improve3_Function()  IF3_Show()
#define Improve4_Function()  IF4_Show()

extern u8 Key,
          _KeyRead,
	        mc,
	        Mode,
					side;
extern u8 MoveLive,
          MoveLiveB,
          MEMU_SURE,
	        BACK_ESC;

typedef enum                //功能
{
  BaseFiction_1=1,
	BaseFiction_2,
	BaseFiction_3,
	BaseFiction_4,
	ImproveFiction_1,
	ImproveFiction_2,
	ImproveFiction_3,
	ImproveFiction_4,
}_ModeFuction;

void Key_Config(void);
void Keyopt(u16 _k);
void Fuction_choose(void);


#endif  //__KEY_ENABLE
#endif  //__KEY_H_
