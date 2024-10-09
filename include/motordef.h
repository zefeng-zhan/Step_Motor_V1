
#include "arduino.h"
#include <stdio.h>
#include "Step_Motor_V1.h"


//-------------add new motor here:
/*example: extern a motor and set the pin position (PLS and DIR, name it to _MX, _MX_DIR, _MX_PLS)
    extern Step_Motor MotorA;
    #define _MA MotorA
    #define _MA_DIR PORTC
    #define _MA_DIRp 6
    #define _MA_PLS PORTC
    #define _MA_PLSp 7
*/
extern Step_Motor MotorA;
#define _MA MotorA
#define _MA_DIR PORTC
#define _MA_DIRp 6
#define _MA_PLS PORTC
#define _MA_PLSp 7
/*
extern Step_Motor MotorB;
#define _MB MotorB
#define _MB_DIR PORT
#define _MB_DIRp 
#define _MB_PLS PORT
#define _MB_PLSp 

extern Step_Motor MotorC;
#define _MC MotorC
#define _MC_DIR PORT
#define _MC_DIRp 
#define _MC_PLS PORT
#define _MC_PLSp 

extern Step_Motor MotorD;
#define _MD MotorD
#define _MD_DIR PORT
#define _MD_DIRp 
#define _MD_PLS PORT
#define _MD_PLSp 

extern Step_Motor MotorE;
#define _ME MotorE
#define _ME_DIR PORT
#define _ME_DIRp 
#define _ME_PLS PORT
#define _ME_PLSp 
*/

//-------------add new motor here:

void _pulse_indv_new();
extern uint8_t interrupt_count_new;
#if defined(_MA) && defined(_MA_DIR) && defined(_MA_DIRp)&& defined(_MA_PLS)&& defined(_MA_PLSp)
  #define PULSE_MA_0_new if(!_MA.rbuffer_dn()){_MA.dn = 0;}bitWrite(_MA_PLS,_MA_PLSp,0);bitWrite(_MA_DIR,_MA_DIRp,_MA.dir_r_now);
  #define PULSE_MA_1_new bitWrite(_MA_PLS,_MA_PLSp,bitRead(interrupt_count_new*_MA.dn,7));
  #define PULSE_MA_255_new _MA.update_abspos(); if(_MA.dn>0)    bitWrite(_MA_PLS,_MA_PLSp,1);
#endif

#if defined(_MB) && defined(_MB_DIR) && defined(_MB_DIRp)&& defined(_MB_PLS)&& defined(_MB_PLSp)
    #define PULSE_MB_0_new if(!_MB.rbuffer_dn()){_MB.dn = 0;}bitWrite(_MB_PLS,_MB_PLSp,0);bitWrite(_MB_DIR,_MB_DIRp,_MB.dir_r_now);
    #define PULSE_MB_1_new bitWrite(_MB_PLS,_MB_PLSp,bitRead(interrupt_count_new*_MB.dn,7));
    #define PULSE_MB_255_new _MB.update_abspos(); if(_MB.dn>0)    bitWrite(_MB_PLS,_MB_PLSp,1);

#endif

#if defined(_MC) && defined(_MC_DIR) && defined(_MC_DIRp)&& defined(_MC_PLS)&& defined(_MC_PLSp)
    #define PULSE_MC_0_new if(!_MC.rbuffer_dn()){_MC.dn = 0;}bitWrite(_MC_PLS,_MC_PLSp,0);bitWrite(_MC_DIR,_MC_DIRp,_MC.dir_r_now);
    #define PULSE_MC_1_new bitWrite(_MC_PLS,_MC_PLSp,bitRead(interrupt_count_new*_MC.dn,7));
    #define PULSE_MC_255_new _MC.update_abspos(); if(_MC.dn>0)    bitWrite(_MC_PLS,_MC_PLSp,1);

#endif

#if defined(_MD) && defined(_MD_DIR) && defined(_MD_DIRp)&& defined(_MD_PLS)&& defined(_MD_PLSp)
  #define PULSE_MD_0_new if(!_MD.rbuffer_dn()){_MD.dn = 0;}bitWrite(_MD_PLS,_MD_PLSp,0);bitWrite(_MD_DIR,_MD_DIRp,_MD.dir_r_now);
  #define PULSE_MD_1_new bitWrite(_MD_PLS,_MD_PLSp,bitRead(interrupt_count_new*_MD.dn,7));
  #define PULSE_MD_255_new _MD.update_abspos(); if(_MD.dn>0)    bitWrite(_MD_PLS,_MD_PLSp,1);
#endif


#if defined(_ME) && defined(_ME_DIR) && defined(_ME_DIRp)&& defined(_ME_PLS)&& defined(_ME_PLSp)
  #define PULSE_ME_0_new if(!_ME.rbuffer_dn()){_ME.dn = 0;}bitWrite(_ME_PLS,_ME_PLSp,0);bitWrite(_ME_DIR,_ME_DIRp,_ME.dir_r_now);
  #define PULSE_ME_1_new bitWrite(_ME_PLS,_ME_PLSp,bitRead(interrupt_count_new*_ME.dn,7));
  #define PULSE_ME_255_new _ME.update_abspos(); if(_ME.dn>0)    bitWrite(_ME_PLS,_ME_PLSp,1);
#endif