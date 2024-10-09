
#include "motordef.h"




uint8_t interrupt_count_new = 0;
void _pulse_indv_new(){
  if(interrupt_count_new == 0){
    #ifdef _MA
      PULSE_MA_0_new
    #endif
    #ifdef _MB
      PULSE_MA_0_new
    #endif
    #ifdef _MC
      PULSE_MA_0_new
    #endif
    #ifdef _MD
      PULSE_MA_0_new
    #endif
    #ifdef _ME
      PULSE_MA_0_new
    #endif
  }
  else if(interrupt_count_new == 255){
    #ifdef _MA
      PULSE_MA_255_new
    #endif
    #ifdef _MB
      PULSE_MB_255_new
    #endif
    #ifdef _MC
      PULSE_MC_255_new
    #endif
    #ifdef _MD
      PULSE_MD_255_new
    #endif
    #ifdef _ME
      PULSE_ME_255_new
    #endif
    
  }
  else{
    #ifdef _MA
      PULSE_MA_1_new
    #endif
    #ifdef _MB
      PULSE_MB_1_new
    #endif
    #ifdef _MC
      PULSE_MC_1_new
    #endif
    #ifdef _MD
      PULSE_MD_1_new
    #endif
    #ifdef _ME
      PULSE_ME_1_new
    #endif
    
  }  
  interrupt_count_new++;
}