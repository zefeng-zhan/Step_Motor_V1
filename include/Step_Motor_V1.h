#ifndef STEP_MOTOR_V1

#define STEP_MOTOR_V1

#include "arduino.h"
#include <stdio.h>
#define Buffer_Size 32


class Step_Motor{
  private:
  //初始化參數，pin腳位
    volatile uint8_t * DIR;
    volatile uint8_t * PLS;
    volatile uint8_t * SEN;
    volatile uint8_t * HOME;
    bool has_home_sensor;
    bool has_home_pin;
    uint8_t pinDIR;
    uint8_t pinPLS;
    uint8_t pinSEN;
    uint8_t pinHOME;
  //0.005s內使用參數 (interrupt內使用)  
    uint8_t inter_count;
    //uint8_t _n_sub;
    //uint16_t next_pulse;
    uint16_t t_5ms;    
    //uint8_t dn;
    
      uint16_t stepcount_interrupt;
  //計算dn使用參數
    float ddn;
    float f_dn; //in test: name test_D
    float f_n;
    int16_t n_pre;
    int16_t n_this;
    int8_t dn_temp;
    uint16_t T_target;  //加速度運行時間
    uint16_t T_constV;  //等速度運行時間長度，超過這個時間(T_target + T_constV)就開始減速
  //填入dn_buf使用參數  
    int8_t dn_buf[Buffer_Size];
    volatile uint8_t dn_buf_Pw;
    volatile uint8_t dn_buf_Pr;
    uint16_t stepcount_buf;  //紀錄目前已經寫入的量，使用方式為到數
    //bool start_wbuf; //move 時會打開，開始寫buf，判斷dn累加(f_n)超過該走距離stepcount_buf，就會關掉 (只有一個地方打開，一個地方關掉)
    uint16_t t_5ms_buf;  //加速度模式下，計算目前記錄時間，判斷該加速等速還是減速
    uint16_t t_5ms_accumulate;
  //外部傳入參數
    uint16_t stepcount;  //這次move要移動的step數量，使用方式為到數
    uint16_t _V0;  //in xv mode, means V0. in xva mode, means V target
    //uint16_t _Accel;  //改成只用float的加速度，因為要計算精確的ddn，才有精確的位移和加速度
    float f_Accel;
    bool use_accel; //有無加速度，攸關dn計算
    bool td_check;  //當加速度無法達到預設速度，會開啟，然後累加寫入時會檢查，檢查到就開始減速
    
    //bool isMultimotor;  //多軸移動flag
    bool xt_mode;
    
    //uint16_t Time_input;
    
    uint16_t abs_steppos;  //馬達絕對位置，假定最低點為0，也為home點，不另外設定min，所以不會有-的位置    
    uint16_t abs_maxpos;  //馬達位置上限
    bool Dir_dir;


/*
    uint16_t test_n_send;
    uint16_t test_cannot_read;    
    uint16_t test_cannot_write;  
    uint16_t test_read_index[300];
    uint8_t test_write_index[300];
    uint8_t test_cannot_write_buf[300];
//*/
    
  public:
    //bool ishoming;
    bool isHomed;
    //bool Start_Move;
    bool continue_mode;

  //拿到public的參數
    volatile uint8_t dn;
    //uint16_t abs_steppos;//馬達絕對位置，假定最低點為0，也為home點，不另外設定min，所以不會有-的位置    
    //int8_t movedir; //紀錄移動的方向，用於計算絕對位置

    bool dir_w_now; //1 positive, 0 negative
    bool dir_r_now; //0 positive, 1 negative
    
    bool multimotor_zero;


    uint16_t test_cannot_read;
//-----------------------------functions------------------------------    
    Step_Motor(volatile uint8_t * dir,uint8_t pdir,volatile uint8_t * pls,uint8_t ppls,volatile uint8_t * sen,uint8_t psen);    //初始化dir pls sensor腳位
    Step_Motor(volatile uint8_t * dir,uint8_t pdir,volatile uint8_t * pls,uint8_t ppls); //初始化 dir pls 腳位
    Step_Motor(volatile uint8_t * dir,uint8_t pdir,volatile uint8_t * pls,uint8_t ppls,volatile uint8_t * sen,uint8_t psen,bool dirposneg);    //加上+-方向不同馬達的修正參數 (物理上的方向不同)
    Step_Motor(volatile uint8_t * dir,uint8_t pdir,volatile uint8_t * pls,uint8_t ppls,bool dirposneg); //初始化 dir pls 腳位
    Step_Motor(volatile uint8_t * dir,uint8_t pdir,volatile uint8_t * pls,uint8_t ppls,bool a,volatile uint8_t * home,uint8_t phome);
    void initial_class();    
    bool homing();
  

  //單馬達 移動
    void move_continue(int v);  //不停止的移動，有時用於homing
    void move_continue_accel();//加速度測試
    void continue_stop(); //停止連續移動
    
    bool move_motor(int16_t x);  //以特定速度等速移動x
    bool move_motor(int16_t x, int16_t v);    //以指定速度移動x
    bool move_motor(int16_t x, int16_t v, float a); //移動x，初始速度0，以a加速到v，然後相同a減速至停止

    void move_to(int abs_x); //移動至絕對位置
    void move_to(int abs_x, int v);
    void move_to(int abs_x, int v, int a); 
  //多馬達xt 同時但獨立 軌跡
    bool move_x(int x); 
    bool move_xt(int x, uint16_t t);    //跑軌跡專用
    bool move_xat(int x, uint16_t t,int next_x, uint16_t next_t, float a);    //用此加速度連貫軌跡頭尾
    //void move_and_count(int x, int v, int a, int * count);  //高速跑軌跡需加減速，但時尚位想出如何寫    

  
  //多馬達 分量同步
    bool move_by_f(float f, bool is_start,bool abs_start);

//--------------------換算-----------------------
    int x_dir(int16_t x);
    int x2step(int16_t x);
    uint16_t v2pps(int16_t v);    
    float a2vps(float a);

    int step2x(int steppos);

//-------------------參數確認---------------------
    void update_abspos();
    //回傳 private 的 abs_pos
    uint16_t now_pos();
    //檢查可不可以下move指令(caldn沒有正在寫入dn了)，可以的話回傳true
    bool check_start();
    //強制把buf清掉 (作法是把兩個pointer重疊就可，read會停下，write若繼續寫會蓋掉原本資料，達到清掉目的)
    void set_start();
    bool check_t5ms();

    /*
     * 
    */
//----------------------------內部計算、填入buffer、內部操作函數---------------------------
    //0319 改
    void cal_dn_write();
    void wbuffer_dn(int8_t in);
    void wbuffer_dn();
    bool rbuffer_dn();

    //void Set_as_Multimotor();
    //void MultiMotor_start();
//----------------------中斷之中執行---------------------    
    void _pulse(); //在 interrupt 裡面發送脈波  //bool改void

//--------------------額外進階功能設想-----------------
    void setMax();  //設定馬達位置上限，機構有上限位置時須設定
/*
 * void change_X(int x);
 * void change_V(int v);
 * void change_A(int a);
    
*/    
    void EMG_Stop();
    void EMG_Stop_decelerate();
    
};


#endif