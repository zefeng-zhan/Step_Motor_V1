#include "Step_Motor_V1.h"
#include "arduino.h"


Step_Motor::Step_Motor(volatile uint8_t * dir,uint8_t pdir,volatile uint8_t * pls,uint8_t ppls,volatile uint8_t * sen,uint8_t psen){
  DIR = dir; pinDIR = pdir;
  PLS = pls; pinPLS = ppls;
  SEN = sen; pinSEN = psen;
  //#define pulseH bitWrite(*PLS,pinDIR,1)
  //#define pulseL bitWrite(*PLS,pinDIR,0) 
  //#define readSen bitRead(*SEN,pinSEN)
  
  initial_class();
  has_home_sensor = true;
  has_home_pin = false;
  isHomed = false;
  Dir_dir = true;
}
Step_Motor::Step_Motor(volatile uint8_t * dir,uint8_t pdir,volatile uint8_t * pls,uint8_t ppls){
  DIR = dir; pinDIR = pdir;
  PLS = pls; pinPLS = ppls;
  
  initial_class();
  has_home_sensor = false;
  has_home_pin = false;
  isHomed = true;
  Dir_dir = true;
}
Step_Motor::Step_Motor(volatile uint8_t * dir,uint8_t pdir,volatile uint8_t * pls,uint8_t ppls,bool a,volatile uint8_t * home,uint8_t phome){
  DIR = dir; pinDIR = pdir;
  PLS = pls; pinPLS = ppls;
  HOME = home; pinHOME = phome;

  initial_class();
  has_home_sensor = false;
  has_home_pin = true;
  isHomed = true;
  Dir_dir = true;
}
Step_Motor::Step_Motor(volatile uint8_t * dir,uint8_t pdir,volatile uint8_t * pls,uint8_t ppls,volatile uint8_t * sen,uint8_t psen,bool dirposneg){
  DIR = dir; pinDIR = pdir;
  PLS = pls; pinPLS = ppls;
  SEN = sen; pinSEN = psen;
  
  //#define pulseH bitWrite(*PLS,pinDIR,1)
  //#define pulseL bitWrite(*PLS,pinDIR,0) 
  //#define readSen bitRead(*SEN,pinSEN)
  initial_class();
  has_home_sensor = true;
  has_home_pin = false;
  isHomed = false;
  Dir_dir = dirposneg;
}
Step_Motor::Step_Motor(volatile uint8_t * dir,uint8_t pdir,volatile uint8_t * pls,uint8_t ppls,bool dirposneg){
  DIR = dir; pinDIR = pdir;
  PLS = pls; pinPLS = ppls;
  initial_class();
  has_home_sensor = false;
  has_home_pin = false;
  isHomed = true;
  Dir_dir = dirposneg;
}

void Step_Motor::initial_class(){
  has_home_sensor = false;
  has_home_pin = false;
  isHomed = false;
  //ishoming = false;
  //Start_Move = false;
  continue_mode = false;


  //0.005s內使用參數 (interrupt內使用)  
    inter_count = 0;
    //_n_sub = 0;
    t_5ms = 0;
    dn = 0;

  //計算dn使用參數
    ddn = 0;
    f_dn = 0; //in test: name test_D
    f_n = 0;
    n_pre = 0;
    n_this = 0;
    T_target = 0;  //加速度運行時間
    T_constV = 0;  //等速度運行時間長度，超過這個時間(T_target + T_constV)就開始減速
  //填入dn_buf使用參數  
    //dn_buf[16];
    //dn_buf_P = 0;
    dn_buf_Pw = 0;
    dn_buf_Pr = 0;
    stepcount_buf = 0;  //紀錄目前已經寫入的量
    //start_wbuf = false; //move 時會打開，開始寫buf，判斷dn累加(f_n)超過該走距離stepcount_buf，就會關掉 (只有一個地方打開，一個地方關掉)
    t_5ms_buf = 0;  //加速度模式下，計算目前記錄時間，判斷該加速等速還是減速
  //外部傳入參數
    stepcount = 0;
    _V0 = 0;  //in xv mode, means V0. in xva mode, means V target
    //uint16_t _Accel;  //改成只用float的加速度，因為要計算精確的ddn，才有精確的位移和加速度
    f_Accel = 0;
    use_accel = false; //有無加速度，攸關dn計算
    td_check = false;
    //movedir = 1;
    xt_mode = false;

    dir_w_now = true;
    dir_r_now = Dir_dir;
    
  abs_steppos = 0;
  stepcount = 0;

    multimotor_zero = false;

    test_cannot_read = 0;
  //測試參數
/*
  test_n_send = 0;
  test_cannot_read = 0;
  test_cannot_write = 0;
//*/
}
/*
bool Step_Motor::homing(){
  
  move_continue(1000);
  ishoming = true;//start homing mode in move continue
  while(ishoming){  //正在homing不能做其他事情 //目前此寫法只考慮一次只進行單軸homing。如果要多軸同時homeing，要如何做?
    if(bitRead(*SEN,pinSEN)){
      continue_stop();
      ishoming = false;
      return true;
    }
  }
  
}
*/
void Step_Motor::move_continue(int v){

}
void Step_Motor::continue_stop(){
  
}
//-------------單次move-----------------
bool Step_Motor::move_motor(int16_t x){  //以特定速度等速移動x
  return move_motor(x, 1000);
}
bool Step_Motor::move_motor(int16_t x, int16_t v){    //以指定速度移動x
  if(stepcount_buf>0 || dn_buf_Pr == ((dn_buf_Pw +1)% Buffer_Size)){return false;}//上次還沒處理完或是buffer無法寫入，就先等等
  else{
    xt_mode = false;
    Serial.println("move_motor(int x, int v)-------------------------------------");
    ddn = 0;
    f_dn = 0; //in test: name test_D
    f_n = 0;
    n_this = 0;
    n_pre = 0;
    
    stepcount = x2step(x);
    stepcount_interrupt = 0;
    //stepcount_interrupt = stepcount;  //移動到這裡的最下面，這條是啟動interrupt條件
    stepcount_buf = stepcount;
    _V0 = v2pps(v);

Serial.print("stepcount_buf in set move is:");
Serial.println(stepcount_buf);

    ddn = 0;//no acceleration
    f_dn = (float)_V0/200;
    
    f_n += f_dn; //or = f_dn
    n_this = f_n;
    dn_temp = n_this; //n_this - n_pre
        
//test_write_index[t_5ms_buf] = dn_buf_Pw % Buffer_Size;  //測試寫入的index，當初判斷停址寫入條件有誤，判斷到底哪裡寫入覆蓋

    stepcount_buf -= dn_temp;
    t_5ms_buf = 1;


    if(dir_w_now)
      wbuffer_dn(dn_temp);
    else
      wbuffer_dn(-dn_temp);
    return true;
  }
}
bool Step_Motor::move_motor(int16_t x, int16_t v, float a){ //移動x，初始速度0，以a加速到v，然後相同a減速至停止 //a引數用float，是因為加速度用16位整數不夠表示，所以直接用float(32位)
  if(stepcount_buf>0|| dn_buf_Pr == ((dn_buf_Pw +1)% Buffer_Size)){return false;}
  else{
    if(x==0||v==0||a==0){
      return false;
    }
    /*
    if(x == 0){
      multimotor_zero = true;
    }
    else{
      
    }
    */
    xt_mode = false;
    Serial.println("move_motor(int x, int v,int a)-------------------------------------");
    ddn = 0;
    f_dn = 0; //in test: name test_D 
    f_n = 0;
    n_this = 0;
    n_pre = 0;

    stepcount = x2step(x);
    stepcount_interrupt = 0;
    //stepcount_interrupt = stepcount;  //移動到這裡的最下面，這條是啟動interrupt條件
    stepcount_buf = stepcount;
    _V0 = v2pps(v);
    f_Accel = a2vps(a);

  //加速時間先處理，然後算出約化後的新a (因應T_target必須是整數的a)
    T_target = _V0/f_Accel*200; //200是單位換算，換算成5ms
    f_Accel = _V0/T_target*200; //200是單位換算，換算成5ms
    T_constV = ceil(stepcount - f_Accel*T_target/200*T_target/200)*200/_V0+1;//推測+1較為合理，不然會有瞬間爆速，因為還沒到位子就被減速到0，剩下的距離演算法會自動一步到達，瞬間高速

    if(stepcount>=f_Accel*T_target/200*T_target/200 ){      
    }
    else{
      T_target = 0;T_constV = 0;
      td_check = true;
    }
    
    ddn = (float)f_Accel/40000;//at2
    f_dn = 0.5*ddn;//0.5at2
    use_accel = true;
    
    f_n += f_dn; //or = f_dn
    n_this = f_n;
    dn_temp = n_this; //n_this - n_pre


    
    stepcount_buf -= dn_temp;
    t_5ms_buf = 1;

    if(dir_w_now)
      wbuffer_dn(dn_temp);
    else
      wbuffer_dn(-dn_temp);
    return true;
  }
}
void Step_Motor::move_continue_accel(){  //等加速度運動，不斷加速，沒有減速停下，僅能做較小加速測試
  if(stepcount_buf>0){}
  else{//馬達沒有在進行才可以進來下新的move指令
    xt_mode = false;
    Serial.println("move_motor(int x, int v)-------------------------------------");
    ddn = 0;
    f_dn = 0; //in test: name test_D
    f_n = 0;
    n_this = 0;
    n_pre = 0;
    
stepcount = x2step(-5000); //random large number
    stepcount_interrupt = 0;  //可省略掉，重複確認歸零
    //stepcount_interrupt = stepcount;  //移動到這裡的最下面，這條是啟動interrupt條件
    stepcount_buf = stepcount;
_V0 = 8000;
f_Accel = 40000.0;


    T_target = _V0/f_Accel*200; //200是單位換算，換算成5ms
    f_Accel = _V0/T_target*200; //200是單位換算，換算成5ms
    T_constV = ceil(stepcount - f_Accel*T_target/200*T_target/200)*200/_V0+1;//推測+1較為合理，不然會有瞬間爆速，因為還沒到位子就被減速到0，剩下的距離演算法會自動一步到達，瞬間高速
    if(stepcount>=f_Accel*T_target/200*T_target/200 ){      
    }
    else{
      T_target = 0;T_constV = 0;
      td_check = true;
    }


    ddn = (float)f_Accel/40000;//at2
    f_dn = 0.5*ddn;//0.5at2
    use_accel = true;
    /*
    ddn = 1;//at2
    f_dn = 0.5*ddn;//0.5at2
    T_target = 60;//0.005*60 = 0.3s，1800steps
    T_constV = 42;// 不可以讓減速到零的時候還沒到位，這樣出現負的位移，但是dn又必須是正的就會變成一堆255，爆速度
    use_accel = true;
    */


    Serial.print("T_target is ----------");
    Serial.println(T_target);
    Serial.print("T_constV is ----------");
    Serial.println(T_constV);
    Serial.print("ddn is ----------");
    Serial.println(ddn);
        
    f_n += f_dn; //or = f_dn
    n_this = f_n;
    dn_temp = n_this; //n_this - n_pre


    wbuffer_dn(dn_temp);
    
    stepcount_buf -= dn_temp;
    t_5ms_buf = 1;

//test_write_index[t_5ms_buf] = dn_temp;  //這是當初要測試哪個dn沒有被讀取到，所以把所有寫入的dn存起來，一一讀出，然後讀取的dn也一一存取，兩者比對，發現write寫滿的判斷條件有誤
  }
}
bool Step_Motor::move_xt(int x, uint16_t t){
  if(t_5ms_accumulate>0 || dn_buf_Pr == ((dn_buf_Pw +1)% Buffer_Size)){
    Serial.println("set move false!!");
    return false;}//前段move有時間還沒處理完成，就不能下move
  else{
    /*
    if(dn_buf_Pr == ((dn_buf_Pw +1)% Buffer_Size)){
      //Serial.println("cannot write");
      return false;
    } 
    */ 
    if(t == 1){
      wbuffer_dn(x2step(x));
      return true;
    }
    xt_mode = true;
    /*
    if(check_start()){      //index相等，代表馬達是停下的，表示上次的軌跡跑完了 (所以要確保馬達不會剛好在處理move時碰到停下)
      t_5ms_buf = 0;
      //cli(); //關掉timer，要等確定buffer有東西，才可以打開，然後多馬達會同時開始
      //new move start
    }
    */
    ddn = 0;
    f_dn = 0; 
    f_n = 0;
    n_this = 0;
    n_pre = 0;

    stepcount = x2step(x);
    //stepcount_interrupt = 0;
    //stepcount_interrupt = stepcount;  //移動到這裡的最下面，這條是啟動interrupt條件
    stepcount_buf = stepcount;
   
    f_dn = (float)stepcount/t;//ceil((float)stepcount/t);   
    //_V0 = f_dn*200;  
    //f_dn = _V0/200; f_dn 就是 _V0，如果V的單位正確，不需要任何運算 (V以0.005秒為單位)
    t_5ms_accumulate = t;
    t_5ms_buf = 0;
//Serial.print("t_5ms_accumulate in set move is:");
//Serial.println(t_5ms_accumulate);
    ddn = 0;//no acceleration
    
    
    f_n += f_dn; //or = f_dn
    n_this = f_n;
    dn_temp = n_this; //n_this - n_pre

    if(dir_w_now)
      wbuffer_dn(dn_temp);

      
    else
      wbuffer_dn(-dn_temp);

    
//test_write_index[t_5ms_buf] = dn_buf_Pw % Buffer_Size;  //測試寫入的index，當初判斷停址寫入條件有誤，判斷到底哪裡寫入覆蓋

    stepcount_buf -= dn_temp;
    t_5ms_buf += 1;
    t_5ms_accumulate -= 1;
    //if(!isMultimotor || stepcount_interrupt>0)
      //stepcount_interrupt += stepcount;
    
  }
  return true;
}
bool Step_Motor::move_xat(int x, uint16_t t,int next_x, uint16_t next_t, float a){
  
}
int Step_Motor::x_dir(int16_t x){
  if(x>=0){
    //bitWrite(*DIR,pinDIR,Dir_dir);
    //movedir = 1;
    dir_w_now = true;
    return x; 
  }  
  //bitWrite(*DIR,pinDIR,!Dir_dir);
  //movedir = -1;
  dir_w_now = false;
  return -x;
}
int Step_Motor::x2step(int16_t x){
  x = x_dir(x);

  return x;
}
uint16_t Step_Motor::v2pps(int16_t v){
  //加入+-判斷，目前版本不寫入-的速度，所以看到-的要轉回正的
  return v;
}
float Step_Motor::a2vps(float a){
  //加入+-判斷，目前版本不寫入-的加速度，所以看到-的要轉回正的
  return a;
}


void Step_Motor::cal_dn_write(){
  if(!xt_mode){  //原本版本，非使用時間 (時間法用在類似我以前機器人軌跡)
    if(stepcount_buf>0){
      if(dn_buf_Pr == (dn_buf_Pw +1)% Buffer_Size){
        return;
      }
      if(use_accel){    // use _Accel == 0 ?
        if(t_5ms_buf == T_target){
          f_dn += 0.5*ddn; 
          ddn = 0;
        }          
        else if(t_5ms_buf == T_target+T_constV){
          ddn = -f_Accel/40000;  //-at平方
          f_dn -= 0.5*ddn; 
        }      
        else if(td_check && stepcount_buf<stepcount/2){
          f_dn += ddn;  //先加一個，然後會被減掉，這樣才會對稱
          ddn = -ddn;
          td_check = false;    
        }
      }
      
      n_pre = n_this;
      f_dn += ddn;  
      f_n += f_dn; 
      n_this = f_n; //or = f_dn
      dn_temp = n_this - n_pre; //or n_this        
      if(stepcount_buf<dn_temp && stepcount_buf>=0)
        dn_temp = stepcount_buf;
      
      if(dir_w_now)
        wbuffer_dn(dn_temp);
      else
        wbuffer_dn(-dn_temp);
//Serial.println(dn_temp);        
      stepcount_buf -= dn_temp;//算已經寫入多少距離，寫完就不用寫了
      t_5ms_buf ++;
      if(stepcount_buf<=0){ //把所有dn算完了，本次move的運算結束
        Serial.println("stop write-------------------------------------");
        use_accel = false;
    
        return;
      }
    }
  }
  else{  //用時間判斷是否該寫入
    if(t_5ms_accumulate>0){      
      if(dn_buf_Pr == ((dn_buf_Pw +1)% Buffer_Size)){
        //Serial.println("cannot write");
        return;
      }            
      
      n_pre = n_this;
      
      f_dn += ddn;  
      f_n += f_dn; 
      n_this = f_n; //or = f_dn
      dn_temp = n_this - n_pre; //or n_this     
      
      if(stepcount_buf<dn_temp && stepcount_buf>=0)
        dn_temp = stepcount_buf;
      else if(t_5ms_accumulate == 1 && stepcount_buf>dn_temp){ //剩下太多步，但時間要到了，該怎辦
        dn_temp = stepcount_buf;
      }
      
      if(dir_w_now)
        wbuffer_dn(dn_temp);
      else
        wbuffer_dn(-dn_temp);
//Serial.println(t_5ms_accumulate);       
//Serial.println(dn_temp);  

      stepcount_buf -= dn_temp;//算已經寫入多少距離，根據上述判斷，寫完就改寫0
      t_5ms_buf ++;
      t_5ms_accumulate --;
      
      if(t_5ms_accumulate<=0){ //把所有dn算完了，時間到，本次move的運算結束，等待下次move，讓下次move指令能下的條件
        //Serial.println("stop write-------------------------------------");
        //use_accel = false;
    
        return;
      }
      
    }
  }

}
void Step_Motor::wbuffer_dn(int8_t in){
  //if(dir_w_now){
    //Serial.print("+++++");
    dn_buf[dn_buf_Pw] = in; //寫入
  //}
    
  //else{
    //Serial.print("-----");
    //dn_buf[dn_buf_Pw] = -in; //寫入
    //Serial.print(dn_buf[dn_buf_Pw]);
  //}
    

  if(dn_buf_Pw>=Buffer_Size-1)
    dn_buf_Pw = 0;
  else
    dn_buf_Pw += 1;       //指標+1 
  
  //Serial.print(dn_buf_Pw);
  //Serial.print(",");
  //Serial.print(dn_buf_Pr);
  //Serial.print(",");
  //Serial.println(in);
}
void Step_Motor::wbuffer_dn(){  //0320 舊版，參考用，打算改掉不用
    if(stepcount_buf>0){//if(start_wbuf){  //這個條件要判斷：是否dn累積已經超過這次要move的距離? 會計算累計的距離，如果那次算出超過該距離，
      //Serial.println("start write-------------------------------------");
      if(dn_buf_Pr == (dn_buf_Pw +1)% Buffer_Size){  //0318 bug! 忘記+1後，還要再%16，不然1111+1 = 10000而不是0000
        //Serial.println("cannot write-------------------------------------");
        //Serial.println(dn_buf_Pw % Buffer_Size+1);
//test_cannot_write += 1;  //測試碰到多少次沒辦法寫入，可驗證寫入速度有無快過讀取
        return;} //寫滿先暫停不寫    
        
    //計算區---------------------------------------------------------
      if(use_accel){    // use _Accel == 0 ?
        if(t_5ms_buf == T_target){
          f_dn += 0.5*ddn; 
          ddn = 0;
        }          
        else if(t_5ms_buf == T_target+T_constV){
          ddn = -f_Accel/40000;  //-at平方
          f_dn -= 0.5*ddn; 
        }      
        else if(td_check && stepcount_buf<stepcount/2){
          f_dn += ddn;  //先加一個，然後會被減掉，這樣才會對稱
          ddn = -ddn;
          td_check = false;    
        }
      }
  
      n_pre = n_this;
      f_dn += ddn;  
      f_n += f_dn; 
      n_this = f_n; //or = f_dn
      dn_temp = n_this - n_pre; //or n_this    
            
      if(stepcount_buf<dn_temp && stepcount_buf>=0){
        dn_temp = stepcount_buf;
      }

      dn_buf[dn_buf_Pw % Buffer_Size] = dn_temp; //寫入
      dn_buf_Pw += 1;     //指標+1 

//test_write_index[t_5ms_buf] = dn_buf_Pr;  //測是這次寫入時，Pointer read 在哪裡，理論上不會差超過14
//test_cannot_write_buf[t_5ms_buf] = test_cannot_write; //測試寫入和讀取速度差異，當碰到越多不能寫入，代表寫入倒追到讀取，這個數字越大代表寫的速度越快
//test_write_index[t_5ms_buf] = dn_temp;  //測是這次寫的dn//這是當初要測試哪個dn沒有被讀取到，所以把所有寫入的dn存起來，一一讀出，然後讀取的dn也一一存取，兩者比對，發現write寫滿的判斷條件有誤
//Serial.print("write dn is-------------------------------------");
//Serial.println(dn_temp);      
      stepcount_buf -= dn_temp;//算已經寫入多少距離，寫完就不用寫了
      t_5ms_buf ++;
      if(stepcount_buf<=0){ //把所有dn算完了，本次move的運算結束
        Serial.println("stop write-------------------------------------");
        //start_wbuf = false;
        use_accel = false;
    //這裡要加一個條件，這個條件會使得最後一個dn填寫完成後，進行下一個move運算，用在跑軌跡，跑軌跡會連續下很多move。這樣發送下個move的時間有0.005秒，
        return;
      }
    }
}
bool Step_Motor::rbuffer_dn(){//read buffer，
  if(dn_buf_Pr== dn_buf_Pw){
    return false;
  }
  //dn = dn_buf[dn_buf_Pr];

  if(dn_buf[dn_buf_Pr]>=0){
    dir_r_now = Dir_dir;
    dn = dn_buf[dn_buf_Pr];
  }
    
  else{
    dir_r_now = !Dir_dir;
    dn = -dn_buf[dn_buf_Pr];
  }

  if(dn_buf_Pr>=Buffer_Size-1)
    dn_buf_Pr = 0;
  else
    dn_buf_Pr += 1;       //指標+1 
  return true;  
}
void Step_Motor::_pulse() {  //(待修改)若系統只有單馬達可用，多馬達就算單獨跑也不可用
  //if(stepcount_interrupt>0){  //改成 if(t_5ms<t_target)? 
    if(inter_count == 0){
      if(!rbuffer_dn()) {
        //Serial.print("no dn exist-------------------------------------");
        //Serial.println(inter_count);
test_cannot_read+=1;  //測試有無碰到buffer是空的情況，這種情況是不允許發生的
        //return false; // read new dn      
      }
//test_read_index[t_5ms] = stepcount_interrupt;  //測試每次0.005進行時，每次剩餘步數多少，也可看出每一個dn是多少
      //t_5ms ++;
      
      //_n_sub = 1;  //for method2
      
      bitWrite(*PLS,pinPLS,0);            
    }
    else if(inter_count == 255){
      abs_steppos += dn;//*movedir;  //絕對位置更新 //不確定是否正確
      //stepcount_interrupt = stepcount_interrupt - dn;  //這次move剩下的step
      if(dn>0)    
        bitWrite(*PLS,pinPLS,1);
      
      //return;
    }
    else{
      //dn:5 555
      if(bitRead(inter_count*dn,7)){
        bitWrite(*PLS,pinPLS,1);
      }
      else{
        bitWrite(*PLS,pinPLS,0);
      }        
    }
    inter_count ++;
  //}
}
bool Step_Motor::check_start(){//檢查可不可以下move指令(caldn沒有正在寫入dn了)，可以的話回傳true
  if(dn_buf_Pr == dn_buf_Pw){
    return true;
  }
  //Serial.println("cannot start!!");  
  return false;
}
void Step_Motor::set_start(){//強制把buf清掉 (作法是把兩個pointer重疊就可，read會停下，write若繼續寫會蓋掉原本資料，達到清掉目的)
  if(!check_start()){
    dn_buf_Pr = 0;
    dn_buf_Pw = 0;
    Serial.print("set start");
  }
  
}
bool Step_Motor::check_t5ms(){
  //Serial.print("t_5ms_accumulate is ");
  //Serial.println(t_5ms_accumulate);
  if(t_5ms_accumulate>0 || dn_buf_Pr == ((dn_buf_Pw +1)% Buffer_Size) || stepcount_buf>0 )   
    return false;
    
  else
    return true;
}
uint16_t Step_Motor::now_pos(){//回傳 private 的 abs_pos
  //Serial.print("test_cannot_read------------------------------------");
  //Serial.println(test_cannot_read);  
/*
  Serial.print("t_5ms_buf------------------------------------");
  Serial.println(t_5ms_buf); 
  Serial.print("test_cannot_write------------------------------------");
  Serial.println(test_cannot_write);  
*/  

  /*
    Serial.print("test_n_send------------------------------------");
  Serial.println(test_n_send);  //第二個方法測試脈波發送數量是否正確
    Serial.print("stepcount_interrupt------------------------------------");
  Serial.println(stepcount_interrupt);
  Serial.print("inter_count------------------------------------");
  Serial.println(inter_count);
    Serial.print("t_5ms------------------------------------");
  Serial.println(t_5ms);
    Serial.print("dn------------------------------------");
  Serial.println(dn);

  for(int i = 0;i<201;i++){
    Serial.print("i------");
    Serial.print(i);
    Serial.print("------test_write_index------");    
    Serial.print(test_write_index[i]);
    Serial.print("----");
    Serial.println(test_cannot_write_buf[i]);
  }
  /*
  for(int i = 0;i<201;i++){
    Serial.print("i------");
    Serial.print(i);
    Serial.print("------test_read_index------");
    Serial.println(test_read_index[i]);
  }
*/
//*/
  return abs_steppos;
}
void Step_Motor::update_abspos(){
  if(!(dir_r_now^Dir_dir)){
    abs_steppos+=dn;
  }
  else
    abs_steppos-=dn;
}

bool Step_Motor::homing(){  //0點設為sensor處，且活動方向只有+
  if(has_home_pin && !isHomed){
    bitWrite(*HOME,pinHOME,1);
    delay(100);
    bitWrite(*HOME,pinHOME,0);
    abs_steppos = 0;
    isHomed = true;
    return true;
  }
  if(has_home_pin && isHomed){
    if(dn_buf_Pr == dn_buf_Pw){
      bitWrite(*HOME,pinHOME,1);
      delay(100);
      bitWrite(*HOME,pinHOME,0);
      abs_steppos = 0;
      return true;
    }
    else{
      return false;
    }
  }
  if(!has_home_sensor)
    return false;
  if(!isHomed){//沒有home pin 但是有home sensor (感光器)
    //dir_w_now = false;
    while(!bitRead(*SEN,pinSEN)){  //遮擋註為0 ??  1才對
      wbuffer_dn(-20);  
      wbuffer_dn(-20);
      //Serial.println("send a 10");
    }
    cli();
    Serial.println("checked"); 
    /*
    uint8_t readptr = dn_buf_Pr;
    Serial.println("checked");    
    uint8_t returnpos = dn_buf_Pw>readptr ? dn_buf_Pw - readptr : dn_buf_Pw + Buffer_Size - readptr;
    bitWrite(*DIR,pinDIR,1);
    for(uint8_t i = 0; i<returnpos;i++ ){
      wbuffer_dn(5); 
      wbuffer_dn(5); 
    }
    
    Serial.println(returnpos);
    */
    bitWrite(*DIR,pinDIR,Dir_dir);
    dn_buf_Pw = 0;
    dn_buf_Pr = 0;
    dn = 0;
    for(uint8_t i = 0; i<Buffer_Size;i++ ){
      dn_buf[i] = 0;
    }
    sei();
    abs_steppos = 0;
    isHomed = true;
    return true;
  }
  return true;
  
}

bool Step_Motor::move_by_f(float f, bool is_start,bool abs_start){
  if(is_start){
    f_n = 0;
    if(abs_start)
      n_pre = abs_steppos; //上次位置!!，此移動是使用絕對位置來描述，所以開始時要把n歸零在原來的絕對位置
    else
      n_pre = 0;
    //stepcount = x2step(x);
    //t_5ms_accumulate = t;
    t_5ms_buf = 0;

    f_n = f; //!!!!!!!!!!!!!
    n_this = f_n;
    dn_temp = n_this-n_pre;

    
    wbuffer_dn(dn_temp);

    t_5ms_buf += 1;  //記一下時間?
    //t_5ms_accumulate -= 1;
    return true;
  }
  else{
    n_pre = n_this;
    f_n = f; //!!!!!!!!!!!!!
    n_this = f_n;
    dn_temp = n_this-n_pre;

    wbuffer_dn(dn_temp);

    t_5ms_buf += 1;  //記一下時間?

    return true;
  }


}

void Step_Motor::EMG_Stop(){//直接中斷不減速，可搭配stop線
  stepcount_buf = 0;
  t_5ms_accumulate = 0;
  dn_buf_Pr = dn_buf_Pw;//這樣下次開啟timer就不會讀取了，寫入之後也會覆蓋掉還在buffer裡的資料
}
void Step_Motor::EMG_Stop_decelerate(){//
  //不關閉中斷
  //把cal_dn_write條件停止
  //填寫新dn，dn遞減，直到0，(while迴圈填寫，寫完後離開算急停完成)
  stepcount_buf = 0;
  t_5ms_accumulate = 0;
  dn_buf_Pr = dn_buf_Pw;//這個動作等同於把buffer清掉

  while(dn_temp!=0){ //buf大小32，dn不能超過1300(1/0.8^31) 程式上dn最大就128 沒問題
    if(dn_temp<3 && dn_temp>-3)
      dn_temp = 0;
    else
      dn_temp*=0.8;
    wbuffer_dn(dn_temp);
  }
}


