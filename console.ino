#include "console.h"
#define left_SW_pin 18
#define right_SW_pin 26
#define brake_SW_pin 19
#define reset_SW_pin 25
#define left_pot_pin 36
#define right_pot_pin 39

/*________________TIMER PROCESS AND SEND__________________*/
uint8_t mode = 0;
hw_timer_t *send_data_timer = NULL;
hw_timer_t *process_data_timer = NULL;
bool flag_send_data = false;
bool flag_process_data = false;
void IRAM_ATTR isr_send_data(){
  flag_send_data = true;
}
void IRAM_ATTR isr_process_data(){
  flag_process_data = true;
}
/*_________________INTERRUPTS FUNCTIONS____________*/
bool flag_change_mode = false;
bool flag_reset_mode = false;
bool flag_dec_mode = false;
bool flag_inc_mode = false;
bool flag_reset = false;
int last_debounce = 0;          // Chống dội nút nhấn
int delay_time = 800;          // delay nút nhấn

void IRAM_ATTR isr_reset_mode(){
  if(flag_change_mode == false){
    flag_change_mode = true;
    flag_reset_mode = true;
    last_debounce = millis();
  }
}
void IRAM_ATTR isr_inc_mode(){
  if(flag_change_mode == false){
    flag_change_mode = true;
    flag_inc_mode = true;
    last_debounce = millis();
  }
}
void IRAM_ATTR isr_dec_mode(){
  if(flag_change_mode == false){
    flag_change_mode = true;
    flag_dec_mode = true;
    last_debounce = millis();
  }
}

void setup(){
  Serial.begin(115200);
  pinMode(left_SW_pin, INPUT_PULLUP);
  pinMode(right_SW_pin, INPUT_PULLUP);
  pinMode(brake_SW_pin, INPUT_PULLUP);
  pinMode(reset_SW_pin, INPUT_PULLUP);
// SETUP LCD:
  setup_LCD();
// SETUP INTERRUPT FUNCTIONS:
  attachInterrupt(left_SW_pin, isr_dec_mode, FALLING);
  attachInterrupt(right_SW_pin, isr_inc_mode, FALLING);
  attachInterrupt(reset_SW_pin, isr_reset_mode, FALLING);
  //attachInterrupt(brake_SW_pin, isr_brake_mode, FALLING);
// SETUP TIMER FOR SENDING DATA BY ESP-NOW:
  send_data_timer = timerBegin(0, 80, true);      // prescaler = 80 -> freq = 1.000.000 Hz
  timerAttachInterrupt(send_data_timer, isr_send_data, true);
  timerAlarmWrite(send_data_timer, 10000, true);  // T = 10000us -> F = 100 Hz (gửi 10ms 1 lần)
  timerAlarmEnable(send_data_timer);
// SETUP TIMER FOR CALCULATING:
  process_data_timer = timerBegin(1, 80, true);
  timerAttachInterrupt(process_data_timer, isr_process_data, true);
  timerAlarmWrite(process_data_timer, 1000, true); // T = 1000us (tính toán 1ms 1 lần)
  timerAlarmEnable(process_data_timer);
// SETUP ESP-NOW:
  setup_esp_now();
}

void loop(){
  if(flag_change_mode){
    if(millis() - last_debounce >= delay_time){
      if(flag_inc_mode && mode < 7){
        flag_inc_mode = false;
        mode++;
      }
      else if(flag_dec_mode && mode > 0){
        flag_dec_mode = false;
        mode--;
      }
      else if(flag_reset_mode){
        flag_reset_mode = false;
        mode = 0;
      }
      if(abs(Set_FL_angle) < 0.5 && abs(Set_FR_angle) < 0.5 && abs(Set_RL_angle) < 0.5 && abs(Set_RR_angle) < 0.5){
        flag_change_mode = false;
      }
    }
  }

  if(flag_change_mode){
    main_controller(0);
  }
  else{
    if(flag_process_data){
      flag_process_data = false;
      // Update V0_max from left pot
      V_0_max = map_pot_to_V_0_max(analogRead(left_pot_pin));
      servo_speed_rate = map_pot_to_servo_speed(analogRead(right_pot_pin));
      main_controller(mode);
    }
  } 

  if(flag_send_data){
    flag_send_data = false;
    control_data.FL_angle = Set_FL_angle;
    control_data.FR_angle = Set_FR_angle;
    control_data.RL_angle = Set_RL_angle;
    control_data.RR_angle = Set_RR_angle;
    control_data.FL_speed = (-1.0)*FL_speed;
    control_data.FR_speed = FR_speed;
    control_data.RL_speed = (-1.0)*RL_speed;
    control_data.RR_speed = RR_speed;
    control_data.mode = mode;
    send_result = esp_now_send(master_addr, (uint8_t *)&control_data, sizeof(control_data));
    LCD_print(mode);
    serial_print();
  }
}