#include "state_machine.h"
#include "Esp_now.h"
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);

float speed, speed_max, angle;

void setup_LCD(){
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("  CAPSTONE PROJECT  ");
  lcd.setCursor(1, 1);
  lcd.print("4x4 vehicle console");
  lcd.setCursor(5, 3);
  delay(500);
  lcd.print("Loading");
  delay(1000);
  lcd.print(".");
  delay(500);
  lcd.print(".");
  delay(500);
  lcd.print(".");
  delay(500);
  lcd.print(".");
  delay(500);
  lcd.clear();
}
void LCD_print(int mode){
  lcd.setCursor(0, 0);
  lcd.print("Mode:");
  lcd.setCursor(5, 0);
  lcd.print(mode);

  speed = cal_speed(V_0);
  speed_max = cal_speed(V_0_max);

  switch(mode){
    case 0:
          lcd.print(" - Park mode  ");
          angle = 0;
          break;
    case 1:
          lcd.print(" - FW steering");
          angle = radian_to_degree(theta_0);
          break;
    case 2:
          lcd.print(" - AW steering");
          angle = radian_to_degree(theta_in);
          break;
    case 3:
          lcd.print(" - Zero turn  ");
          angle = 0;
          break;
    case 4:
          lcd.print("-DiagonalDrive");
          angle = radian_to_degree(theta_in);
          break;
    case 5:
          lcd.print(" - Crab walk  ");
          angle = 0;
          break;
    case 6:
          lcd.print(" - Pivot turn ");
          angle = 0;
          break;
    case 7:
          lcd.print(" - RW steering");
          angle = radian_to_degree(theta_0);
          break;
  }
  
  lcd.setCursor(0, 1);
  lcd.print("Pin:");
  //lcd.print(vehicle_data.SOC);
  lcd.print("85%");
  lcd.print(" - Volt:");
  lcd.print(vehicle_data.Voltage);
  
  lcd.setCursor(0, 2);
  lcd.print("Speed:");
  lcd.print(speed);
  lcd.print("/");
  lcd.print(speed_max);
  lcd.setCursor(17, 2);
  lcd.print("m/s");
  
  lcd.setCursor(0, 3);
  lcd.print("Steer:");
  lcd.print(angle);
  lcd.print("/sens:");
  if(servo_speed_rate == 2){
    lcd.print("100%");
  }
  else if(servo_speed_rate == 1){
    lcd.print("90% ");
  }
  else if(servo_speed_rate == 0.75){
    lcd.print("75% ");
  }
  else if(servo_speed_rate == 0.5){
    lcd.print("50% ");
  }
  else if(servo_speed_rate == 0.25){
    lcd.print("25% ");
  }
}

void serial_print(){
  Serial.print("FL_v: ");
  Serial.print(FL_speed);
  Serial.print("  FR_v: ");
  Serial.print(FR_speed);
  Serial.print("  RL_v: ");
  Serial.print(RL_speed);
  Serial.print("  RR_v: ");
  Serial.print(RR_speed);
  Serial.print("  FL_angle: ");
  //Serial.print(radian_to_degree(FL_angle));
  Serial.print(Set_FL_angle);
  Serial.print("  FR_angle: ");
  //Serial.print(radian_to_degree(FR_angle));
  Serial.print(Set_FR_angle);
  Serial.print("  RL_angle: ");
  //Serial.print(radian_to_degree(RL_angle));
  Serial.print(Set_RL_angle);
  Serial.print("  RR_angle: ");
  //Serial.println(radian_to_degree(RR_angle));
  Serial.print(Set_RR_angle);
  Serial.print("  temp:");
  Serial.print(vehicle_data.Temperature);
  Serial.print("  I:");
  Serial.print(vehicle_data.Current);
  Serial.print("  U:");
  Serial.print(vehicle_data.Voltage);
  Serial.print("  SOC:");
  Serial.println(vehicle_data.SOC);

}
void joystick_print(){ 
  Serial.print("left X = ");
  Serial.print(analogRead(left_X_pin));
  Serial.print("  left Y = ");
  Serial.print(analogRead(left_Y_pin));
  Serial.print("  right X = ");
  Serial.print(analogRead(right_X_pin));
  Serial.print("  right Y = ");
  Serial.println(analogRead(right_Y_pin));
}