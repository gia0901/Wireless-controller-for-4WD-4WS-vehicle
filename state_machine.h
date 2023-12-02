#include "vehicle_dynamics.h"
#define left_X_pin 35
#define left_Y_pin 34
#define right_X_pin 33
#define right_Y_pin 32
#define brake_SW_pin 19
/*________________THÔNG SỐ XE_______________*/
#define SERVO_MAX -450.00
#define SERVO_MIN 450.00
#define THETA_IN_MAX_ANGLE 45 // góc quay tối đa bánh xe
const float theta_in_max = (THETA_IN_MAX_ANGLE * PI) / 180;
const float L = 400.00;
const float W = 490.00;
const float C_2 = L/2;
const float set_V_0_max = 400.00;
float theta_0_max;             // góc quay tại trọng tâm
/*________________TEMP VARIABLES______________*/
float FL_angle = 0;     // Các biến tính theo lý thuyết
float FR_angle = 0;     
float RL_angle = 0;
float RR_angle = 0;
float Set_FL_angle = 0; // Set lại số lý thuyết -> thông số servo
float Set_FR_angle = 0; // Không cần biến set cho speed vì tính ra là RPM -> dùng luôn
float Set_RL_angle = 0;
float Set_RR_angle = 0;
float FL_speed = 0;
float FR_speed = 0;
float RL_speed = 0;
float RR_speed = 0;
float theta_0 = 0;
float theta_in = 0;
float V_0 = 0;
int V_0_max = 0;

float R = 0;
float R1 = 0;

float calculate_theta_0_max(int mode){
  switch(mode){
    case 1:
      return atan(L / ((2*L/tan(theta_in_max))+W));
    case 2:
      return atan(L / ((L/tan(theta_in_max))+W));
  } 
}
int int_map(float x, float in_min, float in_max, float out_min, float out_max){
  return (int)(((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min);
}

float float_map(float x, float in_min, float in_max, float out_min, float out_max){
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}
float map_joystick_to_theta(float joystick_value, int mode){
  switch(mode){
    case 1:
      if(joystick_value > 1850 && joystick_value < 1960){
        return 0;
      }
      else if(joystick_value <= 1850){
        return float_map(joystick_value, 1850.00, 0.00, 0.00, calculate_theta_0_max(1));
      }
      else if(joystick_value >= 1960){
        return float_map(joystick_value, 1960.00, 4095.00, 0.00, (-1.00)*calculate_theta_0_max(1));
      }
      break;
    case 2: // TH này tính theo theta_in_max (góc bx lớn nhất)
      if(joystick_value > 1850 && joystick_value < 1960){
        return 0;
      }
      else if(joystick_value <= 1850){
        //return float_map(joystick_value, 1850.00, 0.00, 0.00, calculate_theta_0_max(3));
        return float_map(joystick_value, 1850.00, 0.00, 0.00, theta_in_max);      
      }
      else if(joystick_value >= 1960){
        //return float_map(joystick_value, 1960.00, 4095.00, 0.00, (-1.00)*calculate_theta_0_max(3));
        return float_map(joystick_value, 1960.00, 4095.00, 0.00, (-1.00)*theta_in_max);
      }
      break;
  }
}

float map_joystick_to_v(float joystick_value){
  if(joystick_value > 1850 && joystick_value < 1960){
    return 0;
  }
  else if(joystick_value <= 1850){
    return float_map(joystick_value, 1850.00, 0.00, 0.00, (-1.0)*V_0_max);
  }
  else if(joystick_value >= 1960){
    return float_map(joystick_value, 1960.00, 4095.00, 0.00, V_0_max);
  }
}

/*_____________________CONTROL SERVOS FUNCTION__________________*/
void control_servos(int mode){
  switch(mode){
    case 0:     // reset mode
            FL_angle = 0;
            FR_angle = 0;
            RL_angle = 0;
            RR_angle = 0;
            Steering_control(2, Set_FL_angle, float_map(FL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(2, Set_FR_angle, float_map(FR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(2, Set_RL_angle, float_map(RL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(2, Set_RR_angle, float_map(RR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
    break;
    case 1:     // Front-wheel-steering
            RL_angle = 0;
            RR_angle = 0;
            if(theta_0 == 0){
              FL_angle = 0;
              FR_angle = 0;
            }
            else if(theta_0 > 0){
              FR_angle = atan(L / (R - W/2));
              FL_angle = atan(L / (R + W/2));
            }
            else if(theta_0 < 0){
              FL_angle = (-1.0)*atan(L / (R - W/2));
              FR_angle = (-1.0)*atan(L / (R + W/2));
            }
            // Set_FL_angle = float_map(FL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
            // Set_FR_angle = float_map(FR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
            // Set_RL_angle = float_map(RL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
            // Set_RR_angle = float_map(RR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);

            Steering_control(1, Set_FL_angle, float_map(FL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_FR_angle, float_map(FR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_RL_angle, float_map(RL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_RR_angle, float_map(RR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
    break;
    case 2:     // All-wheel-steering
            if(theta_in == 0){
              FL_angle = 0;
              FR_angle = 0;
              RL_angle = 0;
              RR_angle = 0;
            }
            else if(theta_in > 0){
              FL_angle = atan(L / (R + W/2));
              FR_angle = theta_in;
              RL_angle = (-1.0)*atan(L / (R + W/2));
              RR_angle = (-1.0)*theta_in;
              //FR_angle = atan(L / (R - W/2));
              //RR_angle = (-1.0)*atan(L / (R - W/2));
            }
            else if(theta_in < 0){
              FL_angle = theta_in;
              FR_angle = (-1.0)*atan(L / (R + W/2));
              RL_angle = (-1.0)*theta_in;
              RR_angle = atan(L / (R + W/2));
              //FL_angle = (-1.0)*atan(L / (R - W/2));
              //RL_angle = atan(L / (R - W/2));
            }
            // Set_FL_angle = float_map(FL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
            // Set_FR_angle = float_map(FR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
            // Set_RL_angle = float_map(RL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
            // Set_RR_angle = float_map(RR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
            Steering_control(1, Set_FL_angle, float_map(FL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_FR_angle, float_map(FR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_RL_angle, float_map(RL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_RR_angle, float_map(RR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
    break;
    case 3:     // Zero turn
            FR_angle = (-1.0) *atan(L/W);
            FL_angle = atan(L/W);
            RL_angle = (-1.0) *atan(L/W);
            RR_angle = atan(L/W);

            Steering_control(2, Set_FL_angle, float_map(FL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(2, Set_FR_angle, float_map(FR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(2, Set_RL_angle, float_map(RL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(2, Set_RR_angle, float_map(RR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
    break;
    // case 5:     // Diagonal with turning axis
    //         if(theta_0 == 0){
    //           FL_angle = 0;
    //           FR_angle = 0;
    //           RL_angle = 0;
    //           RR_angle = 0;
    //         }
    //         else if(theta_0 > 0){
    //           FL_angle = atan((L + C_2) / (R + W/2));
    //           FR_angle = atan((L + C_2)/ (R - W/2));
    //           RL_angle = (-1.0)*atan(C_2 / (R + W/2));
    //           RR_angle = (-1.0)*atan(C_2 / (R - W/2));
    //         }
    //         else if(theta_0 < 0){
    //           FL_angle = (-1.0)*atan((L + C_2) / (R - W/2));
    //           FR_angle = (-1.0)*atan((L + C_2) / (R + W/2));
    //           RL_angle = atan(C_2 / (R - W/2));
    //           RR_angle = atan(C_2 / (R + W/2));
    //         }
    //         Set_FL_angle = float_map(FL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
    //         Set_FR_angle = float_map(FR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
    //         Set_RL_angle = float_map(RL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
    //         Set_RR_angle = float_map(RR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
    // break;
    case 4:     // Diagonal driving
            if(theta_in == 0){
              FL_angle = 0;
              FR_angle = 0;
              RL_angle = 0;
              RR_angle = 0;  
            }
            else if(theta_in != 0){
              FL_angle = theta_in;
              FR_angle = theta_in;
              RL_angle = theta_in;
              RR_angle = theta_in;
            }
            // Set_FL_angle = float_map(FL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
            // Set_FR_angle = float_map(FR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
            // Set_RL_angle = float_map(RL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
            // Set_RR_angle = float_map(RR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
            Steering_control(1, Set_FL_angle, float_map(FL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_FR_angle, float_map(FR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_RL_angle, float_map(RL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_RR_angle, float_map(RR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
    break;
    case 5:     // Crab walk
            if(theta_0 > 0 && V_0 == 0){
              // FR_angle = (-1.0)*atan(L/W);
              // FL_angle = (-1.0)* atan(L/W);
              // RL_angle = atan(L/W);
              // RR_angle = atan(L/W);
              FR_angle = (-1.0)*PI/4;
              FL_angle = (-1.0)*PI/4;
              RL_angle = PI/4;
              RR_angle = PI/4;
            }
            else if(theta_0 < 0 && V_0 == 0){
              // FR_angle = atan(L/W);
              // FL_angle = atan(L/W);
              // RL_angle = (-1.0)* atan(L/W);
              // RR_angle = (-1.0)* atan(L/W);
              FR_angle = PI/4;
              FL_angle = PI/4;
              RL_angle = (-1.0)* PI/4;
              RR_angle = (-1.0)* PI/4;         
            }
            Steering_control(2, Set_FL_angle, float_map(FL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(2, Set_FR_angle, float_map(FR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(2, Set_RL_angle, float_map(RL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(2, Set_RR_angle, float_map(RR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
    break;
    case 6: // Pivot turn
            FL_angle = atan(L / (W/2));
            FR_angle = (-1.0)*atan(L / (W/2));
            RL_angle = 0;
            RR_angle = 0;
            
            Steering_control(1, Set_FL_angle, float_map(FL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_FR_angle, float_map(FR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_RL_angle, float_map(RL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_RR_angle, float_map(RR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
    break;
    case 7: // Rear-wheel-steering
            FL_angle = 0;
            FR_angle = 0;
            if(theta_0 == 0){
              RL_angle = 0;
              RR_angle = 0;
            }
            else if(theta_0 > 0){
              RL_angle = (-1.0)*atan(L / (R - W/2));
              RR_angle = (-1.0)*atan(L / (R + W/2));
            }
            else if(theta_0 < 0){
              RR_angle = atan(L / (R - W/2));
              RL_angle = atan(L / (R + W/2));
            }
            // else if(theta_0 > 0){
            //   FR_angle = atan(L / (R - W/2));
            //   FL_angle = atan(L / (R + W/2));
            // }
            // else if(theta_0 < 0){
            //   FL_angle = (-1.0)*atan(L / (R - W/2));
            //   FR_angle = (-1.0)*atan(L / (R + W/2));
            // }
            // Set_FL_angle = float_map(FL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
            // Set_FR_angle = float_map(FR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
            // Set_RL_angle = float_map(RL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);
            // Set_RR_angle = float_map(RR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX);

            Steering_control(1, Set_FL_angle, float_map(FL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_FR_angle, float_map(FR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_RL_angle, float_map(RL_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
            Steering_control(1, Set_RR_angle, float_map(RR_angle, -(1.0)*PI/4, PI/4, SERVO_MIN, SERVO_MAX));
    break;
  }  
}

/*___________________CONTROL MOTORS FUNCTION_________________*/
void control_motors(int mode){
  switch(mode){
    case 0:       // Reset mode (Parking mode)
          FL_speed = De_Ac_celeration(FL_speed, 0);
          FR_speed = De_Ac_celeration(FR_speed, 0);
          RL_speed = De_Ac_celeration(RL_speed, 0);
          RR_speed = De_Ac_celeration(RR_speed, 0);
    break;
    case 1:       // Front-wheel-steering
          if(theta_0 == 0){
            FL_speed = V_0;
            FR_speed = FL_speed;
            RL_speed = FL_speed;
            RR_speed = FL_speed;
          }
          else if(theta_0 > 0){
            FL_speed = (V_0 / R1) * sqrt(pow(L, 2.0) + pow(R + W/2, 2.0));
            FR_speed = (V_0 / R1) * sqrt(pow(L, 2.0) + pow(R - W/2, 2.0));
            RL_speed = (V_0 / R1) * (R + W/2);
            RR_speed = (V_0 / R1) * (R - W/2); 
          }
          else if(theta_0 < 0){
            FL_speed = (V_0 / R1) * sqrt(pow(L, 2.0) + pow(R - W/2, 2.0));
            FR_speed = (V_0 / R1) * sqrt(pow(L, 2.0) + pow(R + W/2, 2.0));
            RL_speed = (V_0 / R1) * (R - W/2);
            RR_speed = (V_0 / R1) * (R + W/2);
          }
    break;
    case 2:      // All-wheel-steering
          if(theta_in == 0){
            FL_speed = V_0;
            FR_speed = FL_speed;
            RL_speed = FL_speed;
            RR_speed = FL_speed;
          }
          else if(theta_in > 0){
            FL_speed = (V_0 / R1)*sqrt(pow(L/2, 2.0) + pow(R + W/2, 2.0));
            FR_speed = (V_0 / R1)*sqrt(pow(L/2, 2.0) + pow(R - W/2, 2.0));
            RL_speed = (V_0 / R1)*sqrt(pow(L/2, 2.0) + pow(R + W/2, 2.0));
            RR_speed = (V_0 / R1)*sqrt(pow(L/2, 2.0) + pow(R - W/2, 2.0));
          }
          else if(theta_in < 0){
            FL_speed = (V_0 / R1)*sqrt(pow(L/2, 2.0) + pow(R - W/2, 2.0));
            FR_speed = (V_0 / R1)*sqrt(pow(L/2, 2.0) + pow(R + W/2, 2.0));
            RL_speed = (V_0 / R1)*sqrt(pow(L/2, 2.0) + pow(R - W/2, 2.0));
            RR_speed = (V_0 / R1)*sqrt(pow(L/2, 2.0) + pow(R + W/2, 2.0));
          }
    break;
    case 3:       // Zero-turn 
        if(flag_control_speed){
          if(V_0 == 0){
            FL_speed = 0;
            FR_speed = 0;
            RL_speed = 0;
            RR_speed = 0;
          }
          else if(V_0 < 0){
            FL_speed = (-1.0)*V_0;
            FR_speed = V_0;
            RL_speed = (-1.0)*V_0;
            RR_speed = V_0;
          }
          else if(V_0 > 0){
            FL_speed = (-1.0)*V_0;
            FR_speed = V_0;
            RL_speed = (-1.0)*V_0;
            RR_speed = V_0;
          }
        }
    break;
    case 4:       // Diagonal driving
            FL_speed = V_0;
            FR_speed = FL_speed;      
            RL_speed = FL_speed;
            RR_speed = FL_speed;     
    break;
    case 5:      // Crab walk
          if(V_0 == 0){
            FL_speed = 0;
            FR_speed = 0;
            RL_speed = 0;
            RR_speed = 0;
          }
          else if(V_0 > 0){
            FL_speed = (-1.0)*V_0;
            FR_speed = (-1.0)*V_0;
            RL_speed = V_0;
            RR_speed = V_0;
          }
          else if(V_0 < 0){
            FL_speed = V_0;
            FR_speed = V_0;
            RL_speed = (-1.0)*V_0;
            RR_speed = (-1.0)*V_0;
          }
    break;
    
    // case 5:       // Diagonal driving with turning axis
    //       if(theta_0 == 0){
    //         FL_speed = De_Ac_celeration(FL_speed, V_0);
    //         FR_speed = FL_speed;
    //         RL_speed = FL_speed;
    //         RR_speed = FL_speed;
    //       }
    //       else if(theta_0 > 0){ 
    //         FL_speed = De_Ac_celeration(FL_speed, (V_0 / R1)*sqrt(pow(L + C_2, 2.0) + pow(R + W/2, 2.0)));
    //         FR_speed = De_Ac_celeration(FR_speed, (V_0 / R1)*sqrt(pow(L + C_2, 2.0) + pow(R - W/2, 2.0)));
    //         RL_speed = De_Ac_celeration(RL_speed, (V_0 / R1)*sqrt(pow(C_2, 2.0) + pow(R + W/2, 2.0)));
    //         RR_speed = De_Ac_celeration(RR_speed, (V_0 / R1)*sqrt(pow(C_2, 2.0) + pow(R - W/2, 2.0)));
    //       }
    //       else if(theta_0 < 0){ 
    //         FL_speed = De_Ac_celeration(FL_speed, (V_0 / R1)*sqrt(pow(L + C_2, 2.0) + pow(R - W/2, 2.0)));
    //         FR_speed = De_Ac_celeration(FR_speed, (V_0 / R1)*sqrt(pow(L + C_2, 2.0) + pow(R + W/2, 2.0)));
    //         RL_speed = De_Ac_celeration(RL_speed, (V_0 / R1)*sqrt(pow(C_2, 2.0) + pow(R - W/2, 2.0)));
    //         RR_speed = De_Ac_celeration(RR_speed, (V_0 / R1)*sqrt(pow(C_2, 2.0) + pow(R + W/2, 2.0)));
    //       }
    // break;
    case 6:   // Pivot turn
          if(V_0 == 0){
            FL_speed = 0;
            FR_speed = 0;
            RL_speed = 0;
            RR_speed = 0;
          }
          else if(V_0 < 0){
            FL_speed = (-1.0)*(V_0 / (W/2)) * sqrt (pow (W/2, 2.0) + pow (L, 2.0));
            FR_speed = (V_0 / (W/2)) * sqrt (pow (W/2, 2.0) + pow (L, 2.0));
            RL_speed = (-1.0)*V_0;
            RR_speed = V_0;
          }
          else if(V_0 > 0){
            FL_speed = (-1.0) *(V_0 / (W/2)) * sqrt (pow (W/2, 2.0) + pow (L, 2.0));
            FR_speed = (V_0 / (W/2)) * sqrt (pow (W/2, 2.0) + pow (L, 2.0));
            RL_speed = (-1.0) * V_0;
            RR_speed = V_0;
          }
    break;
    case 7:       // Rear-wheel-steering
          if(theta_0 == 0){
            FL_speed = V_0;
            FR_speed = FL_speed;
            RL_speed = FL_speed;
            RR_speed = FL_speed;
          }
          else if(theta_0 > 0){
            RR_speed = (V_0 / R1) * sqrt(pow(L, 2.0) + pow(R + W/2, 2.0));
            RL_speed = (V_0 / R1) * sqrt(pow(L, 2.0) + pow(R - W/2, 2.0));
            FR_speed = (V_0 / R1) * (R + W/2);
            FL_speed = (V_0 / R1) * (R - W/2); 
          }
          else if(theta_0 < 0){
            RR_speed = (V_0 / R1) * sqrt(pow(L, 2.0) + pow(R - W/2, 2.0));
            RL_speed = (V_0 / R1) * sqrt(pow(L, 2.0) + pow(R + W/2, 2.0));
            FR_speed = (V_0 / R1) * (R - W/2);
            FL_speed = (V_0 / R1) * (R + W/2);
          }
    break;
  }
}
/*____________________MAIN CONTROL FUNCTION____________________*/
void main_controller(int mode){
  switch(mode){
    case 0:     // Reset mode (parking mode)
            control_servos(0);
            control_motors(0);
    break;
    case 1:     // Front-wheel-steering
            theta_0 = map_joystick_to_theta(analogRead(right_X_pin), 1);
            V_0 = map_joystick_to_v(analogRead(left_Y_pin));
            if(theta_0 != 0){
              R = L / (2 * abs(tan(theta_0)));
              R1 = sqrt(pow(R, 2.0) + pow((L/2), 2.0)); 
            }
            control_servos(1);
            control_motors(1);
    break;
    case 2:     // All-wheel-steering
            theta_in = map_joystick_to_theta(analogRead(right_X_pin), 2);
            V_0 = map_joystick_to_v(analogRead(left_Y_pin));
            if(theta_in != 0){
              R = 0.5*((L/tan(abs(theta_in)) + W));
              R1 = R;
            }
            control_servos(2);
            control_motors(2);
    break;
    case 3:  // Zero turn
            V_0 = map_joystick_to_v(analogRead(left_X_pin));
            control_servos(3);
            control_motors(3);
    break;
    case 4: // Diagonal drive
            theta_in = map_joystick_to_theta(analogRead(right_X_pin), 2);
            V_0 = map_joystick_to_v(analogRead(left_Y_pin));
            control_servos(4);
            control_motors(4);
    break;
    case 5:   // Crab walk
            theta_0 = map_joystick_to_theta(analogRead(right_X_pin), 1);
            V_0 = map_joystick_to_v(analogRead(left_X_pin));
            control_servos(5);
            control_motors(5);
    break;
    // case 5: // Diagonal driving with turning axis
    //         theta_0 = map_joystick_to_theta(analogRead(right_X_pin), 1);
    //         V_0 = map_joystick_to_v(analogRead(left_Y_pin));
    //         if(theta_0 != 0){
    //           R = (L + C_2)/abs(tan(theta_0));
    //           R1 = sqrt(pow(L/2 + C_2 , 2.0) + pow(R , 2.0));
    //         }
    //         control_servos(5);
    //         control_motors(5);
    // break;
    case 6: // Pivot turn
            V_0 = map_joystick_to_v(analogRead(left_X_pin));
            control_servos(6);
            control_motors(6);
    break;
    case 7: // Rear-wheel-steering
            theta_0 = map_joystick_to_theta(analogRead(right_X_pin), 1);
            V_0 = map_joystick_to_v(analogRead(left_Y_pin));
            if(theta_0 != 0){
              R = L / (2 * abs(tan(theta_0)));
              R1 = sqrt(pow(R, 2.0) + pow((L/2), 2.0)); 
            }
            control_servos(7);
            control_motors(7);
    break;
  }
}

float radian_to_degree(float radian){
  return radian * 180 / PI;
}
int map_pot_to_V_0_max(int pot){
  if(pot == 0){
    return 0.2 * set_V_0_max;
  }
  else if(pot > 0 && pot <= 1020){
    return 0.4 * set_V_0_max;
  }
  else if(pot > 1040 && pot <= 2040){
    return 0.6 * set_V_0_max;
  }
  else if(pot > 2060 && pot <= 3070){
    return 0.8 * set_V_0_max;
  }
  else if(pot >= 3090){
    return set_V_0_max;
  }
}
float map_pot_to_servo_speed(int pot){
  if(pot == 0){
    return 0.25;
  }
  else if(pot > 0 && pot <= 1020){
    return 0.5;
  }
  else if(pot > 1040 && pot <= 2040){
    return 0.75;
  }
  else if(pot > 2060 && pot <= 3070){
    return 1;
  }
  else if(pot >= 3090){
    return 2;
  }
}
float cal_speed(float speed){
  return speed/125;
}