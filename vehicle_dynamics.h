#define brake_SW_pin 19
int brake_state = 0;
float servo_speed_rate;
bool flag_control_speed = false;
/*_____________________CHANGE STEERING DE/ACCELERATION_________________*/
float Steering_control(uint8_t MODE, float &steering_angle, float set_point){
  if(servo_speed_rate == 2 && MODE != 2){
    steering_angle = set_point;
    return steering_angle;
  }
  switch(MODE){
    case 0:
          if(abs(set_point - steering_angle) > 200){
            if(steering_angle < set_point){
              steering_angle += servo_speed_rate * 150;
            }
            else if(steering_angle > set_point){
              steering_angle -= servo_speed_rate * 150;
            }
            else if(steering_angle == set_point){
              steering_angle = set_point;
            }
          }
          else if(100 < abs(set_point - steering_angle) && abs(set_point - steering_angle) <= 200){
            if(steering_angle < set_point){
              steering_angle += servo_speed_rate * 100;
            }
            else if(steering_angle > set_point){
              steering_angle -= servo_speed_rate * 100;
            }
            else if(steering_angle == set_point){
              steering_angle = set_point;
            }
          }
          else if(90 < abs(set_point - steering_angle) && abs(set_point - steering_angle) <= 100){
            if(steering_angle < set_point){
              steering_angle += servo_speed_rate * 80;
            }
            else if(steering_angle > set_point){
              steering_angle -= servo_speed_rate * 80;
            }
            else if(steering_angle == set_point){
              steering_angle = set_point;
            }
          }
          else if(40 < abs(set_point - steering_angle) && abs(set_point - steering_angle) <= 90){
            if(steering_angle < set_point){
              steering_angle += servo_speed_rate * 50;
            }
            else if(steering_angle > set_point){
              steering_angle -= servo_speed_rate * 50;
            }
            else if(steering_angle == set_point){
              steering_angle = set_point;
            }
          }
          else if(20 < abs(set_point - steering_angle) && abs(set_point - steering_angle) <= 40){
            if(steering_angle < set_point){
              steering_angle += servo_speed_rate * 20;
            }
            else if(steering_angle > set_point){
              steering_angle -= servo_speed_rate * 20;
            }
            else if(steering_angle == set_point){
              steering_angle = set_point;
            }
          }
          else if(10 < abs(set_point - steering_angle) && abs(set_point - steering_angle) <= 20){
            if(steering_angle < set_point){
              steering_angle += servo_speed_rate * 10;
            }
            else if(steering_angle > set_point){
              steering_angle -= servo_speed_rate * 10;
            }
            else if(steering_angle == set_point){
              steering_angle = set_point;
            }
          }
          else if(abs(set_point - steering_angle) <= 10){
            if(steering_angle < set_point){
              steering_angle += 1;
            }
            else if(steering_angle > set_point){
              steering_angle -= 1;
            }
            else if(steering_angle == set_point){
              steering_angle = set_point;
            }
          }
          return steering_angle;
    break;
    case 1:
          if(servo_speed_rate == 1){
            if(abs(steering_angle - set_point) >= 200){
              if(steering_angle < set_point){
                steering_angle += abs(steering_angle - set_point) / 2;
              }
              else if(steering_angle > set_point){
                steering_angle -= abs(steering_angle - set_point) / 2;
              }
            }
            else if(abs(steering_angle - set_point) < 200){
              steering_angle = set_point;
            } 
          }
          else if(servo_speed_rate == 0.75){
            if(abs(steering_angle - set_point) >= 150){
              if(steering_angle < set_point){
                steering_angle += abs(steering_angle - set_point) / 3;
              }
              else if(steering_angle > set_point){
                steering_angle -= abs(steering_angle - set_point) / 3;
              }
            }
            else if(abs(steering_angle - set_point) < 150){
              steering_angle = set_point;
            } 
          }
          else if(servo_speed_rate == 0.5){
            if(abs(steering_angle - set_point) >= 100){
              if(steering_angle < set_point){
                steering_angle += abs(steering_angle - set_point) / 4;
              }
              else if(steering_angle > set_point){
                steering_angle -= abs(steering_angle - set_point) / 4;
              }
            }
            else if(abs(steering_angle - set_point) < 100){
              steering_angle = set_point;
            } 
          }
          return steering_angle;
    break;
    case 3:
          if(abs(set_point - steering_angle) > 250){
            if(steering_angle < set_point){
              steering_angle += servo_speed_rate * 200;
            }
            else if(steering_angle > set_point){
              steering_angle -= servo_speed_rate * 200;
            }            
          }
          else if(100 < abs(set_point - steering_angle) && abs(set_point - steering_angle) <= 250){
            if(steering_angle < set_point){
              steering_angle += servo_speed_rate * 100;
            }
            else if(steering_angle > set_point){
              steering_angle -= servo_speed_rate * 100;
            }
          }
          else if(abs(set_point - steering_angle) <= 100){
            steering_angle = set_point;
          }
          return steering_angle;
    break;
    // case 1:
    //       if(abs(set_point - steering_angle) > 40){
    //         if(steering_angle < set_point){
    //           steering_angle += servo_speed_rate * 34.5;
    //         }
    //         else if(steering_angle > set_point){
    //           steering_angle -= servo_speed_rate * 34.5;
    //         }
    //         else if(steering_angle == set_point){
    //           steering_angle = set_point;
    //         }
    //       }
    //       else if(10 < abs(set_point - steering_angle) && abs(set_point - steering_angle) <= 40){
    //         if(steering_angle < set_point){
    //           steering_angle += servo_speed_rate * 34.5;
    //         }
    //         else if(steering_angle > set_point){
    //           steering_angle -= servo_speed_rate * 34.5;
    //         }
    //         else if(steering_angle == set_point){
    //           steering_angle = set_point;
    //         }
    //       }
    //       else if(abs(set_point - steering_angle) <= 10){
    //         if(steering_angle < set_point){
    //           steering_angle += 1;
    //         }
    //         else if(steering_angle > set_point){
    //           steering_angle -= 1;
    //         }
    //         else if(steering_angle == set_point){
    //           steering_angle = set_point;
    //         }
    //       }
    //       return steering_angle;
    // break;
    case 2:
          if(abs(set_point - steering_angle) > 50){
            if(steering_angle < set_point){
              steering_angle += 20;
            }
            else if(steering_angle > set_point){
              steering_angle -= 20;
            }
            else if(steering_angle == set_point){
              steering_angle = set_point;
            }
          }
          else if(10 < abs(set_point - steering_angle) && abs(set_point - steering_angle <= 50)){
            if(steering_angle < set_point){
              steering_angle += 10;
            }
            else if(steering_angle > set_point){
              steering_angle -= 10;
            }
            else if(steering_angle == set_point){
              steering_angle = set_point;
            }
          }
          else if(1 < abs(set_point - steering_angle) && abs(set_point - steering_angle <= 10)){
            if(steering_angle < set_point){
              steering_angle += 1;
            }
            else if(steering_angle > set_point){
              steering_angle -= 1;
            }
            else if(steering_angle == set_point){
              steering_angle = set_point;
            }
          }
          else if(abs(set_point - steering_angle) <= 1){
            // if(steering_angle < set_point){
            //   steering_angle += 0.1;
            // }
            // else if(steering_angle > set_point){
            //   steering_angle -= 0.1;
            // }
            // else if(steering_angle == set_point){
            //   steering_angle = set_point;
            // }
            steering_angle = set_point;
          }
          if(steering_angle == set_point){
            flag_control_speed = true;
          }
          else{
            flag_control_speed = false;
          }
          return steering_angle;
    break;
  }
}

/*___________________De_Ac_celeration FUNCTION__________________*/
float De_Ac_celeration(float &vehicle_speed, float set_point){
  brake_state = digitalRead(brake_SW_pin);
  if(brake_state == 0){
    if(abs(vehicle_speed) > 200){
      if(vehicle_speed > 0){
        vehicle_speed -= 150;
      }
      else if(vehicle_speed < 0){
        vehicle_speed += 150;
      }
    }
    else if(abs(vehicle_speed) > 100 && abs(vehicle_speed) <= 200){
      if(vehicle_speed > 0){
        vehicle_speed -= 100;
      }
      else if(vehicle_speed < 0){
        vehicle_speed += 100;
      }
    }
    else if(abs(vehicle_speed) <= 100){
      vehicle_speed = 0;
    }
  }
  else{
    if(abs(set_point - vehicle_speed) > 100){
      if(vehicle_speed < set_point){
        //vehicle_speed += 10;
        vehicle_speed += 25;
      }
      else if(vehicle_speed > set_point){
        vehicle_speed -= 25;
      }
      else if(vehicle_speed == set_point){
        vehicle_speed = set_point;
      }
    }
    else if(50 < abs(set_point - vehicle_speed) && abs(set_point - vehicle_speed <= 100)){
      if(vehicle_speed < set_point){
        //vehicle_speed += 5;
        vehicle_speed += 10;
      }
      else if(vehicle_speed > set_point){
        vehicle_speed -= 10;
      }
      else if(vehicle_speed == set_point){
        vehicle_speed = set_point;
      }
    }
    else if(10 < abs(set_point - vehicle_speed) && abs(set_point - vehicle_speed <= 50)){
      if(vehicle_speed < set_point){
        //vehicle_speed += 2;
        vehicle_speed += 5;
      }
      else if(vehicle_speed > set_point){
        vehicle_speed -= 5;
      }
      else if(vehicle_speed == set_point){
        vehicle_speed = set_point;
      }
    }
    else if(abs(set_point - vehicle_speed) <= 10){
      // if(vehicle_speed < set_point){
      //   vehicle_speed += 1;
      // }
      // else if(vehicle_speed > set_point){
      //   vehicle_speed -= 1;
      // }
      // else if(vehicle_speed == set_point){
      //   vehicle_speed = set_point;
      // }
      vehicle_speed = set_point;
    }
  }
  return vehicle_speed;
}