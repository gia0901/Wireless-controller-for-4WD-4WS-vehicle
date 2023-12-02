#include <esp_now.h>
#include <WiFi.h>
/*________________MASTER address______________*/
//uint8_t master_addr[] = {0xC4, 0xDE, 0xE2, 0x1F, 0xA3, 0x10};
//uint8_t master_addr[] = {0x58 ,0xBF ,0x25, 0x35, 0xF6, 0x58};
//uint8_t master_addr[] = {0xAC, 0x67, 0xB2, 0x3D, 0x1F, 0x4C};
//uint8_t master_addr[] = {0x58, 0xBF, 0x25, 0x37, 0x2C, 0x60};
uint8_t master_addr[] = {0xAC, 0x67, 0xB2, 0x3D, 0x1F, 0x4C};

esp_err_t send_result;

/*________________STRUCT__________________*/
typedef struct control_struct{
  uint8_t mode;
  float FL_angle;
  float FR_angle;
  float RL_angle;
  float RR_angle;
  float FL_speed;
  float FR_speed;
  float RL_speed;
  float RR_speed;
};
control_struct control_data;

typedef struct Confirm_data_send
{
 float confirm_FL_angle;
 float confirm_FR_angle;
 float confirm_RL_angle;
 float confirm_RR_angle;

 float confirm_FL_speed;
 float confirm_FR_speed; 
 float confirm_RL_speed; 
 float confirm_RR_speed;
  
 float measure_FL_speed;
 float measure_FR_speed;
 float measure_RL_speed;
 float measure_RR_speed;

 float PWM_signal_FL_speed;
 float PWM_signal_FR_speed;
 float PWM_signal_RL_speed;
 float PWM_signal_RR_speed;

 float PWM_signal_FL_angle;
 float PWM_signal_FR_angle;
 float PWM_signal_RL_angle;
 float PWM_signal_RR_angle;
 
 float Voltage;
 float Current;
 uint16_t SOC;
 uint16_t Temperature;

 uint8_t MODE;
 
}data_send;
data_send vehicle_data;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  //Serial.print("Trang thai du lieu gui di: ");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Gui thanh cong" : "Gui that bai");
}
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len){
  memcpy(&vehicle_data, incomingData, sizeof(vehicle_data));
}

void setup_esp_now(){
  WiFi.mode(WIFI_STA);
  if(esp_now_init() != ESP_OK){
    //Serial.println("Khoi tao ESP-NOW that bai");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  memcpy(peerInfo.peer_addr, master_addr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if(esp_now_add_peer(&peerInfo) != ESP_OK){
    //Serial.println("Them MASTER vao ds peer that bai");
    return;
  }
}