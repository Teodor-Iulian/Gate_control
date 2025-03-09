#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

#define Button_Hold_Open 27
#define Button_Press_Close 33
#define Button_Press_Open 32

uint8_t broadcastAddress[] = {0xd4, 0x8a, 0xfc, 0xa3, 0x58, 0xb8};
uint8_t key1[] = { 
  0x1F, 0xA7, 0xD3, 0x6C, 
  0x9E, 0x52, 0xB8, 0x4A, 
  0xF4, 0x3D, 0x7B, 0x89, 
  0xC1, 0x25, 0xE6, 0x90 
};

typedef struct message{
  bool holdOpen;
  bool pressOpen;
  bool pressClose;
} message;

message commands;

String success;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    success = "Delivery Success";
  }
  else{
    success = "Delivery Fail";
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(Button_Hold_Open, INPUT_PULLUP);
  pinMode(Button_Press_Open, INPUT_PULLUP);
  pinMode(Button_Press_Close, INPUT_PULLUP);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(14, OUTPUT);

  WiFi.mode(WIFI_STA);
  
  esp_now_init();
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error couldn't initialize esp-now");
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = true;
  //peerInfo.encrypt = false;
  memcpy(peerInfo.lmk, key1, 16);

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
  }
};

void loop() {
  commands.holdOpen=!digitalRead(Button_Hold_Open);
  commands.pressOpen=!digitalRead(Button_Press_Open);
  commands.pressClose=!digitalRead(Button_Press_Close);

  digitalWrite(12, commands.holdOpen);
  digitalWrite(13, commands.pressOpen);
  digitalWrite(14, commands.pressClose);

  Serial.print("holdOpen: ");
  Serial.println(commands.holdOpen);
  Serial.print("pressOpen: ");
  Serial.println(commands.pressOpen);
  Serial.print("pressClose: ");
  Serial.println(commands.pressClose);
  
  esp_now_send(broadcastAddress, (uint8_t *) &commands, sizeof(message));

}
