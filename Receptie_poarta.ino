#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Arduino.h>

#define Pin_Open  13
#define Pin_Close 14

#define Limiter_Full_Open    33
#define Limiter_Partial_Open 32
#define Limiter_Full_Close   27

uint8_t senderAddress1[] = {0xa0, 0xa3, 0xb3, 0x97, 0x4a, 0xdc};
uint8_t senderAddress2[] = {0xa0, 0xa3, 0xb3, 0x96, 0x67, 0xb8};
uint8_t key1[] = { 
  0x1F, 0xA7, 0xD3, 0x6C, 
  0x9E, 0x52, 0xB8, 0x4A, 
  0xF4, 0x3D, 0x7B, 0x89, 
  0xC1, 0x25, 0xE6, 0x90 
};

typedef struct incomingMessage{
  bool holdOpenGate;
  bool pressOpenGate;
  bool pressCloseGate;
} incomingMessage;

incomingMessage gate_control1;
incomingMessage gate_control2;

bool control[6];

esp_now_peer_info_t peerInfo1;
esp_now_peer_info_t peerInfo2;

bool compareMac(const uint8_t* mac1, const uint8_t* mac2)
{
  for(int i=0; i<6; i++)
  {
    if (mac1[i]!=mac2[i])
      return 0;
  }
  return 1;
}

void OnDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  if (compareMac(recvInfo->src_addr, senderAddress1)==1)
  {
    memcpy(&gate_control1, incomingData, sizeof(incomingMessage));
  }
  else if (compareMac(recvInfo->src_addr, senderAddress2)==1)
  {
    memcpy(&gate_control2, incomingData, sizeof(incomingMessage));
  }
  //Serial.print("Bytes received: ");
  //Serial.println(len);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(Pin_Open, OUTPUT);
  pinMode(Pin_Close, OUTPUT);

  digitalWrite(Pin_Open, LOW);
  digitalWrite(Pin_Close, LOW);

  pinMode(Limiter_Full_Open, INPUT_PULLUP);
  pinMode(Limiter_Partial_Open, INPUT_PULLUP);
  pinMode(Limiter_Full_Close, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);

  esp_now_init();
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error couldn't initialize esp-now");
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  memcpy(peerInfo1.peer_addr, senderAddress1, 6);
  peerInfo1.channel = 0;  
  peerInfo1.encrypt = true;
  //peerInfo1.encrypt = false;
  memcpy(peerInfo1.lmk, key1, 16);

  if (esp_now_add_peer(&peerInfo1) != ESP_OK)
  {
    Serial.println("Failed to add peer");
  }
  memcpy(peerInfo2.peer_addr, senderAddress2, 6);
  peerInfo2.channel = 0;  
  peerInfo2.encrypt = true;
  //peerInfo2.encrypt = false;
  memcpy(peerInfo2.lmk, key1, 16);

  if (esp_now_add_peer(&peerInfo2) != ESP_OK)
  {
    Serial.println("Failed to add peer");
  }
}

void transfer_data()
{
  control[0]=gate_control1.holdOpenGate;
  control[1]=gate_control1.pressOpenGate;
  control[2]=gate_control1.pressCloseGate;
  control[3]=gate_control2.holdOpenGate;
  control[4]=gate_control2.pressOpenGate;
  control[5]=gate_control2.pressCloseGate;
}

bool holdOpenLogic()
{
  return ((control[0] || control[3]) && control[2]==0 && control[5]==0 && (digitalRead(Limiter_Full_Open)==HIGH));
}

void holdOpenFunction()
{
  delay(200);
  transfer_data();
  while ((control[0] || control[3]) && control[1]==0 && control[2]==0 && control[4]==0 && control[5]==0 && (digitalRead(Limiter_Full_Open)==HIGH))
  {
    digitalWrite(Pin_Open, HIGH);
    transfer_data();
  }
  digitalWrite(Pin_Open, LOW);
  delay(200);
  transfer_data();
}

bool pressOpenPartialLogic()
{
  return ((control[1] || control[4]) && control[2]==0 && control[5]==0 && 
  (digitalRead(Limiter_Partial_Open)==HIGH && digitalRead(Limiter_Full_Open)==HIGH));
}

bool pressOpenFullLogic()
{
  return ((control[1] || control[4]) && control[2]==0 && control[5]==0 && 
  (digitalRead(Limiter_Full_Open)==HIGH && digitalRead(Limiter_Partial_Open)==LOW));
}

void pressOpenFunction()
{
  delay(400);
  if (pressOpenPartialLogic())
  {
    transfer_data();
    while(pressOpenPartialLogic())
    {
      transfer_data();
    }
    while (control[0]==0 && control[1]==0 && control[2]==0 && control[3]==0 && control[4]==0 && control[5]==0 && 
    (digitalRead(Limiter_Partial_Open)==HIGH && digitalRead(Limiter_Full_Open)==HIGH))
    {
      digitalWrite(Pin_Open, HIGH);
      transfer_data();
    }
  }
  else if (pressOpenFullLogic())
  {
    transfer_data();
    while(pressOpenFullLogic())
    {
      transfer_data();
    }
    while (control[0]==0 && control[1]==0 && control[2]==0 && control[3]==0 && control[4]==0 && control[5]==0 && 
    (digitalRead(Limiter_Partial_Open)==LOW && digitalRead(Limiter_Full_Open)==HIGH))
    {
      digitalWrite(Pin_Open, HIGH);
      transfer_data();
    }
  }
  digitalWrite(Pin_Open, LOW);
  delay(200);
  transfer_data();
}

bool pressCloseLogic()
{
  return ((control[2] || control[5]) && control[0]==0 && control[1]==0 && control[3]==0 && control[4]==0 && digitalRead(Limiter_Full_Close)==HIGH);
}

void pressCloseFunction()
{
  delay(400);
  if (pressCloseLogic())
  {
    transfer_data();
    while (pressCloseLogic())
    {
      transfer_data();
    }
    while (control[0]==0 && control[1]==0 && control[2]==0 && control[3]==0 && control[4]==0 && control[5]==0 && digitalRead(Limiter_Full_Close)==HIGH)
    {
      digitalWrite(Pin_Close, HIGH);
      transfer_data();
    }
  }
  digitalWrite(Pin_Close, LOW);
  delay(200);
  transfer_data();
}

void loop() {
  transfer_data();
  if (holdOpenLogic())
  {
    holdOpenFunction();
  }
  else if (pressOpenPartialLogic())
  {
    pressOpenFunction();
  }
  else if (pressOpenFullLogic())
  {
    pressOpenFunction();
  }
  else if (pressCloseLogic())
  {
    pressCloseFunction();
  }
}