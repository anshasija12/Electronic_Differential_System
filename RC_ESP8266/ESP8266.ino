#include <ESP8266WiFi.h>
#include <espnow.h>

uint8_t controller[3]; 
uint8_t Address[] = {0x08, 0xD1, 0xF9, 0xEB, 0x90, 0x5C};
 
 typedef struct struct_message {
  uint8_t ign;
  uint8_t acc;
  uint8_t steer;
} struct_message;

struct_message data;

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}
 
void setup() {

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != 0) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
 
  esp_now_register_send_cb(OnDataSent);

  esp_now_add_peer(Address, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}
 
void loop() {

  esp_now_send(Address, (uint8_t *) &controller, sizeof(controller));
  if (Serial.available())
  {
    controller[3] = Serial.readBytes((uint8_t*)controller, sizeof(controller));
    data.ign = controller[0];
    data.acc = controller[1];
    data.steer = controller[2];
    esp_now_send(Address, (uint8_t *) &controller, sizeof(controller));
  }
}