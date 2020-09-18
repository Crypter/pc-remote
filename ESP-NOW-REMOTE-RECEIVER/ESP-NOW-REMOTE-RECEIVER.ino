#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi_internal.h>
#include "translation_matrix.h"

HardwareSerial SerialOut(1);

const uint8_t channel = 6;
const uint8_t broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void received_callback(const uint8_t *mac, const uint8_t *data, uint8_t len) {
  uint8_t state = !!data[0];
  uint8_t ID = data[1];
  //state[1/0]: [modifier-click / regular-move] [down-horizontal / up-vertical] [keyboard / mouse]

  if (ID == 0x1F || ID == 0x9F || ID == 0x1F || ID == 0x7D || ID == 0xFD || ID == 0xDD) { //mouse
    if (state) {
      if (ID == 0x1F) {
        SerialOut.write(0b010); SerialOut.write(-2);
      } else if (ID == 0x9F) {
        SerialOut.write(0b010); SerialOut.write(2);
      } else if (ID == 0xFD) {
        SerialOut.write(0b000); SerialOut.write(-2);
      } else if (ID == 0x7D) {
        SerialOut.write(0b000); SerialOut.write(2);
      } else if (ID == 0xDD) {
        SerialOut.write( (0b110) ); SerialOut.write(MOUSEBTN_LEFT_MASK);
      }
    } else {
        if (ID == 0xDD) {
        SerialOut.write( (0b100) ); SerialOut.write(MOUSEBTN_LEFT_MASK);
      } else { 
        SerialOut.write(0b001); SerialOut.write(0);
      }
    }
  } else { //keyboard
    if (translation_matrix[ID]) {
      if (ID == 0xBF || ID == 0x3F || ID == 0x6F)
        SerialOut.write( (0b1001) | (state << 1) );
      else
        SerialOut.write( (0b0001) | (state << 1) );
      SerialOut.write(translation_matrix[ID]);
    }
    //else if (ID==0x3F) {SerialOut.write(0); SerialOut.write(0xEA); }
  }



  Serial.printf("State: %d, ID: %02X\n", !!state, ID);
}


void setup() {
  init_matrix();
  Serial.begin(115200);
  SerialOut.begin(9600, SERIAL_8N1, -1, 21);

  WiFi.enableSTA(true);
  WiFi.setSleep(false);


  esp_wifi_stop();

  esp_wifi_deinit();

  wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
  my_config.ampdu_tx_enable = 0;
  my_config.ampdu_rx_enable = 0;

  esp_wifi_init(&my_config);

  esp_wifi_start();

  //  TRY_ESP_ACTION( esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE), "Set channel");

  //  TRY_ESP_ACTION( esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_STA, true, WIFI_PHY_RATE_54M), "Fixed rate set up");
  //  TRY_ESP_ACTION( esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_STA, true, WIFI_PHY_RATE_LORA_250K), "Fixed rate set up");

  //  TRY_ESP_ACTION( esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR), "WIFI_PROTOCOL_LR");

  esp_now_init();


  //  esp_now_register_send_cb(reinterpret_cast<esp_now_send_cb_t>(sent_callback));
  esp_now_register_recv_cb(reinterpret_cast<esp_now_recv_cb_t>(received_callback));

  esp_now_peer_info_t brcst;
  memset(&brcst, 0, sizeof(brcst));
  memcpy(brcst.peer_addr, broadcast_addr, ESP_NOW_ETH_ALEN);
  brcst.channel = 6;
  brcst.ifidx = ESP_IF_WIFI_STA;
  esp_now_add_peer(&brcst);

}

void loop() {
  // put your main code here, to run repeatedly:

}
