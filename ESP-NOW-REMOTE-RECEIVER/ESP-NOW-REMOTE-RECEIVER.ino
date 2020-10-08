#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi_internal.h>
#include "translation_matrix.h"

#define keyboard_mask 0b00000001
#define win_mask    0b00100000
#define alt_mask    0b00010000
#define shift_mask  0b00001000
#define ctrl_mask   0b00000100
#define media_mask  0b01000000
#define syskey_mask 0b10000000
#define direction_mask  0b00000010
#define mouse_click_mask  0b00000100

HardwareSerial SerialOut(1);

const uint8_t channel = 1;
const uint8_t broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//void print_text(const char *input = 0, const uint16_t length = 0) {
//  if (!input && !length) return;
//  for (uint16_t i = 0; i < length; i++) {
//    delay(100);
//    if (input[i] >= 'a' && input[i] <= 'z') {
//      //-'a'+KEYCODE_A
//      SerialOut.write(keyboard_mask);
//      SerialOut.write(input[i] - 'a' + KEYCODE_A);
//      SerialOut.write(keyboard_mask);
//      SerialOut.write(0);
//    } else if (input[i] >= 'A' && input[i] <= 'Z') {
//      //-'A'+KEYCODE_A, modifier: shift
//      SerialOut.write(keyboard_mask | shift_mask);
//      SerialOut.write(input[i] - 'A' + KEYCODE_A);
//      SerialOut.write(keyboard_mask);
//      SerialOut.write(0);
//    } else if (input[i] >= '1' && input[i] <= '0') {
//      //-'1'+KEYCODE_1
//      SerialOut.write(keyboard_mask);
//      SerialOut.write(input[i] - '1' + KEYCODE_1);
//      SerialOut.write(keyboard_mask);
//      SerialOut.write(0);
//    } else if (input[i] >= '!' && input[i] <= ')') {
//      //-'!'+KEYCODE_1, modifier: shift
//      SerialOut.write(keyboard_mask | shift_mask);
//      SerialOut.write(input[i] - '!' + KEYCODE_1);
//      SerialOut.write(keyboard_mask);
//      SerialOut.write(0);
//    } else if (input[i] == ' ') {
//      //KEYCODE_SPACE
//      SerialOut.write(keyboard_mask);
//      SerialOut.write(KEYCODE_SPACE);
//      SerialOut.write(keyboard_mask);
//      SerialOut.write(0);
//    } else if (input[i] == '+' || input[i] == '=') {
//      //KEYCODE_EQUAL
//      SerialOut.write(keyboard_mask|(input[i] == '+' ? shift_mask:0));
//      SerialOut.write(KEYCODE_EQUAL);
//      SerialOut.write(keyboard_mask);
//      SerialOut.write(0);
//    } else if (input[i] == '_' || input[i] == '-') {
//      //KEYCODE_EQUAL
//      SerialOut.write(keyboard_mask|(input[i] == '_' ? shift_mask:0));
//      SerialOut.write(KEYCODE_MINUS);
//      SerialOut.write(keyboard_mask);
//      SerialOut.write(0);
//    } else if (input[i] == '\n' || input[i] == '\0') {
//      //KEYCODE_ENTER
//      SerialOut.write(keyboard_mask);
//      SerialOut.write(KEYCODE_ENTER);
//      SerialOut.write(keyboard_mask);
//      SerialOut.write(0);
//    }
//  }
//}

uint8_t last_data[2]={0,0};

void received_callback(const uint8_t *mac, const uint8_t *data, uint8_t len) {
  uint8_t state = !!data[0];
  uint8_t ID = data[1];

  if (data[0]==last_data[0] && data[1]==last_data[1]) {
    Serial.print("!");
    return;
  }
  last_data[0]=data[0];
  last_data[1]=data[1];
  //state[1/0]: [syskey / no_syskey] [media / no_media] [win / no_win] [alt / no_alt] [shift / no_shift] [ctrl-click / no_ctrl-move] [down-horizontal / up-vertical] [keyboard / mouse]

  //  if (ID == 0x2A) { //guide
  //    if (state) {
  //      String output = "Text = On Screen Keyboard\nMenu = Right Click\n0 = Middle Click\nPower = Alt+F4\nInput = Win + Tab\nQ.View = Alt + Tab\nList = Tab\nT.Opt and Subtitle = Prev Next Tab Firefox\nRed = b\nBlue = w\n";
  //      print_text(output.c_str(), output.length());
  //    }
  //    return;
  //  }


  if (ID == 0x1F || ID == 0x9F || ID == 0x1F || ID == 0x7D || ID == 0xFD || ID == 0xDD || ID == 0x3D || ID == 0xF7 ) { //mouse
    if (state) {
      if (ID == 0x1F) {
        SerialOut.write(1 * direction_mask); SerialOut.write(-2);
      } else if (ID == 0x9F) {
        SerialOut.write(1 * direction_mask); SerialOut.write(2);
      } else if (ID == 0xFD) {
        SerialOut.write(0 * direction_mask); SerialOut.write(-2);
      } else if (ID == 0x7D) {
        SerialOut.write(0 * direction_mask); SerialOut.write(2);
      } else if (ID == 0xDD) {
        SerialOut.write(mouse_click_mask); SerialOut.write(MOUSEBTN_LEFT_MASK);
      } else if (ID == 0x3D) {
        SerialOut.write(mouse_click_mask); SerialOut.write(MOUSEBTN_RIGHT_MASK);
      } else if (ID == 0xF7) {
        SerialOut.write(mouse_click_mask); SerialOut.write(MOUSEBTN_MIDDLE_MASK);
      }
    } else {
      SerialOut.write(0); SerialOut.write(0);
    }
  } else { //keyboard
    if (translation_matrix[ID]) {
      uint8_t modifiers = keyboard_mask;
      if (ID == 0x56)
        modifiers |= syskey_mask * state;
      if (ID == 0xBF || ID == 0x3F || ID == 0x6F || ID == 0x39 || ID == 0x71)
        modifiers |= media_mask * state;
      if (ID == 0xA7 || ID == 0x37 || ID == 0x67 || ID == 0xEF )
        modifiers |= alt_mask * state;
      if (ID == 0x2F || ID == 0xFB)
        modifiers |= win_mask * state;
      if (ID == 0xFB || ID == 0x63 || ID == 0x7B)
        modifiers |= ctrl_mask * state;

      if (ID == 0x6F || ID == 0x39 || ID == 0x71) { //media keys with immediate release
        if (state) {
          SerialOut.write(modifiers);
          SerialOut.write(translation_matrix[ID]);
          SerialOut.write(modifiers);
          SerialOut.write(0);
        }
      } else {
        SerialOut.write(modifiers);
        SerialOut.write(translation_matrix[ID]*state);
      }
    }
  }
  Serial.printf("\nState: %d, ID: %02X", state, ID);
}


void setup() {
  init_matrix();
  Serial.begin(115200);
  SerialOut.begin(9600, SERIAL_8N1, -1, 21);
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);


  esp_wifi_stop();

  esp_wifi_deinit();

  wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
  my_config.ampdu_tx_enable = 0;
  my_config.ampdu_rx_enable = 0;

  esp_wifi_init(&my_config);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_start();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

  //  TRY_ESP_ACTION( esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_STA, true, WIFI_PHY_RATE_54M), "Fixed rate set up");
  //  TRY_ESP_ACTION( esp_wifi_internal_set_fix_rate(ESP_IF_WIFI_STA, true, WIFI_PHY_RATE_LORA_250K), "Fixed rate set up");

  //  TRY_ESP_ACTION( esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR), "WIFI_PROTOCOL_LR");

  esp_now_init();


  //  esp_now_register_send_cb(reinterpret_cast<esp_now_send_cb_t>(sent_callback));
  esp_now_register_recv_cb(reinterpret_cast<esp_now_recv_cb_t>(received_callback));

  esp_now_peer_info_t brcst;
  memset(&brcst, 0, sizeof(brcst));
  memcpy(brcst.peer_addr, broadcast_addr, ESP_NOW_ETH_ALEN);
  brcst.channel = channel;
  brcst.ifidx = ESP_IF_WIFI_STA;
  esp_now_add_peer(&brcst);

}

void loop() {
  // put your main code here, to run repeatedly:

}
