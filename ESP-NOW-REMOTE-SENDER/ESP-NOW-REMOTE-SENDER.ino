#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi_internal.h>
#include <driver/adc.h>

const uint8_t channel = 6;
const uint8_t broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t packet_data[2] = {0, 0};


int32_t last_signal = 0;
int32_t delta_signal = 0;
uint16_t timing_chain[4096];
volatile uint16_t counter = 0, marker = 0, decoder = 0;
volatile uint8_t active = 0;


uint32_t current_code = 0, last_code = 0;
uint8_t button_down = 0;


void send_button_down(const uint32_t button_code) {
  if (((button_code >> (8 * 3)) | (button_code >> (8 * 2))) & 0xFF == 0xFF && ((button_code >> (8 * 1)) | (button_code >> (8 * 0))) & 0xFF == 0xFF) {
    if (!button_down) {
      packet_data[0] = 0xFF;
      packet_data[1] = button_code;

      esp_now_send(broadcast_addr, packet_data, 2);
      //    Serial.printf("DOWN: %08X\n", button_code);
    }
    button_down = 1;
  }
}

void send_button_up(const uint32_t button_code) {
  if (((button_code >> (8 * 3)) | (button_code >> (8 * 2))) & 0xFF == 0xFF && ((button_code >> (8 * 1)) | (button_code >> (8 * 0))) & 0xFF == 0xFF) {
    if (button_down) {
      packet_data[0] = 0x00;
      packet_data[1] = button_code;

      esp_now_send(broadcast_addr, packet_data, 2);
//      Serial.printf("UP:   %08X\n", button_code);
    }
    button_down = 0;
  }
}
  void IRAM_ATTR pin_toggle() {
    delta_signal = last_signal;
    last_signal = micros();
    if (active == 0) {
      delta_signal = last_signal;
      counter = 0;
      active = 1;
    }
    else if (last_signal - delta_signal > 200) {
      counter++;
      if (last_signal - delta_signal > 30000) {
        marker = counter;
        current_code = 0;
        //      Serial.println("!");
      } else {
        current_code = (current_code << 1) | !!((last_signal - delta_signal) / 1050);
        if (counter - marker == 33 && last_code != current_code) {
          if (last_code) send_button_up(last_code);
          last_code = current_code;
          send_button_down(current_code);
        }
      }
    }
  }


  void setup() {
//    Serial.begin(115200);

    memset(&timing_chain, 0, sizeof(timing_chain));

    pinMode(33, INPUT_PULLUP);
    attachInterrupt(33, pin_toggle, RISING);


    // ESP NOW
    WiFi.enableSTA(true);
    WiFi.setSleep(true);


//    esp_wifi_stop();
//
//    esp_wifi_deinit();
//
//    wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
//    my_config.ampdu_tx_enable = 0;
//    my_config.ampdu_rx_enable = 0;
//
//    esp_wifi_init(&my_config);
//
//    esp_wifi_start();

    //  TRY_ESP_ACTION( esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_ABOVE), "Set channel 40MHz");

    //  TRY_ESP_ACTION( esp_wifi_internal_set_fix_rate(WIFI_IF_STA, true, WIFI_PHY_RATE_MCS7_SGI ), "WIFI_PHY_RATE_MCS7_SGI ");
    //  TRY_ESP_ACTION( esp_wifi_internal_set_fix_rate(WIFI_IF_STA, true, WIFI_PHY_RATE_LORA_250K ), "WIFI_PHY_RATE_LORA_250K ");
    //  TRY_ESP_ACTION( esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR), "WIFI_PROTOCOL_LR");

    esp_now_init();

    //  esp_now_register_send_cb(reinterpret_cast<esp_now_send_cb_t>(sent_callback));
    //  esp_now_register_recv_cb(reinterpret_cast<esp_now_recv_cb_t>(received_callback));

    esp_now_peer_info_t brcst;
    memset(&brcst, 0, sizeof(brcst));
    memcpy(brcst.peer_addr, broadcast_addr, ESP_NOW_ETH_ALEN);
    brcst.channel = 6;
    //  brcst.ifidx = ESP_IF_WIFI_STA;
    esp_now_add_peer(&brcst);

  }

  void loop() {
    int32_t timing = (micros() - last_signal);
    if (active && timing > 100000) {
      //        Serial.print("active: "); Serial.print(active); Serial.print(", timing: "); Serial.print(timing); Serial.print(", counter: "); Serial.println(counter-marker);
      send_button_up(last_code);

      current_code = 0;
      last_code = 0;

      active = 0;
      counter = 0;
      memset(&timing_chain, 0, sizeof(timing_chain));
    }
    if (!active && timing > 10000000) {
      
    WiFi.mode(WIFI_OFF);
    adc_power_off();  // adc power off disables wifi entirely, upstream bug
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0);
////  esp_sleep_enable_ext1_wakeup((uint64_t)(1<<33),ESP_EXT1_WAKEUP_ALL_LOW);
////  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
////  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
////  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  gpio_pulldown_dis(GPIO_NUM_33);
  gpio_pullup_en(GPIO_NUM_33);
  esp_deep_sleep_start();
    }
  }
