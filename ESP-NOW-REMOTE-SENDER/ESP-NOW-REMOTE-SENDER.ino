#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi_internal.h>
#include <driver/adc.h>

const uint8_t channel = 1;
const uint8_t broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t packet_data[2] = {0, 0};

uint8_t wifi_state = 0;
uint8_t wifi_sent = 1;
uint8_t data_to_send = 0;

int32_t last_signal = 0;
int32_t last_send = 0;
int32_t delta_signal = 0;
uint16_t timing_chain[4096];
volatile uint16_t counter = 0, marker = 0, decoder = 0;
volatile uint8_t active = 0;


uint32_t current_code = 0, last_code = 0;
uint8_t button_down = 0;

void packet_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  wifi_sent++;
//  Serial.println("ACK");
//  Serial.flush();
}

void stop_wifi() {
  if (wifi_state && wifi_sent>=2){
  esp_now_deinit();
  esp_wifi_stop();
  esp_wifi_set_promiscuous(false);
  WiFi.mode( WIFI_MODE_NULL );
  adc_power_off();  // adc power off disables wifi entirely, upstream bug
  wifi_state = 0;
  }
}


void start_wifi() {
  // ESP NOW
  if (!wifi_state){
  adc_power_on();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_start();
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);

  esp_now_init();
  esp_now_peer_info_t brcst;
  memset(&brcst, 0, sizeof(brcst));
  memcpy(brcst.peer_addr, broadcast_addr, ESP_NOW_ETH_ALEN);
  brcst.channel = channel;
  brcst.ifidx = ESP_IF_WIFI_STA;
  esp_now_add_peer(&brcst);
  esp_now_register_send_cb(packet_sent);
  wifi_state = 1;
  }
}


void send_data() {
  if (data_to_send) {
    start_wifi();
    wifi_sent=0;
    esp_now_send(broadcast_addr, packet_data, 2);
    esp_now_send(broadcast_addr, packet_data, 2);
    data_to_send = 0;
    last_send=millis();
  }
  stop_wifi();
}

void send_button_down(const uint32_t button_code) {
  if (((button_code >> (8 * 3)) | (button_code >> (8 * 2))) & 0xFF == 0xFF && ((button_code >> (8 * 1)) | (button_code >> (8 * 0))) & 0xFF == 0xFF) {
    if (!button_down) {
      packet_data[0] = 0xFF;
      packet_data[1] = button_code;

      data_to_send = 1;

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

      data_to_send = 1;
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


void IRAM_ATTR debug_button() {
  data_to_send=1;
  if (digitalRead(0)) send_button_up(0x6F); //mute
  else send_button_down(0x6F);
  Serial.println(digitalRead(0));
}

void setup() {
//      Serial.begin(921600);

  memset(&timing_chain, 0, sizeof(timing_chain));

  pinMode(33, INPUT_PULLUP);
  attachInterrupt(33, pin_toggle, RISING);
//pinMode(0, INPUT_PULLUP);
//attachInterrupt(0, debug_button, CHANGE);

}

void loop() {

  send_data();
//  return;

  int32_t timing = (micros() - last_signal);
//Serial.print("a: "); Serial.print(active);
//Serial.print(", t: "); Serial.print(timing);
//Serial.print(", s: "); Serial.print(last_signal - delta_signal);
//Serial.print(", d: "); Serial.print(data_to_send);
//Serial.print(", w: "); Serial.print(wifi_state);
//Serial.print(", c: "); Serial.println(counter-marker);
//Serial.flush();
  if (active && timing > 50000) {
    send_button_up(last_code);

    current_code = 0;
    last_code = 0;

    active = 0;
    counter = 0;
    marker=0;
    memset(&timing_chain, 0, sizeof(timing_chain));
  }

  if (!active && !data_to_send && !wifi_state) {
    esp_sleep_enable_timer_wakeup(5*60*(1000000)); //5min
    gpio_wakeup_enable(GPIO_NUM_33, GPIO_INTR_LOW_LEVEL );
    esp_sleep_enable_gpio_wakeup();
//    Serial.println("L");
//    Serial.flush();
    esp_light_sleep_start();
    
    esp_sleep_source_t wakeup_id = esp_sleep_get_wakeup_cause();
    if ( wakeup_id == ESP_SLEEP_WAKEUP_TIMER){ //timer wakeup!
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
      esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0);
      ////  esp_sleep_enable_ext1_wakeup((uint64_t)(1<<33),ESP_EXT1_WAKEUP_ALL_LOW);
      ////  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
      ////  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
      ////  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
      gpio_pulldown_dis(GPIO_NUM_33);
      gpio_pullup_en(GPIO_NUM_33);
//    Serial.println("D");
//    Serial.flush();
      esp_deep_sleep_start();
    }
  }


}
