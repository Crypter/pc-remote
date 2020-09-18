// see tutorial at http://learn.adafruit.com/trinket-usb-volume-knob

#include "TrinketHidCombo.h"
#include <SoftSerial_INT0.h>

void trinket_delay(uint32_t value) {
  uint32_t start = millis();
  while (millis() - start < value) {
    TrinketHidCombo.poll();
    delay(1);
  }
}

SoftSerial SerialIn(2, -1);           // Software serial port for control the BLE module

void setup()
{
  SerialIn.begin(9600);
  TrinketHidCombo.begin(); // start the USB device engine and enumerate
}

uint8_t state = 0, value = 0, repeat_action = 0, update = 0;
//state[1/0]: [mediakey / regular] [modifier-click / regular-move] [down-horizontal / up-vertical] [keyboard / mouse]
uint8_t is_keyboard = 0, direction = 0, is_modifier = 0, is_media = 0;

void loop()
{

  if (SerialIn.available() >= 2) {
    state = SerialIn.read();
    value = SerialIn.read();
    update = 1;
    is_keyboard = state & 1;
    direction = (state >> 1) & 1;
    is_modifier = (state >> 2) & 1;
    is_media = (state >> 3) & 1;
  }

  if (is_keyboard && update) { //keyboard
    //update = 0;
    if (!direction) {
      TrinketHidCombo.pressKey(0, 0);
    } else {
      if (is_modifier) {
        TrinketHidCombo.pressKey(KEYCODE_MOD_LEFT_ALT, value);
        update = 0;
      } else if (is_media) {
        TrinketHidCombo.pressMultimediaKey(value);
        trinket_delay(100);
      } else {
        if (value >= 0 && value <= 3) {
          TrinketHidCombo.pressSystemCtrlKey(value);
          update = 0;
        } else {
          TrinketHidCombo.pressKey(0, value);
          update = 0;
        }
      }
    }
  } else if (!is_keyboard) { //mouse
    if (is_modifier) {
      if (update) {
        if (direction) TrinketHidCombo.mouseMove(0, 0, value);
        else TrinketHidCombo.mouseMove(0, 0, 0);
      }
    } else {
      int8_t direction_cast = 0;
      direction_cast |= value;
      if (value) TrinketHidCombo.mouseMove(direction * direction_cast, (!direction)*direction_cast, 0);
    }
  }

  TrinketHidCombo.poll();

}
