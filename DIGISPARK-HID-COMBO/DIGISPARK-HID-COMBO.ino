// see tutorial at http://learn.adafruit.com/trinket-usb-volume-knob

#include "TrinketHidCombo.h"
#include <SoftSerial_INT0.h>


#define keyboard_mask 0b00000001
#define win_mask    0b00100000
#define alt_mask    0b00010000
#define shift_mask  0b00001000
#define ctrl_mask   0b00000100
#define media_mask  0b01000000
#define syskey_mask 0b10000000
#define direction_mask  0b00000010
#define mouse_click_mask  0b00000100



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

uint8_t state = 0, value = 0, update = 0;
//state[1/0]: [syskey / no_syskey] [media / no_media] [win / no_win] [alt / no_alt] [shift / no_shift] [ctrl-click / no_ctrl-move] [down-horizontal / up-vertical] [keyboard / mouse]

void loop()
{

  if (SerialIn.available() >= 2) {
    state = SerialIn.read();
    value = SerialIn.read();
    update = 1;
  }

  if ( (state & keyboard_mask) && update) { //keyboard
    //update = 0;
    if (state & media_mask) {
      TrinketHidCombo.pressMultimediaKey(value);
      trinket_delay(100);
    } else if (state & syskey_mask) {
      TrinketHidCombo.pressSystemCtrlKey(value);
      update = 0;
    } else {
      TrinketHidCombo.pressKey(((state >> 2) & 0b1111), value);
      update = 0;
    }
  } else if (!(state & keyboard_mask)) { //mouse
    if (state & mouse_click_mask) {
      if (update) {
        TrinketHidCombo.mouseMove(0, 0, value);
        update=0;
      }
    } else {
      int8_t direction_cast = 0;
      direction_cast |= value;
      if (value || update) TrinketHidCombo.mouseMove((!!(state & direction_mask)) * direction_cast, (!(state & direction_mask))*direction_cast, 0);
      update=0;
    }
  }

  TrinketHidCombo.poll();

}
