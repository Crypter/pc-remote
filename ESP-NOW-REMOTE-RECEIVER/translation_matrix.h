// use these masks with the "move" function
#define MOUSEBTN_LEFT_MASK    0x01
#define MOUSEBTN_RIGHT_MASK   0x02
#define MOUSEBTN_MIDDLE_MASK  0x04

// LED state masks
#define KB_LED_NUM      0x01
#define KB_LED_CAPS     0x02
#define KB_LED_SCROLL   0x04

// some convenience definitions for modifier keys
#define KEYCODE_MOD_LEFT_CONTROL  0x01
#define KEYCODE_MOD_LEFT_SHIFT    0x02
#define KEYCODE_MOD_LEFT_ALT    0x04
#define KEYCODE_MOD_LEFT_GUI    0x08
#define KEYCODE_MOD_RIGHT_CONTROL 0x10
#define KEYCODE_MOD_RIGHT_SHIFT   0x20
#define KEYCODE_MOD_RIGHT_ALT   0x40
#define KEYCODE_MOD_RIGHT_GUI   0x80

// some more keycodes
#define KEYCODE_LEFT_CONTROL  0xE0
#define KEYCODE_LEFT_SHIFT    0xE1
#define KEYCODE_LEFT_ALT    0xE2
#define KEYCODE_LEFT_GUI    0xE3
#define KEYCODE_RIGHT_CONTROL 0xE4
#define KEYCODE_RIGHT_SHIFT   0xE5
#define KEYCODE_RIGHT_ALT   0xE6
#define KEYCODE_RIGHT_GUI   0xE7
#define KEYCODE_1       0x1E
#define KEYCODE_2       0x1F
#define KEYCODE_3       0x20
#define KEYCODE_4       0x21
#define KEYCODE_5       0x22
#define KEYCODE_6       0x23
#define KEYCODE_7       0x24
#define KEYCODE_8       0x25
#define KEYCODE_9       0x26
#define KEYCODE_0       0x27
#define KEYCODE_A       0x04
#define KEYCODE_B       0x05
#define KEYCODE_C       0x06
#define KEYCODE_D       0x07
#define KEYCODE_E       0x08
#define KEYCODE_F       0x09
#define KEYCODE_G       0x0A
#define KEYCODE_H       0x0B
#define KEYCODE_I       0x0C
#define KEYCODE_J       0x0D
#define KEYCODE_K       0x0E
#define KEYCODE_L       0x0F
#define KEYCODE_M       0x10
#define KEYCODE_N       0x11
#define KEYCODE_O       0x12
#define KEYCODE_P       0x13
#define KEYCODE_Q       0x14
#define KEYCODE_R       0x15
#define KEYCODE_S       0x16
#define KEYCODE_T       0x17
#define KEYCODE_U       0x18
#define KEYCODE_V       0x19
#define KEYCODE_W       0x1A
#define KEYCODE_X       0x1B
#define KEYCODE_Y       0x1C
#define KEYCODE_Z       0x1D
#define KEYCODE_COMMA     0x36
#define KEYCODE_PERIOD      0x37
#define KEYCODE_MINUS     0x2D
#define KEYCODE_EQUAL     0x2E
#define KEYCODE_BACKSLASH   0x31
#define KEYCODE_SQBRAK_LEFT   0x2F
#define KEYCODE_SQBRAK_RIGHT  0x30
#define KEYCODE_SLASH     0x38
#define KEYCODE_F1        0x3A
#define KEYCODE_F2        0x3B
#define KEYCODE_F3        0x3C
#define KEYCODE_F4        0x3D
#define KEYCODE_F5        0x3E
#define KEYCODE_F6        0x3F
#define KEYCODE_F7        0x40
#define KEYCODE_F8        0x41
#define KEYCODE_F9        0x42
#define KEYCODE_F10       0x43
#define KEYCODE_F11       0x44
#define KEYCODE_F12       0x45
#define KEYCODE_APP       0x65
#define KEYCODE_ENTER     0x28
#define KEYCODE_BACKSPACE   0x2A
#define KEYCODE_ESC       0x29
#define KEYCODE_TAB       0x2B
#define KEYCODE_SPACE     0x2C
#define KEYCODE_INSERT      0x49
#define KEYCODE_HOME      0x4A
#define KEYCODE_PAGE_UP     0x4B
#define KEYCODE_DELETE      0x4C
#define KEYCODE_END       0x4D
#define KEYCODE_PAGE_DOWN   0x4E
#define KEYCODE_PRINTSCREEN   0x46
#define KEYCODE_ARROW_RIGHT   0x4F
#define KEYCODE_ARROW_LEFT    0x50
#define KEYCODE_ARROW_DOWN    0x51
#define KEYCODE_ARROW_UP    0x52

// multimedia keys
#define MMKEY_KB_VOL_UP     0x80 // do not use
#define MMKEY_KB_VOL_DOWN   0x81 // do not use
#define MMKEY_VOL_UP      0xE9
#define MMKEY_VOL_DOWN      0xEA
#define MMKEY_SCAN_NEXT_TRACK 0xB5
#define MMKEY_SCAN_PREV_TRACK 0xB6
#define MMKEY_STOP        0xB7
#define MMKEY_PLAYPAUSE     0xCD
#define MMKEY_MUTE        0xE2
#define MMKEY_BASSBOOST     0xE5
#define MMKEY_LOUDNESS      0xE7
#define MMKEY_KB_EXECUTE    0x74
#define MMKEY_KB_HELP     0x75
#define MMKEY_KB_MENU     0x76
#define MMKEY_KB_SELECT     0x77
#define MMKEY_KB_STOP     0x78
#define MMKEY_KB_AGAIN      0x79
#define MMKEY_KB_UNDO     0x7A
#define MMKEY_KB_CUT      0x7B
#define MMKEY_KB_COPY     0x7C
#define MMKEY_KB_PASTE      0x7D
#define MMKEY_KB_FIND     0x7E
#define MMKEY_KB_MUTE     0x7F // do not use

// system control keys
#define SYSCTRLKEY_POWER    0x01
#define SYSCTRLKEY_SLEEP    0x02
#define SYSCTRLKEY_WAKE     0x03

uint8_t translation_matrix[0xFF];
void init_matrix(){
  memset(translation_matrix, 0, 0xFF);

translation_matrix[0xEF]=KEYCODE_F4; //alt
translation_matrix[0x56]=SYSCTRLKEY_SLEEP; //eco

translation_matrix[0xFF]=KEYCODE_PAGE_UP;
translation_matrix[0x7F]=KEYCODE_PAGE_DOWN;

translation_matrix[0x37]=KEYCODE_PAGE_UP; //alt
translation_matrix[0x67]=KEYCODE_PAGE_DOWN; //alt


translation_matrix[0x7B]=KEYCODE_PAGE_UP; //ctrl
translation_matrix[0x63]=KEYCODE_PAGE_DOWN; //ctrl

translation_matrix[0xBF]=MMKEY_VOL_UP;
translation_matrix[0x3F]=MMKEY_VOL_DOWN;
translation_matrix[0x6F]=MMKEY_MUTE;
translation_matrix[0x39]=MMKEY_PLAYPAUSE;
translation_matrix[0x71]=MMKEY_STOP;


translation_matrix[0x57]=KEYCODE_ENTER;
translation_matrix[0xB7]=KEYCODE_ARROW_UP;
translation_matrix[0xE7]=KEYCODE_ARROW_DOWN;
translation_matrix[0xD7]=KEYCODE_ARROW_LEFT;
translation_matrix[0x97]=KEYCODE_ARROW_RIGHT;

translation_matrix[0xEB]=KEYCODE_ESC;
translation_matrix[0x35]=KEYCODE_TAB;
translation_matrix[0xA7]=KEYCODE_TAB; //alt
translation_matrix[0x2F]=KEYCODE_TAB; //win
translation_matrix[0xF3]=KEYCODE_F11;
translation_matrix[0xF0]=KEYCODE_F5;
translation_matrix[0xB1]=KEYCODE_B;
translation_matrix[0x79]=KEYCODE_W;

translation_matrix[0xFB]=KEYCODE_O; //ctrl+win - on screen keyboard
translation_matrix[0x87]=0x8A; //WWW - on screen keyboard


}


//EF  on-off
//56  energy
//F3  av. mode
//2F  input
//F0  tv/rad
//77  1
//B7  2
//37  3
//D7  4
//57  5
//97  6
//17  7
//E7  8
//67  9
//F7  0
//35  list
//A7  quick view
//BF  vol +
//3F  vol- 
//FF  prog. +
//7F  prog. - 
//2A  guide
//3D  menu
//87  fav
//61  ratio
//6F  mute
//FD  arrow up
//7D  arrow down
//9F  arrow right
//1F  arrow left
//DD  ok button
//EB  back
//AA  info
//25  exit
//B1  red button
//71  green button
//39  yellow button
//79  blue button 
//FB  text
//7B  t.opot
//63  subtitle
//72  stop
//F2  play
//A2  pause
//0E  rew
//8E  forward
//76  ad
