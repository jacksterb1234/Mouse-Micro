/*
  PMW3389DM-Mouse.ino
  Based on BenMakesEverything PMW3389_Mouse project.
  Updated mapping:
    Btn1 (5) = Left
    Btn2 (4) = Right
    Btn3 (6) = Middle
    Btn4 (7) = DPI cycle
    Btn5 (8) = Back
    Btn6 (9) = Forward
  No LEDs, no keyboard buttons.
*/

#include <SPI.h>
#include <RotaryEncoder.h>

#define ADVANCE_MODE

#ifdef ADVANCE_MODE
#include <AdvMouse.h>
#define MOUSE_BEGIN       AdvMouse.begin()
#define MOUSE_PRESS(x)    AdvMouse.press_(x)
#define MOUSE_RELEASE(x)  AdvMouse.release_(x)
#else
#include <Mouse.h>
#define MOUSE_BEGIN       Mouse.begin()
#define MOUSE_PRESS(x)    Mouse.press(x)
#define MOUSE_RELEASE(x)  Mouse.release(x)
#endif

#if defined(AVR)
#include <avr/pgmspace.h>
#else
#include <pgmspace.h>
#endif

// ==== Configuration ====
#define CPI      1000
#define DEBOUNCE 5
#define NUMCPI   7

#define NUMBTN  6   // Left, Right, Middle, Back, Forward, (we treat DPI separately)

// Your pin mapping:
#define Btn1_Pin 5   // Left
#define Btn2_Pin 4   // Right
#define Btn3_Pin 6   // Middle
#define Btn4_Pin 7   // DPI button
#define Btn5_Pin 8   // Back
#define Btn6_Pin 9   // Forward

// Encoder pins
#define PIN_IN1 A3
#define PIN_IN2 A2

// PMW3389 pins (per your schematic: SS on B6 -> D10, RST tied to board reset)
const int ncs   = 10;  // SS on B6 -> D10

// ==== PMW3389 Registers (unchanged) ====
#define Product_ID         0x00
#define Revision_ID        0x01
#define Motion             0x02
#define Delta_X_L          0x03
#define Delta_X_H          0x04
#define Delta_Y_L          0x05
#define Delta_Y_H          0x06
#define SQUAL              0x07
#define Raw_Data_Sum       0x08
#define Maximum_Raw_data   0x09
#define Minimum_Raw_data   0x0A
#define Shutter_Lower      0x0B
#define Shutter_Upper      0x0C
#define Ripple_Control     0x0D
#define Resolution_L       0x0E
#define Resolution_H       0x0F
#define Config2            0x10
#define Angle_Tune         0x11
#define Frame_Capture      0x12
#define SROM_Enable        0x13
#define Run_Downshift      0x14
#define Rest1_Rate_Lower   0x15
#define Rest1_Rate_Upper   0x16
#define Rest1_Downshift    0x17
#define Rest2_Rate_Lower   0x18
#define Rest2_Rate_Upper   0x19
#define Rest2_Downshift    0x1A
#define Rest3_Rate_Lower   0x1B
#define Rest3_Rate_Upper   0x1C
#define Observation        0x24
#define Data_Out_Lower     0x25
#define Data_Out_Upper     0x26
#define SROM_ID            0x2A
#define Min_SQ_Run         0x2B
#define Raw_Data_Threshold 0x2C
#define Control2           0x2D
#define Config5_L          0x2E
#define Config5_H          0x2F
#define Power_Up_Reset     0x3A
#define Shutdown           0x3B
#define Inverse_Product_ID 0x3F
#define LiftCutoff_Cal3    0x41
#define Angle_Snap         0x42
#define LiftCutoff_Cal1    0x4A
#define Motion_Burst       0x50
#define SROM_Load_Burst    0x62
#define Lift_Config        0x63
#define Raw_Data_Burst     0x64
#define LiftCutoff_Cal2    0x65
#define LiftCutoff_Cal_Timeout 0x71
#define LiftCutoff_Cal_Min_Length 0x72
#define PWM_Period_Cnt     0x73
#define PWM_Width_Cnt      0x74

// ==== Globals ====
RotaryEncoder *encoder = nullptr;

// Mouse buttons: Left, Right, Middle, Back, Forward
int     Btn_pins[NUMBTN]    = {Btn1_Pin, Btn2_Pin, Btn3_Pin, Btn5_Pin, Btn6_Pin};
bool    Btns[NUMBTN]        = {false, false, false, false, false, false};
uint8_t Btn_buffers[NUMBTN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
char    Btn_keys[NUMBTN]    = {MOUSE_LEFT, MOUSE_RIGHT, MOUSE_MIDDLE, MOUSE_BACK, MOUSE_FORWARD, 0};

// DPI button separate handling (Btn4)
bool    DpiBtn        = false;
uint8_t DpiBtn_buffer = 0xFF;

unsigned long Cpis[NUMCPI] = {400, 600, 800, 1000, 1200, 1400, 1600};
struct CpiUpdater {
  bool    target_set;
  bool    updated;
  uint8_t target_cpi_index;
};
CpiUpdater CpiUpdate = {false, false, 3};

byte         initComplete = 0;
bool         inBurst      = false;
bool         reportSQ     = false;
int16_t      dx, dy;
unsigned long lastTS;
unsigned long lastButtonCheck = 0;
unsigned long curTime;

// SROM firmware
extern const unsigned short firmware_length;
extern const unsigned char  firmware_data[];

// ==== SPI helpers ====
void adns_com_begin() { digitalWrite(ncs, LOW); }
void adns_com_end()   { digitalWrite(ncs, HIGH); }

byte adns_read_reg(byte reg_addr) {
  adns_com_begin();
  SPI.transfer(reg_addr & 0x7F);
  delayMicroseconds(35);
  byte data = SPI.transfer(0x00);
  delayMicroseconds(1);
  adns_com_end();
  delayMicroseconds(19);
  return data;
}

void adns_write_reg(byte reg_addr, byte data) {
  adns_com_begin();
  SPI.transfer(reg_addr | 0x80);
  SPI.transfer(data);
  delayMicroseconds(20);
  adns_com_end();
  delayMicroseconds(100);
}

void adns_upload_firmware() {
  adns_write_reg(Config2, 0x00);
  adns_write_reg(SROM_Enable, 0x1D);
  delay(10);
  adns_write_reg(SROM_Enable, 0x18);

  adns_com_begin();
  SPI.transfer(SROM_Load_Burst | 0x80);
  delayMicroseconds(15);

  for (int i = 0; i < firmware_length; i++) {
    unsigned char c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns_com_end();

  adns_read_reg(SROM_ID);
  adns_write_reg(Config2, 0x00);
}

void setCPI(int cpi) {
  unsigned cpival = cpi / 50;
  adns_com_begin();
  adns_write_reg(Resolution_L, (cpival & 0xFF));
  adns_write_reg(Resolution_H, ((cpival >> 8) & 0xFF));
  adns_com_end();
}

void performStartup(void) {
  adns_com_end();
  adns_com_begin();
  adns_com_end();

  adns_write_reg(Shutdown, 0xB6);
  delay(300);

  adns_com_begin();
  delayMicroseconds(40);
  adns_com_end();
  delayMicroseconds(40);

  adns_write_reg(Power_Up_Reset, 0x5A);
  delay(50);

  adns_read_reg(Motion);
  adns_read_reg(Delta_X_L);
  adns_read_reg(Delta_X_H);
  adns_read_reg(Delta_Y_L);
  adns_read_reg(Delta_Y_H);

  adns_upload_firmware();
  delay(10);

  setCPI(Cpis[CpiUpdate.target_cpi_index]);
}

// ==== Setup ====
void setup() {
  pinMode(ncs, OUTPUT);

  pinMode(Btn1_Pin, INPUT_PULLUP);
  pinMode(Btn2_Pin, INPUT_PULLUP);
  pinMode(Btn3_Pin, INPUT_PULLUP);
  pinMode(Btn4_Pin, INPUT_PULLUP);
  pinMode(Btn5_Pin, INPUT_PULLUP);
  pinMode(Btn6_Pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_IN1), [](){ encoder->tick(); }, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), [](){ encoder->tick(); }, RISING);

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);

  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

  performStartup();

  dx = dy = 0;
  delay(1000);
  initComplete = 9;
  lastTS = micros();

  MOUSE_BEGIN;
}

// ==== Button state check ====
void check_button_state() {
  if (initComplete != 9) return;

  unsigned long elapsed = curTime - lastButtonCheck;
  if (elapsed < (DEBOUNCE * 1000UL / 8)) return;

  lastButtonCheck = curTime;

  // Mouse buttons (L, R, Middle, Back, Forward)
  for (int i = 0; i < NUMBTN; i++) {
    int btn_state = digitalRead(Btn_pins[i]);
    Btn_buffers[i] = (Btn_buffers[i] << 1) | btn_state;

    if (!Btns[i] && Btn_buffers[i] == 0x00) {
      if (Btn_keys[i] != 0) {
        MOUSE_PRESS(Btn_keys[i]);
      }
      Btns[i] = true;
    } else if (Btns[i] && Btn_buffers[i] == 0xFF) {
      if (Btn_keys[i] != 0) {
        MOUSE_RELEASE(Btn_keys[i]);
      }
      Btns[i] = false;
    }
  }

  // DPI button (Btn4 on its own)
  int dpi_btn_state = digitalRead(Btn4_Pin);
  DpiBtn_buffer = (DpiBtn_buffer << 1) | dpi_btn_state;

  if (!DpiBtn && DpiBtn_buffer == 0x00) {
    DpiBtn = true;
    CpiUpdate.target_cpi_index = (CpiUpdate.target_cpi_index + 1) % NUMCPI;
    CpiUpdate.updated = false;
  } else if (DpiBtn && DpiBtn_buffer == 0xFF) {
    DpiBtn = false;
  }
}

// ==== Main loop ====
void loop() {
  static byte burstBuffer[12];
  curTime = micros();
  unsigned long elapsed = curTime - lastTS;

  // Encoder scroll
  static int pos = 0;
  encoder->tick();
  int newPos = encoder->getPosition();
  if (pos != newPos) {
    int dir = (int)encoder->getDirection();
    AdvMouse.move(0, 0, dir); // scroll only
    pos = newPos;
  }

  check_button_state();

  if (!inBurst) {
    adns_write_reg(Motion_Burst, 0x00);
    lastTS = curTime;
    inBurst = true;
  }

  if (elapsed >= 1000) {
    adns_com_begin();
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));

    SPI.transfer(Motion_Burst);
    delayMicroseconds(35);

    for (int i = 0; i < 12; i++) {
      burstBuffer[i] = SPI.transfer(0x00);
    }

    delayMicroseconds(1);
    SPI.endTransaction();
    adns_com_end();

    int motion = (burstBuffer[0] & 0x80) > 0;

    int8_t xl = (int8_t)burstBuffer[2];
    int8_t xh = (int8_t)burstBuffer[3];
    int8_t yl = (int8_t)burstBuffer[4];
    int8_t yh = (int8_t)burstBuffer[5];

    int16_t x = (int16_t)(((int16_t)xh << 8) | (uint8_t)xl);
    int16_t y = (int16_t)(((int16_t)yh << 8) | (uint8_t)yl);

    dx = x;
    dy = y;

#ifdef ADVANCE_MODE
    if (AdvMouse.needSendReport() || motion) {
      AdvMouse.move(-dx, -dy, 0); // flip both axes; adjust if directions feel wrong
      dx = 0;
      dy = 0;
    }
#else
    if (motion) {
      signed char mdx = constrain(dx, -127, 127);
      signed char mdy = constrain(dy, -127, 127);
      Mouse.move(-mdx, -mdy, 0);
      dx = 0;
      dy = 0;
    }
#endif

    lastTS = curTime;
  }

  // Apply CPI change when DPI button cycled
  if (CpiUpdate.updated == false) {
    setCPI(Cpis[CpiUpdate.target_cpi_index]);
    CpiUpdate.updated = true;
  }
}
