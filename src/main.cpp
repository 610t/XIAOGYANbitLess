#include <Arduino.h>

#if !defined(SPEAKER)
#define SPEAKER 1
#endif

#include <Xiaogyan.hpp>  // https://github.com/algyan/xiaogyan_arduino
static int EncoderValue_ = 127;

#if defined(ARDUINO_NRF52840_FEATHER_SENSE) || defined(ARDUINO_Seeed_XIAO_nRF52840_Sense) || defined(ARDUINO_SEEED_XIAO_NRF52840_SENSE)
#define NRF52840_SENSE
#endif

#if defined(NRF52840_SENSE)
#define BTN_PIN 7
#else
#define BTN_PIN D1
#endif
#define SPEAKER_PIN A0
#define ANALOG_INPUT_PIN A3
#define LED_ENCODER D6
#define LED_OPTIONAL D6

#include <U8x8lib.h>
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/7, /* data=*/6, /* reset=*/U8X8_PIN_NONE);  // OLEDs without Reset of the Display

#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// font data
#define FONT_SIZE 8
#if FONT_SIZE == 8
#include "font_8x8.hpp"
#define FONT_DATA font_8x8
#else  // May be 5x5
#include "font_5x5.hpp"
#define FONT_DATA font_5x5
#endif

#if defined(ARDUINO_XIAO_ESP32C3)

#define IMU_NONE 0
#define IMU_MPU6886 1
#define IMU_ADXL345 2

#if !defined(IMU_DEVICE)
#define IMU_DEVICE IMU_ADXL345  // Default IMU is ADXL345.
#endif
#if IMU_DEVICE == IMU_MPU6886
#include "I2C_MPU6886.h"  // https://github.com/tanakamasayuki/I2C_MPU6886
I2C_MPU6886 imu(I2C_MPU6886_DEFAULT_ADDRESS, Wire);
#elif IMU_DEVICE == IMU_ADXL345
static constexpr uint8_t I2C_ADDRESS = 0x53;

static constexpr uint8_t REG_POWER_CTL = 0x2d;
static constexpr uint8_t REG_DATAX0 = 0x32;
#endif
#endif

#if defined(NRF52840_SENSE)
#include "LSM6DS3.h"
LSM6DS3 imu(I2C_MODE, 0x6A);
#endif

#if defined(ARDUINO_XIAO_ESP32C3)
// For multitask for play tone
TaskHandle_t playToneTaskHandle = NULL;
#endif
bool isPlayTone = false;

// Play tone parameters
uint32_t duration;
uint8_t volume;

#if defined(ARDUINO_XIAO_ESP32C3)
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#endif

#if defined(NRF52840_SENSE)
#include <bluefruit.h>

// Dummy log
void log_i(const char *format, ...) {
  Serial.print(format);
}
#endif

#define MBIT_MORE_SERVICE "0b50f3e4-607f-4151-9091-7d008d6ffc5c"
#define MBIT_MORE_CH_COMMAND "0b500100-607f-4151-9091-7d008d6ffc5c"       // R&W(20byte)
#define MBIT_MORE_CH_STATE "0b500101-607f-4151-9091-7d008d6ffc5c"         // R(7byte)
#define MBIT_MORE_CH_MOTION "0b500102-607f-4151-9091-7d008d6ffc5c"        // R(18byte)    :pitch,roll,accel,and gyro
#define MBIT_MORE_CH_PIN_EVENT "0b500110-607f-4151-9091-7d008d6ffc5c"     // R&N
#define MBIT_MORE_CH_ACTION_EVENT "0b500111-607f-4151-9091-7d008d6ffc5c"  // R&N(20byte)  :Buttons with timestamp
#define MBIT_MORE_CH_ANALOG_IN_P0 "0b500120-607f-4151-9091-7d008d6ffc5c"  // R
#define MBIT_MORE_CH_ANALOG_IN_P1 "0b500121-607f-4151-9091-7d008d6ffc5c"  // R
#define MBIT_MORE_CH_ANALOG_IN_P2 "0b500122-607f-4151-9091-7d008d6ffc5c"  // R
#define MBIT_MORE_CH_MESSAGE "0b500130-607f-4151-9091-7d008d6ffc5c"       // R : only for v2
#define ADVERTISING_STRING "BBC micro:bit [m5scr]"

// COMMAND CH 20byte
byte cmd[] = { 0x02,  // microbit version (v1:0x01, v2:0x02)
               0x02,  // protocol 0x02 only
               0x00, 0x00, 0x00, 0x00, 0x00,
               0x00, 0x00, 0x00, 0x00, 0x00,
               0x00, 0x00, 0x00, 0x00, 0x00,
               0x00, 0x00, 0x00 };

// STATE CH 7byte
byte state[] = {
  0x00, 0x00, 0x00, 0x00,  // GPIO 0-3
  0x00,                    // lightlevel
  0x00,                    // temperature(+128)
  0x00                     // soundlevel
};

// MOTION CH 18 byte
byte motion[] = {
  0x00, 0x00,  // pitch
  0x00, 0x00,  // roll
  0xff, 0xff,  // ax
  0xff, 0x00,  // ay
  0x00, 0xff,  // az
  0x00, 0x00,  // gx
  0x00, 0x00,  // gy
  0x00, 0x00,  // gz
  0x00, 0x00   // ??
};

// ACTION CH 20 byte
byte action[] = {
  0x01,                    // BUTTON cmd; BUTTON:0x01, GESTURE: 0x02
  0x01, 0x00,              // Button Name;1:A,2:B,100:P0,101:P1,102:P2,121:LOGO
  0x00,                    // Event Name;1:DOWN, 2:UP, 3:CLICK, 4:LONG_CLICK, 5:HOLD, 6:DOUBLE_CLICK
  0x00, 0x00, 0x00, 0x00,  // Timestamp
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00,
  0x12  // ACTION Event
};

byte message[19] = { 0 };

// ANALOG PIN 2 byte
uint8_t analog[] = { 0x00, 0x00 };

#if defined(NRF52840_SENSE)
BLEService pService = BLEService(MBIT_MORE_SERVICE);
BLECharacteristic pCharacteristic[9];
#else
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic[9] = { 0 };
#endif
bool deviceConnected = false;

// for pixel pattern

#define TEXT_SPACE 10
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define BLACK 0
#define RED 1    // for XIAOGYAN 8x8 Matrix LED
#define WHITE 1  // for XIAO Expansion board
#define GREEN 2
#define ORANGE 3

uint16_t pixel[5][5] = { 0 };

void drawPixel(int x, int y, int c) {
  int w = DISPLAY_WIDTH;
  int h = DISPLAY_HEIGHT;

  int ps = (w < (h - TEXT_SPACE)) ? w / 5 : (h - TEXT_SPACE) / 5;  // Pixel size

  if (c != BLACK) {
    u8g2.drawBox(x * ps, y * ps + TEXT_SPACE, ps, ps);
    u8g2.sendBuffer();
  }
  Xiaogyan.ledMatrix.drawPixel(x, y, c);
};

void displayShowPixel() {
#if defined(ARDUINO_XIAO_ESP32C3)
  u8g2.clearDisplay();
#endif
  for (int y = 0; y < 5; y++) {
    for (int x = 0; x < 5; x++) {
      log_i("%1d", pixel[y][x] & 0b1);
      if (pixel[y][x] & 0b1) {
        drawPixel(x, y, RED);
      } else {
        drawPixel(x, y, BLACK);
      }
    }
  }
};

void fillScreen(int c) {
  Xiaogyan.ledMatrix.fillScreen(0);

  if (c == BLACK) {
    u8g2.clearDisplay();
  } else {
    u8g2.drawBox(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
    u8g2.sendBuffer();
  }
};

#if defined(NRF52840_SENSE)
void onConnect(uint16_t conn_handle) {
  BLEConnection *connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  log_i("Connected to ");
  log_i(central_name);
#else
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
#endif
  log_i("connect\n");
  deviceConnected = true;

  u8g2.clearDisplay();
  u8g2.sendBuffer();
  fillScreen(BLACK);
}
#if !defined(NRF52840_SENSE)
}
;
#endif

#if defined(NRF52840_SENSE)
void onDisconnect(uint16_t conn_handle, uint8_t reason) {
  char str[10];
  itoa(reason, str, 16);

  log_i("Disconnected, reason = 0x");
  log_i(str);
  log_i("Advertising!");
  deviceConnected = false;
#if defined(ARDUINO_XIAO_ESP32C3)
  ESP.restart();
#else
  setup();
#endif
}
#else
  void onDisconnect(BLEServer *pServer) {
    log_i("disconnect\n");
    deviceConnected = false;
#if defined(ARDUINO_XIAO_ESP32C3)
    ESP.restart();
#else
    setup();
#endif
  }
#endif

// dummy callback
#if defined(NRF52840_SENSE)
void dummyReadHandler(BLECharacteristic *chr) {
  log_i("DUMMY Read\n");
}
void dummyWriteCallback(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len) {
  log_i("DUMMY Write\n");
}
#else
  class DummyCallbacks : public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic) {
      log_i("DUMMY Read\n");
    }
    void onWrite(BLECharacteristic *pCharacteristic) {
      log_i("DUMMY Write\n");
    }
  };
#endif

void dispString(const char mes[], int duration) {
  for (int i = 0; i < strlen(mes); i++) {
    char c = mes[i];

    if (c < 32 || c > 127) {
      Serial.println("dispString Error...");
      continue;
    }

    Serial.printf("data:%x\n", FONT_DATA[c - 32]);

    for (int y = 0; y < FONT_SIZE; y++) {
      int f_data = FONT_DATA[(c - 32) * FONT_SIZE + y];
      int bit = (0b1 << FONT_SIZE - 1);
      for (int x = 0; x < FONT_SIZE; x++) {
        if (f_data & bit) {
          drawPixel(x, y, GREEN);
        } else {
          drawPixel(x, y, BLACK);
        }
        bit = bit >> 1;
      }
    }
    delay(duration);
  }
}

// for cmd
#if defined(NRF52840_SENSE)
void cmdReadHandler(BLECharacteristic *chr) {
#else
  class CmdCallbacks : public BLECharacteristicCallbacks {
    void onRead(BLECharacteristic *pCharacteristic) {
      pCharacteristic->setValue(cmd, 20);
#endif
  log_i(">>> onCmdReadHandler\n");
}

#if defined(NRF52840_SENSE)
void cmdWriteCallback(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len) {
#else
    void onWrite(BLECharacteristic *pCharacteristic) {
#endif
  log_i(">>> onCmdWriteHandler\n");
  log_i("CMD write\n");
  ////// MUST implement!!
  //// CMD_CONFIG 0x00
  // MIC    0x01
  // TOUCH  0x02
  //// CMD_PIN  0x01
  // SET_OUTPUT 0x01
  // SET_PWM    0x02
  // SET_SERVO  0x03
  // SET_PULL   0x04
  // SET_EVENT  0x05

#if defined(NRF52840_SENSE)
  String value = String((char *)data);
#else
      std::string value = pCharacteristic->getValue();
#endif
  log_i("CMD len:%d\n", value.length());
  log_i("%s\n", value.c_str());

  const char *cmd_str = value.c_str();
  log_i("%s\n", cmd_str);
  char cmd = (cmd_str[0] >> 5);
  if (cmd == 0x02) {
    //// CMD_DISPLAY  0x02
    log_i("CMD display\n");
    char cmd_display = cmd_str[0] & 0b11111;
    if (cmd_display == 0x00) {
      // CLEAR    0x00
      log_i(">> clear\n");
      fillScreen(BLACK);
    } else if (cmd_display == 0x01) {
      // TEXT     0x01
      log_i(">> text\n");
      log_i("%s\n", &(cmd_str[1]));
      u8x8.setCursor(0, 0);
      u8x8.print("                 ");  // Clear text space
      u8x8.setCursor(0, 0);
      char str[128];
      snprintf(str, sizeof(str), "%s", &(cmd_str[1]));
      // for OLED
      u8x8.print(str);
      // for 5x5 LED Matrix
      dispString(str, value[1] * 10);
      dispString(" ", 0);
    } else if (cmd_display == 0x02) {
      // PIXELS_0 0x02
      log_i(">> pixel0\n");
      for (int y = 0; y < 3; y++) {
        for (int x = 0; x < 5; x++) {
          pixel[y][x] = (cmd_str[y * 5 + (x + 1)] & 0xb);
        }
      }
    } else if (cmd_display == 0x03) {
      // PIXELS_1 0x03
      log_i(">> pixel1\n");
      for (int y = 3; y < 5; y++) {
        for (int x = 0; x < 5; x++) {
          pixel[y][x] = (cmd_str[(y - 3) * 5 + (x + 1)] & 0xb);
        }
      }
      displayShowPixel();
    }
  } else if (cmd == 0x03) {
    //// CMD_AUDIO  0x03
    log_i("CMD audio\n");
    char cmd_audio = cmd_str[0] & 0b11111;
    if (cmd_audio == 0x00) {
      // STOP_TONE  0x00
      log_i(">> Stop tone\n");
      isPlayTone = false;
    } else if (cmd_audio == 0x01) {
      // PLAY_TONE  0x01
      const uint8_t max_volume = 5;
      log_i(">> Play tone\n");
      duration = (cmd_str[4] & 0xff) << 24
                 | (cmd_str[3] & 0xff) << 16
                 | (cmd_str[2] & 0xff) << 8
                 | (cmd_str[1] & 0xff);
      volume = map(cmd_str[5], 0, 255, 0, max_volume);
      isPlayTone = true;
    }
  } else if (cmd == 0x04) {
    //// CMD_DATA (only v2) 0x04
    log_i("CMD DATA\n");

    // Show input data.
    log_i(">>> Data input:");
    for (int i = 0; i <= 20; i++) {
      log_i("(%d)%02x%c:", i, cmd_str[i], cmd_str[i]);
    }
    log_i("\n");

    // Convert from input data to label & data.
    char label[9] = { 0 };
    strncpy(label, &cmd_str[1], sizeof(label) - 1);
    String label_str = String(label);

    char data[12] = { 0 };
    strncpy(data, &cmd_str[9], sizeof(data) - 1);
    String data_str = String(data);

    // Convert from 8bit uint8_t x 4 to 32bit float with little endian.
    static union {
      uint32_t i;
      uint8_t b[sizeof(float)];
      float f;
    } conv_data;
    conv_data.b[0] = cmd_str[9];
    conv_data.b[1] = cmd_str[10];
    conv_data.b[2] = cmd_str[11];
    conv_data.b[3] = cmd_str[12];
    float data_val = conv_data.f;

    log_i("Label str:%s, Data str:%s, Data value:%f.\n", label_str, data_str, data_val);

    // Can't get correct command for number=0x13 and text=0x14. Why?
    char cmd_data = cmd_str[20];
    if (cmd_data == 0x13) {
      log_i("Data is Number.\n");
    } else if (cmd_data == 0x14) {
      log_i("Data is Text.\n");
    } else {
      log_i("Data is Unknown:%02x.\n", cmd_data);
    }

    // Sample implementation label & data event handling for M5StickC and Plus.
    // If the label "led" is data "on", the LED is turned on;
    //  otherwise, the LED is turned off.
    if (strcmp(label, "led") == 0) {
      if (strcmp(data, "on") == 0) {
        digitalWrite(LED_ENCODER, LOW);
      } else {
        digitalWrite(LED_ENCODER, HIGH);
      }
    }
  }
}
#if !defined(NRF52840_SENSE)
}
;
#endif

// for temperature
float temp = 0;

// for state
#if defined(NRF52840_SENSE)
void stateReadHandler(BLECharacteristic *chr){
#else
    class StateCallbacks : public BLECharacteristicCallbacks {
      void onRead(BLECharacteristic *pCharacteristic) {
#endif

  log_i(">> onStateReadReadHandler\n");
state[6] = (random(256) & 0xff);        // Random sensor value for soundlevel
state[5] = ((int)(temp + 128) & 0xff);  // temperature(+128)
state[4] = (EncoderValue_ & 0xff);      // lightlevel for encoder

log_i("STATE read %s", (char *)state);

#if defined(NRF52840_SENSE)
chr->write((byte *)state, 7);
}
#else
        pCharacteristic->setValue(state, 7);
      }
    };
#endif

// for accelerometer related values
#define ACC_MULT 512
#if !defined(RAD_TO_DEG)
#define RAD_TO_DEG 57.324
#endif
float ax = 0, ay = 0, az = 0;
int16_t iax, iay, iaz;
float gx, gy, gz;
float pitch, roll, yaw;

void updateIMU() {
#if defined(NRF52840_SENSE)
  // for Accelerometer nRF52840 Sense internal IMU
  ax = imu.readFloatAccelX();
  ay = imu.readFloatAccelY();
  az = imu.readFloatAccelZ();
  pitch = atan(-ax / sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
  roll = atan(ay / az) * RAD_TO_DEG;
  temp = imu.readTempC();
#else
#if IMU_DEVICE == IMU_MPU6886
      // for Accelerometer MPU6886
      imu.getAccel(&ax, &ay, &az);
      imu.getGyro(&gx, &gy, &gz);
      imu.getTemp(&temp);
#elif IMU_DEVICE == IMU_ADXL345
      // for Accelerometer ADXL345
      Wire.beginTransmission(I2C_ADDRESS);
      Wire.write(REG_DATAX0);
      if (Wire.endTransmission() == 0) {
        uint8_t readData[6];
        if (Wire.requestFrom(I2C_ADDRESS, sizeof(readData)) != sizeof(readData)) abort();
        for (size_t i = 0; i < sizeof(readData); ++i) readData[i] = Wire.read();
        int16_t val;

        ((uint8_t *)&val)[0] = readData[0];
        ((uint8_t *)&val)[1] = readData[1];
        ax = val * 2.0f / 512.0f;

        ((uint8_t *)&val)[0] = readData[2];
        ((uint8_t *)&val)[1] = readData[3];
        ay = val * 2.0f / 512.0f;

        ((uint8_t *)&val)[0] = readData[4];
        ((uint8_t *)&val)[1] = readData[5];
        az = val * 2.0f / 512.0f;
      }
#endif
#endif

  iax = (int16_t)(ax * ACC_MULT);
  iay = (int16_t)(ay * ACC_MULT);
  iaz = (int16_t)(az * ACC_MULT);
}

#if defined(NRF52840_SENSE)
void motionReadHandler(BLECharacteristic *chr) {
#else
    class MotionCallbacks : public BLECharacteristicCallbacks {
      void onRead(BLECharacteristic *pCharacteristic) {
#endif
  log_i(">>> onMotionReadHandler\n");

  updateIMU();

  motion[0] = ((int)(pitch * ACC_MULT) & 0xff);
  motion[1] = (((int)(pitch * ACC_MULT) >> 8) & 0xff);
  motion[2] = ((int)(roll * ACC_MULT) & 0xff);
  motion[3] = (((int)(roll * ACC_MULT) >> 8) & 0xff);
  motion[4] = (iax & 0xff);
  motion[5] = ((iax >> 8) & 0xff);
  motion[6] = (iay & 0xff);
  motion[7] = ((iay >> 8) & 0xff);
  motion[8] = (-iaz & 0xff);
  motion[9] = ((-iaz >> 8) & 0xff);
#if defined(NRF52840_SENSE)
  chr->write(motion, 20);
#else
        pCharacteristic->setValue(motion, 20);
#endif

  // debug print
  char msg[256] = { 0 };
  for (int i = 0; i < sizeof(motion); i++) {
    sprintf(&msg[i * 3], "%02x,", motion[i], sizeof(motion) * 3 - 3 * i);
  }
  log_i("MOTION read: %s\n", msg);
}
#if !defined(NRF52840_SENSE)
}
;
#endif

#if defined(NRF52840_SENSE)
// for button
void actionReadHandler(BLECharacteristic *chr) {
#else
      // for button
      class ActionCallbacks : public BLECharacteristicCallbacks {
        void onRead(BLECharacteristic *pCharacteristic) {
          pCharacteristic->setValue("Read me!!");  // dummy data
#endif
  log_i(">>> onActionReadHandler\n");
}
#if !defined(NRF52840_SENSE)
}
;
#endif

// for Analog pin
#if defined(NRF52840_SENSE)
void analogpinReadHandler(BLECharacteristic *chr) {
#else
        class AnalogPinCallbacks : public BLECharacteristicCallbacks {
          void onRead(BLECharacteristic *pCharacteristic) {
#endif
  log_i(">>> onAnalogPinReadHandler\n");

  int r = analogRead(ANALOG_INPUT_PIN);

  log_i("Analog Pin0 Read:%d\n", r);

  analog[0] = (r & 0xff);
  analog[1] = ((r >> 8) & 0xff);

#if defined(NRF52840_SENSE)
  chr->write(analog, 2);
#else
            pCharacteristic->setValue(analog, 2);
#endif
}
#if !defined(NRF52840_SENSE)
}
;
#endif

// Tone handler
void playToneTask(void *args) {
  while (1) {
    if (isPlayTone) {
      uint16_t freq = 1000000 / duration;

      log_i("Volume:%d\n", volume);
      log_i("Duration:%d\n", duration);
      log_i("Freq:%d\n", freq);
      // Play tone with frequency freq and duration.
      for (long i = 0; i < duration * 1000L; i += freq * 2) {
        digitalWrite(SPEAKER_PIN, HIGH);
        delayMicroseconds(freq);
        digitalWrite(SPEAKER_PIN, LOW);
        delayMicroseconds(freq);
      }
      isPlayTone = false;
    }
    delay(10);
  }
}

void setup() {
  Serial.begin(115200);

  Xiaogyan.begin();
  Xiaogyan.ledMatrix.setBrightness(2);
  Xiaogyan.ledMatrix.fillScreen(0);

  // button
  pinMode(BTN_PIN, INPUT_PULLUP);

  // for OLED Display
  //// for text
  u8x8.begin();
  u8x8.setFlipMode(1);
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  //// for 5x5 LED
  u8g2.begin();
  u8g2.setFlipMode(1);
  u8g2.clearDisplay();
  u8g2.sendBuffer();

#if defined(ARDUINO_XIAO_ESP32C3) || defined(NRF52840_SENSE)
  // Setup IMU
#if IMU_DEVICE == IMU_MP6886
  imu.begin();
  // IMU
#elif IMU_DEVICE == IMU_ADXL345
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(REG_POWER_CTL);
  Wire.write(0x08);
  if (Wire.endTransmission() != 0) abort();
#endif
#endif

  // for buzzer
  pinMode(SPEAKER_PIN, OUTPUT);

  // for encoder
  Xiaogyan.encoder.setRotatedHandler([](bool cw) {
    const int value = EncoderValue_ + (cw ? -1 : 1);
    EncoderValue_ = constrain(value, 0, 255);
    Serial.println(EncoderValue_);
  });

  // Create MAC address base fixed ID
  uint8_t mac0[6] = { 0 };

#if defined(ARDUINO_XIAO_ESP32C3)
  esp_efuse_mac_get_default(mac0);
#else
            // Create random mac address for avoid conflict ID.
            randomSeed(analogRead(A0));
            for (int i = 0; i < sizeof(mac0); i++) {
              mac0[i] = random(256);
            }
#endif

  String ID;
  char adv_str[32] = { 0 };
  for (int i = 0; i < 6; i++) {
    char ID_char = (((mac0[i] - 0x61) & 0b0011110) >> 1) + 0x61;
    ID += ID_char;
  }
  log_i("ID char:%s\n", ID.c_str());
  // Advertisement string for Microbit More.
  String("BBC micro:bit [" + ID + "]").toCharArray(adv_str, sizeof(adv_str));
  // ID string to show at splash screen.
  char id_str[32] = { 0 };
  String("ID:" + ID).toCharArray(id_str, sizeof(id_str));

  // Start up screen
  fillScreen(WHITE);
  u8x8.setCursor(0, 0);
  u8x8.print(id_str);

  log_i("BLE start.\n");
  log_i("%s", adv_str);

  Serial.print("BLE start.\n");
  Serial.println(adv_str);

#if defined(NRF52840_SENSE)
  Bluefruit.begin();
  Bluefruit.setName(adv_str);
  Bluefruit.autoConnLed(true);
  Bluefruit.setTxPower(8);

  char buf[64] = { 0 };
  uint8_t len = Bluefruit.getName(buf, 64);
  Serial.print("Get name.\n");
  Serial.println(buf);
  Serial.printf("%d\n", len);

  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  pService.begin();

  //// Characteristics
  // CMD
  pCharacteristic[0] = BLECharacteristic(MBIT_MORE_CH_COMMAND,
                                         BLERead | BLEWrite | BLEWriteWithoutResponse);
  pCharacteristic[0].setFixedLen(20);
  pCharacteristic[0].setWriteCallback(cmdWriteCallback);
  pCharacteristic[0].setPermission(SECMODE_OPEN, SECMODE_OPEN);
  pCharacteristic[0].begin();

  // STATE
  pCharacteristic[1] = BLECharacteristic(MBIT_MORE_CH_STATE,
                                         BLERead);
  pCharacteristic[1].setFixedLen(7);
  pCharacteristic[1].setPermission(SECMODE_OPEN, SECMODE_OPEN);
  pCharacteristic[1].begin();

  // MOTION
  pCharacteristic[2] = BLECharacteristic(MBIT_MORE_CH_MOTION,
                                         BLERead);
  pCharacteristic[2].setFixedLen(18);
  pCharacteristic[2].setPermission(SECMODE_OPEN, SECMODE_OPEN);
  pCharacteristic[2].begin();

  // PIN
  pCharacteristic[3] = BLECharacteristic(MBIT_MORE_CH_PIN_EVENT,
                                         BLERead | BLENotify);
  pCharacteristic[3].setFixedLen(1);
  pCharacteristic[3].setPermission(SECMODE_OPEN, SECMODE_OPEN);
  pCharacteristic[3].begin();

  // ACTION
  pCharacteristic[4] = BLECharacteristic(MBIT_MORE_CH_ACTION_EVENT,
                                         BLERead | BLENotify);
  pCharacteristic[4].setFixedLen(20);
  pCharacteristic[4].setPermission(SECMODE_OPEN, SECMODE_OPEN);
  pCharacteristic[4].begin();

  // PINS
  pCharacteristic[5] = BLECharacteristic(MBIT_MORE_CH_ANALOG_IN_P0,
                                         BLERead);
  pCharacteristic[5].setPermission(SECMODE_OPEN, SECMODE_OPEN);
  pCharacteristic[5].begin();

  pCharacteristic[6] = BLECharacteristic(MBIT_MORE_CH_ANALOG_IN_P1,
                                         BLERead);
  pCharacteristic[6].setPermission(SECMODE_OPEN, SECMODE_OPEN);
  pCharacteristic[6].begin();

  pCharacteristic[7] = BLECharacteristic(MBIT_MORE_CH_ANALOG_IN_P2,
                                         BLERead);
  pCharacteristic[7].setFixedLen(1);
  pCharacteristic[7].setPermission(SECMODE_OPEN, SECMODE_OPEN);
  pCharacteristic[7].begin();

  // MESSAGE (only for v2)
  pCharacteristic[8] = BLECharacteristic(MBIT_MORE_CH_MESSAGE,
                                         BLERead);
  pCharacteristic[8].setFixedLen(1);
  pCharacteristic[8].setPermission(SECMODE_OPEN, SECMODE_OPEN);
  pCharacteristic[8].begin();

  //// Advertise
  Bluefruit.Advertising.addService(pService);
  Bluefruit.Advertising.clearData();
  Serial.printf("%d\n", Bluefruit.Advertising.addName());
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);

  Serial.printf("count:%d\n", Bluefruit.Advertising.count());

  Bluefruit.printInfo();
#else
            BLEDevice::init(adv_str);
            BLEServer *pServer = BLEDevice::createServer();
            pServer->setCallbacks(new MyServerCallbacks());
            BLEService *pService = pServer->createService(BLEUUID(MBIT_MORE_SERVICE), 27);

            // CMD
            pCharacteristic[0] = pService->createCharacteristic(
              MBIT_MORE_CH_COMMAND,
              BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
            pCharacteristic[0]->setCallbacks(new CmdCallbacks());
            pCharacteristic[0]->addDescriptor(new BLE2902());

            // STATE
            pCharacteristic[1] = pService->createCharacteristic(
              MBIT_MORE_CH_STATE,
              BLECharacteristic::PROPERTY_READ);
            pCharacteristic[1]->setCallbacks(new StateCallbacks());
            pCharacteristic[1]->addDescriptor(new BLE2902());

            // MOTION
            pCharacteristic[2] = pService->createCharacteristic(
              MBIT_MORE_CH_MOTION,
              BLECharacteristic::PROPERTY_READ);
            pCharacteristic[2]->setCallbacks(new MotionCallbacks());
            pCharacteristic[2]->addDescriptor(new BLE2902());

            pCharacteristic[3] = pService->createCharacteristic(
              MBIT_MORE_CH_PIN_EVENT,
              BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
            pCharacteristic[3]->setCallbacks(new DummyCallbacks());
            pCharacteristic[3]->addDescriptor(new BLE2902());

            // ACTION
            pCharacteristic[4] = pService->createCharacteristic(
              MBIT_MORE_CH_ACTION_EVENT,
              BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
            pCharacteristic[4]->setCallbacks(new ActionCallbacks());
            pCharacteristic[4]->addDescriptor(new BLE2902());

            // PINS
            pCharacteristic[5] = pService->createCharacteristic(
              MBIT_MORE_CH_ANALOG_IN_P0,
              BLECharacteristic::PROPERTY_READ);
            pCharacteristic[5]->setCallbacks(new AnalogPinCallbacks());
            pCharacteristic[5]->addDescriptor(new BLE2902());

            pCharacteristic[6] = pService->createCharacteristic(
              MBIT_MORE_CH_ANALOG_IN_P1,
              BLECharacteristic::PROPERTY_READ);
            pCharacteristic[6]->setCallbacks(new AnalogPinCallbacks());
            pCharacteristic[6]->addDescriptor(new BLE2902());

            pCharacteristic[7] = pService->createCharacteristic(
              MBIT_MORE_CH_ANALOG_IN_P2,
              BLECharacteristic::PROPERTY_READ);
            pCharacteristic[7]->setCallbacks(new DummyCallbacks());
            pCharacteristic[7]->addDescriptor(new BLE2902());


            // MESSAGE (only for v2)
            pCharacteristic[8] = pService->createCharacteristic(
              MBIT_MORE_CH_MESSAGE,
              BLECharacteristic::PROPERTY_READ);
            pCharacteristic[8]->setCallbacks(new DummyCallbacks());
            pCharacteristic[8]->addDescriptor(new BLE2902());


            pService->start();
            BLEAdvertising *pAdvertising = pServer->getAdvertising();
            pAdvertising->start();
#endif

#if defined(ARDUINO_XIAO_ESP32C3)
  // Multitask for play tone.
  xTaskCreatePinnedToCore(playToneTask, "playToneTask", 4096, NULL, 1, &playToneTaskHandle, 1);
#endif

  log_i("Setup end\n");
}

void sendBtn(uint8_t btnID, uint8_t btn, uint8_t btn_status, uint8_t prev) {
  memset((char *)(action), 0, 20);  // clear action buffer

  action[0] = 0x01;   // for Button event
  action[19] = 0x12;  // ACTION_EVENT

  action[1] = btnID;  // btnID 0x01:BtnA, 0x02:BtnB, 121:BtnC(LOGO)

  // Set TimeStamp (Little Endian)
  uint32_t time = (uint32_t)millis();
  action[4] = (time & 0xff);
  action[5] = (time >> 8) & 0xff;
  action[6] = (time >> 16) & 0xff;
  action[7] = (time >> 24) & 0xff;

  if (btn) {
    // Button CLICK
    log_i(" button clicked!\n");
    action[3] = 0x03;
#if defined(NRF52840_SENSE)
    pCharacteristic[4].notify(action, 20);
#else
              pCharacteristic[4]->setValue(action, 20);
              pCharacteristic[4]->notify();
#endif
  }
  if (btn_status == 0 && prev == 1) {
    // Button Up
    log_i(" button up!\n");
    action[3] = 0x02;
#if defined(NRF52840_SENSE)
    pCharacteristic[4].notify(action, 20);
#else
              pCharacteristic[4]->setValue(action, 20);
              pCharacteristic[4]->notify();
#endif
  } else if (btn_status == 1 && prev == 0) {
    // Button Down
    log_i(" button down!\n");
    action[3] = 0x01;
#if defined(NRF52840_SENSE)
    pCharacteristic[4].notify(action, 20);
#else
              pCharacteristic[4]->setValue(action, 20);
              pCharacteristic[4]->notify();
#endif
  }
}

// Previous button state
uint8_t prevA = 0, prevB = 0, prevC = 0;
uint32_t old_label_time = 0;

void loop() {
  // Xiaogyan
  Xiaogyan.doWork();

  if (deviceConnected) {
    // Send notify data for button A, B and C(LOGO).
    uint8_t btnA = 0, btnB = 0, btnC = 0,
            btn_statusA = 0, btn_statusB = 0, btn_statusC = 0;

    // Get all button status
    btnA = Xiaogyan.buttonA.read() == LOW;
    btn_statusA = btnA;
    btnB = Xiaogyan.buttonB.read() == LOW;
    btn_statusB = btnB;

#define BUTTON_DELAY 50

    //// Button A
    action[1] = 0x01;
    sendBtn(0x01, btnA, btn_statusA, prevA);
    prevA = btn_statusA;
    delay(BUTTON_DELAY);

    //// Button B
    action[1] = 0x02;
    sendBtn(0x02, btnB, btn_statusB, prevB);
    prevB = btn_statusB;
    delay(BUTTON_DELAY);

    //// Button C (LOGO)
    action[1] = 121;  // LOGO 121
    sendBtn(121, btnC, btn_statusC, prevC);
    prevC = btn_statusC;
    delay(BUTTON_DELAY);

    // updateGesture();

    // Send dummy data label='a' data=random('a'-'z') every 50ms
    uint32_t label_time = (uint32_t)millis();
    if (label_time - old_label_time > 50) {
      memset((char *)(action), 0, 20);  // clear action buffer
      action[19] = 0x14;                // DATA_TEXT
      action[0] = 0x61;                 // 'a'
      action[1] = 0;
      action[8] = 0x61 + random(26);  // 'a-z'
      action[9] = 0;
      // Set TimeStamp (Little Endian)
      uint32_t time = (uint32_t)millis();
      action[4] = (time & 0xff);
      action[5] = (time >> 8) & 0xff;
      action[6] = (time >> 16) & 0xff;
      action[7] = (time >> 24) & 0xff;
#if defined(NRF52840_SENSE)
      pCharacteristic[4].notify(action, 20);
#else
                pCharacteristic[4]->setValue(action, 20);
                pCharacteristic[4]->notify();
#endif
      old_label_time = label_time;
    }

#if defined(NRF52840_SENSE)
    //// Read event handlers
    cmdReadHandler(&pCharacteristic[0]);
    stateReadHandler(&pCharacteristic[1]);
    motionReadHandler(&pCharacteristic[2]);
    actionReadHandler(&pCharacteristic[4]);
    //analogpinReadHandler(&pCharacteristic[5]);
    //analogpinReadHandler(&pCharacteristic[6]);
#endif
  }
}
