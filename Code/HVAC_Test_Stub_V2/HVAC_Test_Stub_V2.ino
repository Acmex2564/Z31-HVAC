#include <mcp_can.h>
#include <Keypad.h>
#include <OneWire.h>
#include <SPI.h>
#include "bitmaps.h"

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#include <Multi_BitBang.h>
#include <Multi_OLED.h>

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "hvaclib.h"


#define OUT_DOOR_MIX_1_O 5 //SV9
#define OUT_DOOR_MIX_1_C 6 //SV8
#define OUT_DOOR_MIX_2_O 3 //SV5
#define OUT_DOOR_MIX_2_C 4 //SV4

#define OUT_DOOR_FLOOR 12 //SV7
#define OUT_DOOR_HEAD 10 //SV6

#define OUT_EM_DEF 7

#define OUT_AUX_1 17
#define OUT_AUX_2 9
#define OUT_AUX_3 8

#define OUT_DOOR_INTAKE_1 11
#define OUT_DOOR_INTAKE_3 14

#define OUT_BLOWER_SPEED 13
#define OUT_BLOWER_RELAY 2

#define OUT_VALVE_WATER 15
#define OUT_RELAY_COMPRESSOR 16

#define LIGHT_REC 18
#define LIGHT_LOW 20
#define LIGHT_HIGH 22
#define LIGHT_DEFROST 24
#define LIGHT_ECON 26
#define LIGHT_AC 28
#define LIGHT_DEMIST 30

#define KEY_ROW_1 31
#define KEY_ROW_2 29
#define KEY_ROW_3 25
#define KEY_ROW_4 21
#define KEY_COL_1 27
#define KEY_COL_2 23

#define IN_TEMP_LOWER A0
#define IN_ILLUM_BRIGHTNESS A1
#define IN_SUN_LOAD A2
#define IN_TEMP_UPPER A3
#define IN_POSITION_2 A4
#define IN_TEMP_WATER A5
#define IN_TEMP_DUCT_LOWER A6
#define IN_POSITION_1 A7
#define IN_TEMP_AMBIENT A8
#define IN_TEMP_DUCT_UPPER A9
#define IN_TEMP_MODULE A10
#define IN_TEMP_DUCT_DEFROST A11
#define IN_BATTERY_VOLTAGE A12
#define IN_AUX_ANALOG_1 A13
#define IN_AUX_ANALOG_2 A14
#define IN_TEMP_SPLIT_ADJUST A15

#define IN_ILLUM_ENABLE 33
#define IN_SH_TIMER 35
#define IN_BULB_CHECK 37
#define IN_IGNITION 39

#define ONE_WIRE 41

#define DISPLAY_BL_1 44
#define DISPLAY_BL_2 45
#define DISPLAY_BL_3 46
#define DISPLAY_CS_1 38
#define DISPLAY_CS_2 40
#define DISPLAY_CS_3 42
#define DISPLAY_AUX_1 32
#define DISPLAY_AUX_2 34
#define DISPLAY_AUX_3 36

#define DISPLAY_DATA_COMMAND 48

#define SPI_RESET 47

#define CAN_CS 49
#define CAN_RESET 43
#define CAN_INT 19

#define NUM_DISPLAYS 2
#define NUM_BUSES 2
#define NUM_DISPLAY_STEP 5

uint8_t scl_list[NUM_BUSES] = {DISPLAY_AUX_1, DISPLAY_AUX_1};
uint8_t sda_list[NUM_BUSES] = {DISPLAY_AUX_2, DISPLAY_AUX_3};
int32_t speed_list[NUM_BUSES] = {400000L, 400000L};
uint8_t bus_list[NUM_DISPLAYS] = {0, 1};
uint8_t addr_list[NUM_DISPLAYS] = {0x3c, 0x3c};
uint8_t type_list[NUM_DISPLAYS] = {OLED_128x32, OLED_128x32};
uint8_t flip_list[NUM_DISPLAYS] = {0, 0};
uint8_t invert_list[NUM_DISPLAYS] = {0, 0};

const uint8_t addrLastTemp  = 0;
const uint8_t addrLimitsAM1 = sizeof(int);
const uint8_t addrLimitsAM2 = sizeof(int) + (sizeof(int)*2);

const int slopeNissan = -232;
const int interceptNissan = 876;

const int multiplierNissan = 16791;
const float exponentNissan = -0.0488;

const int pullTemperature = 4700;

//keypad setup
const byte rows = 4;
const byte cols = 2;

const unsigned long int idPacketTemperature = 0x750;
const uint8_t byteTemperatureWater = 0;
const uint8_t byteTemperatureAmbient = 1;
const unsigned long int idPacketFanSpeed = 0x755;
const unsigned long int idPacketVehicleSpeed = 0x760;

const unsigned long int idPacketStatusOut = 0x740;
const unsigned long int idPacketTemperatureOut = 0x745;


const unsigned long int idPacketBrightness = 0x202;



unsigned long int canRxID;
uint8_t canRxLen;
uint8_t canRxBuf[8];

uint8_t canTxLen;
uint8_t canTxBuf[8];


char keys[rows][cols] = {
                          {'H', 'L'}, //Fan HI, Fan LO
                          {'U', 'D'}, //Temp UP, Temp Down
                          {'O', 'R'}, //CC Off, Mode Recirc
                          {'A', 'M'} //Auto, DeMist
                        };

byte rowPins[rows] = {KEY_ROW_1, KEY_ROW_2, KEY_ROW_3, KEY_ROW_4};
byte colPins[cols] = {KEY_COL_1, KEY_COL_2};
Keypad buttons = Keypad(makeKeymap(keys), rowPins, colPins, rows, cols);


MCP_CAN CAN0(CAN_CS);

MixDoor doorMix1 = MixDoor(IN_POSITION_1, OUT_DOOR_MIX_1_O, OUT_DOOR_MIX_1_C, 'U');
MixDoor doorMix2 = MixDoor(IN_POSITION_2, OUT_DOOR_MIX_2_O, OUT_DOOR_MIX_2_C, 'L');
ModeDoor doorMode = ModeDoor(OUT_DOOR_FLOOR, OUT_DOOR_HEAD);
IntakeDoor doorIntake = IntakeDoor(OUT_DOOR_INTAKE_1, OUT_DOOR_INTAKE_3, LIGHT_REC);
Blower fan = Blower(OUT_BLOWER_RELAY, OUT_BLOWER_SPEED, LIGHT_LOW, LIGHT_HIGH);

Temperature tempLower = Temperature(IN_TEMP_LOWER, pullTemperature, multiplierNissan, exponentNissan);
Temperature tempUpper = Temperature(IN_TEMP_UPPER, pullTemperature, multiplierNissan, exponentNissan);
Temperature tempDuctLower = Temperature(IN_TEMP_DUCT_LOWER, pullTemperature, multiplierNissan, exponentNissan);
Temperature tempDuctUpper = Temperature(IN_TEMP_DUCT_UPPER, pullTemperature, multiplierNissan, exponentNissan);
Temperature tempDuctDefrost = Temperature(IN_TEMP_DUCT_DEFROST, pullTemperature, multiplierNissan, exponentNissan);
Temperature tempAmbient = Temperature(canRxBuf, byteTemperatureAmbient);
Temperature tempWater = Temperature(canRxBuf, byteTemperatureWater);

Climate control = Climate(&doorMix1, &doorMix2, &fan, &doorMode, &doorIntake,
                          &tempUpper, &tempLower, &tempDuctUpper, &tempDuctLower, &tempDuctDefrost, &tempAmbient, &tempWater,
                          IN_TEMP_SPLIT_ADJUST,
                          IN_SUN_LOAD,
                          LIGHT_ECON, LIGHT_AC, LIGHT_DEMIST, LIGHT_DEFROST,
                          OUT_VALVE_WATER, OUT_RELAY_COMPRESSOR);


Adafruit_ST7735 tft = Adafruit_ST7735(DISPLAY_CS_1, DISPLAY_DATA_COMMAND, SPI_RESET);

long time_current;

const int interval_print = 2 * 1000;
long time_last_print;

const int interval_max_ign = 10 * 1000;
long time_last_ign;
long time_last_can_in;
boolean broadcast;

const int interval_cycle = 4 * 1000;
long time_last_cycle;
int cycle_state = 0;

const int interval_screen = 300;
long time_last_screen;
int disp_step = 0;

const int interval_sample = 300;
long time_last_sample;

const int interval_service = 500;
long time_last_service;

const int interval_can = 500;
long time_last_can_out;

char msgString[128];

float p = 3.1415926;

void setup() {

  // configure displays
  Serial.begin(115200);
  Serial.println("Alive");

  Multi_I2CInit(sda_list, scl_list, speed_list, NUM_BUSES);
  Multi_OLEDInit(bus_list, addr_list, type_list, flip_list, invert_list, NUM_DISPLAYS);

  tft.initR(INITR_144GREENTAB);
  tft.setRotation(3);
  pinMode(DISPLAY_BL_1, OUTPUT);
  digitalWrite(DISPLAY_BL_1, LOW);
  digitalWrite(SPI_RESET, HIGH);
  tft.setRotation(3);
  tft.cp437(true);
  //tftPrintTest();
  tft.fillScreen(ST77XX_BLACK);
  tft.drawBitmap(0, 0, logoZ, 128, 128, ST77XX_ORANGE);

  char szTemp[16];
  for (uint8_t i = 0; i<NUM_DISPLAYS; i++)
  {
    Multi_OLEDFill(i,0);
    Multi_OLEDSetContrast(i, 20);
    Multi_OLEDWriteString(i, 0, 0, (char*)"Display", FONT_NORMAL, 0);
    sprintf(szTemp, "Num: %d", i);
    Multi_OLEDWriteString(i, 0, 2, szTemp, FONT_NORMAL, 0);

  }

  Serial.println("Display Init OK");


  //configure CAN controller
  pinMode(CAN_RESET, OUTPUT);
  pinMode(CAN_INT, INPUT);
  digitalWrite(CAN_RESET, HIGH);
  delay(50);
  int canstat = CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
  if (canstat == CAN_OK)
  {
    Serial.println("MCP2515 Initialized Successfully!");
  }
  else
  {
    Serial.print("Error Initializing MCP2515..."); Serial.println(canstat);
  }
  // can controller into normal mode
  CAN0.setMode(MCP_NORMAL);
  // set can transciever into normal mode
  CAN0.setGPO(0b00000011); //EN and STB HIGH


  doorMix1.begin(addrLimitsAM1);
  doorMix2.begin(addrLimitsAM2);

  pinMode(OUT_EM_DEF, OUTPUT);

  pinMode(OUT_AUX_1, OUTPUT);
  pinMode(OUT_AUX_2, OUTPUT);
  pinMode(OUT_AUX_3, OUTPUT);

  doorMode.begin();
  doorIntake.begin();

  fan.begin();

  tempLower.begin();
  tempUpper.begin();
  tempDuctLower.begin();
  tempDuctUpper.begin();
  tempDuctDefrost.begin();
  tempAmbient.begin();
  tempWater.begin();

  control.begin(addrLastTemp);

  // flash display LEDs
  digitalWrite(LIGHT_REC, HIGH);
  digitalWrite(LIGHT_LOW, HIGH);
  digitalWrite(LIGHT_HIGH, HIGH);
  digitalWrite(LIGHT_DEFROST, HIGH);
  digitalWrite(LIGHT_ECON, HIGH);
  digitalWrite(LIGHT_AC, HIGH);
  digitalWrite(LIGHT_DEMIST, HIGH);

  // one-wire bus config

  // flash display LEDs
  digitalWrite(LIGHT_REC, LOW);
  digitalWrite(LIGHT_LOW, LOW);
  digitalWrite(LIGHT_HIGH, LOW);
  digitalWrite(LIGHT_DEFROST, LOW);
  digitalWrite(LIGHT_ECON, LOW);
  digitalWrite(LIGHT_AC, LOW);
  digitalWrite(LIGHT_DEMIST, LOW);

  //doorMix1.calibrate();
  //doorMix2.calibrate();

  Serial.print("AM Door 1 Open: "); Serial.println(doorMix1.getOpenPos());
  Serial.print("AM Door 1 Close: "); Serial.println(doorMix1.getClosePos());
  Serial.print("AM Door 2 Open: "); Serial.println(doorMix2.getOpenPos());
  Serial.print("AM Door 2 Close: "); Serial.println(doorMix2.getClosePos());

  doorMix1.setTarget((int)128);
  doorMix2.setTarget((int)128);

  for (int i = 0; i <= NUM_DISPLAY_STEP-1; i++)
  {
    updateDisplay(i);
  }
  Serial.println("Setup Complete");
}

void loop() {
  // put your main code here, to run repeatedly:
  time_current = millis();
  uiServicing();

  if (digitalRead(IN_BULB_CHECK))
  {
    if (time_current - time_last_service > interval_service) {
      control.service();
    }
    doorMix1.service();
    doorMix2.service();
  };

  if (time_current - time_last_screen > interval_screen) {
    time_last_screen = time_current;
    
    updateDisplay(disp_step);
    disp_step++;
    if (disp_step > NUM_DISPLAY_STEP - 1)
    {
      disp_step = 0;
    };
  };

  if (time_current - time_last_sample > interval_sample) {
    time_last_sample = time_current;
    tempLower.update();
    tempUpper.update();
    tempDuctLower.update();
    tempDuctUpper.update();
    tempDuctDefrost.update();
  };


  //debug print handling
  if (time_current - time_last_print > interval_print) {
    time_last_print = time_current;
    print_analog_in();
    //print_discrete_in();
  }

  //power state handling
  if (digitalRead(IN_IGNITION)) {
    time_last_ign = time_current;
    //Serial.println(time_last_ign);
  }

  if (((time_current - time_last_ign > interval_max_ign) && (time_current - time_last_can_in > interval_max_ign )) || (time_current - time_last_ign > 3*interval_max_ign)) {
    poweroff();
  }

  if (millis() - time_last_ign < interval_max_ign/2) {
      broadcast = true;
    }
    else {
      broadcast = false;
    }


  if ((time_current - time_last_can_out > interval_can) && broadcast)
  {
    canTxLen = 2;
    if (digitalRead(OUT_RELAY_COMPRESSOR) == HIGH)
    {
      canTxBuf[0] = 128;
    }
    else
    {
      canTxBuf[0] = 0;
    };
    CAN0.sendMsgBuf(idPacketStatusOut, 0, canTxLen, canTxBuf);

    canTxLen = 8;
    canTxBuf[0] = (int)(control.getTargetTemp()/10);
    canTxBuf[1] = 0;
    canTxBuf[2] = (int)(tempLower.getTemperature()/10);
    canTxBuf[3] = (int)(tempUpper.getTemperature()/10);
    canTxBuf[4] = (int)(tempDuctLower.getTemperature()/10);
    canTxBuf[5] = (int)(tempDuctUpper.getTemperature()/10);
    canTxBuf[6] = (int)(tempDuctDefrost.getTemperature()/10);
    canTxBuf[7] = map(analogRead(IN_SUN_LOAD), 0, 1023, 0, 255);
    CAN0.sendMsgBuf(idPacketTemperatureOut, 0, canTxLen, canTxBuf);

    //canTxLen = 2;
    //canTxBuf[0] = 0x88;
    //canTxBuf[1] = 0xFF;
    //CAN0.sendMsgBuf(idPacketBrightness, 0, canTxLen, canTxBuf);

    time_last_can_out = time_current;
  };

  //canbus handling
  if (!digitalRead(CAN_INT))
  {
    CAN0.readMsgBuf(&canRxID, &canRxLen, canRxBuf);
    //sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", canRxID, canRxLen);
    //Serial.println(msgString);
    if (canRxID == idPacketTemperature)
    {
      //Serial.println("Recv'd temp packet");
      tempAmbient.update();
      tempWater.update();
    }


  }

}

void uiServicing() {

  char key = buttons.getKey();
  if (key) {
    //Serial.println(key);
    switch (key) {
      case 'H':
        fan.toggleSpeedHigh();
        break;
      case 'L':
        fan.toggleSpeedLow();
        break;
      case 'O':
        fan.setBlowerOff();
        control.setModeOff();
        break;
      case 'R':
        doorIntake.toggleIntake();
        break;
      case 'A':
        control.toggleModeA();
        break;
      case 'M':
        control.toggleModeD();
        break;
      case 'U':
        control.increaseTemp();
        updateDisplay(0);
        break;
      case 'D':
        control.decreaseTemp();
        updateDisplay(0);
        break;
    }
  }

}

void updateDisplay(int dispStep) {
  char szTemp[16];
  int temperature;
  uint8_t bufferVal;
  switch(dispStep)
  {
    case 0:
      
      
      temperature = control.getTargetTemp();
      sprintf(szTemp, "%d.%dC  ", temperature/10, abs(temperature%10));
      Multi_OLEDWriteString(0, 0, 0, szTemp, FONT_LARGE, 0);

      temperature = tempDuctDefrost.getTemperature();
      sprintf(szTemp, "Dd:%d.%dC", temperature/10, abs(temperature%10));
      Multi_OLEDWriteString(0, 80, 0, szTemp, FONT_SMALL, 0);
      
      temperature = tempDuctUpper.getTemperature();
      sprintf(szTemp, "Du:%d.%dC", temperature/10, abs(temperature%10));
      Multi_OLEDWriteString(0, 80, 2, szTemp, FONT_SMALL, 0);
      
      temperature = tempDuctLower.getTemperature();
      sprintf(szTemp, "Dl:%d.%dC", temperature/10, abs(temperature%10));
      Multi_OLEDWriteString(0, 80,3, szTemp, FONT_SMALL, 0);


      
      temperature = tempAmbient.getTemperature();
      sprintf(szTemp, "%d.%dC  ", temperature/10, abs(temperature%10));
      Multi_OLEDWriteString(1, 0, 0, szTemp, FONT_LARGE, 0);

      temperature = tempUpper.getTemperature();
      sprintf(szTemp, "Tu:%d.%dC", temperature/10, abs(temperature%10));
      Multi_OLEDWriteString(1, 80, 0, szTemp, FONT_SMALL, 0);

      temperature = tempLower.getTemperature();
      sprintf(szTemp, "Tl:%d.%dC", temperature/10, abs(temperature%10));
      Multi_OLEDWriteString(1, 80, 2, szTemp, FONT_SMALL, 0);
      
      break;

    case 1:
      tft.setCursor(0, 0);
      tft.setTextSize(2);
      if (digitalRead(OUT_VALVE_WATER) == LOW)
      {
        tft.setTextColor(ST77XX_BLACK, ST77XX_ORANGE);
        tft.print("HEAT: ON");
      }
      else
      {
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.print("HEAT: OFF");
      }

      tft.setCursor(0, 16);

      if (digitalRead(OUT_RELAY_COMPRESSOR) == HIGH)
      {
        tft.setTextColor(ST77XX_BLACK, ST77XX_BLUE);
        tft.print("  AC: ON");
      }
      else
      {
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.print("  AC: OFF");
      }
      break;
      
    case 2:
      tft.setCursor(0, 32);
      tft.setTextColor(ST77XX_ORANGE, ST77XX_BLACK);
      tft.print("SRC: ");
      switch(doorIntake.getMode())
      {
        case 0:
          tft.print((char)0xB0);
          break;
        case 1:
          tft.print((char)0xB1);
          break;
        case 2:
          tft.print((char)0xB2);
          break;
      }
      tft.setCursor(0, 48);
      tft.print("TGT: ");
      switch(doorMode.getMode())
      {
        case 0:
          tft.setTextColor(ST77XX_BLACK, ST77XX_ORANGE);
          tft.print((char)0x18); tft.print("  ");
          break;
        case 1:
          tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
          tft.print(" "); tft.print((char)0x1A); tft.print(" ");
          break;
        case 2:
          tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
          tft.print(" "); tft.print((char)0x1A); tft.print((char)0x19);
          break;
        case 3:
          tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
          tft.print(" " ); tft.print(" "); tft.print((char)0x19);
          break;
        case 4:
          tft.setTextColor(ST77XX_BLACK, ST77XX_ORANGE);
          tft.print((char)0x18); tft.print(" "); tft.print((char)0x19);
          break;
          
      }

      tft.setCursor(0, 112);
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      if (digitalRead(IN_ILLUM_ENABLE))
      {
        tft.print("Illum: "); tft.print(calibration_voltage(analogRead(IN_BATTERY_VOLTAGE)) - calibration_voltage(analogRead(IN_ILLUM_BRIGHTNESS)), 1); tft.print("V");
      };
      tft.setCursor(0, 120);
      tft.print("Module: "); tft.print(calibration_voltage(analogRead(IN_BATTERY_VOLTAGE)), 1); tft.print("V ");
      if (digitalRead(IN_IGNITION) && digitalRead(IN_BULB_CHECK))
      {
        tft.print("KOER");
      }
      else if (digitalRead(IN_IGNITION))
      {
        tft.print("KOEO");
      }
      else 
      {
        tft.print("OFF ");
      }
      //Serial.println("flow updt");
      break;
      
    case 3:
      
      bufferVal = constrain(fan.getSpeed(), 0, 255);

      tft.fillRoundRect(120, 2, 6, 124, 2, ST77XX_ORANGE);
      tft.drawRoundRect(120, 2, 6, 124, 2, ST77XX_WHITE);
      tft.fillRoundRect(120, 2+map(bufferVal, 0, 255, 124, 0), 6, map(bufferVal, 0, 255, 0, 124),2, ST77XX_WHITE);

      bufferVal = constrain(doorMix1.getCurrentPos(), 0, 255);
      tft.fillRoundRect(112, 2, 6, 60, 2, ST77XX_ORANGE);
      tft.drawRoundRect(112, 2, 6, 60, 2, ST77XX_WHITE);
      tft.fillRoundRect(112, 2+map(bufferVal, 0, 255, 60, 0), 6, map(bufferVal, 0, 255, 0, 60),2, ST77XX_WHITE);

      bufferVal = constrain(doorMix2.getCurrentPos(), 0, 255);
      tft.fillRoundRect(112, 66, 6, 60, 2, ST77XX_ORANGE);
      tft.drawRoundRect(112, 66, 6, 60, 2, ST77XX_WHITE);
      tft.fillRoundRect(112, 66+map(bufferVal, 0, 255, 60, 0), 6, map(bufferVal, 0, 255, 0, 60),2, ST77XX_WHITE);
      //Serial.println("bar updt");
      break;
    case 4:
      break;
  }
}

void poweroff() {
  Multi_OLEDFill(0,0);
  Multi_OLEDFill(1,0);
  tft.drawBitmap(0, 0, logoZ, 128, 128, ST77XX_ORANGE);
  Serial.println("Poweroff");
  //can transceiver to standby (INH == high impedance)

  control.end(addrLastTemp);
  doorMix1.end(addrLimitsAM1);
  doorMix2.end(addrLimitsAM2);
  delay(500);
  CAN0.setGPO(0b00000010);
  delay(1000);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_mode();
}

void canStatus(){

}

void print_analog_in() {
  Serial.println();
  Serial.println();
  Serial.println("Printing Temperature Data Block: ");
  Serial.print("Cabin (Lower): "); Serial.println(calibration_thermistor(analogRead(IN_TEMP_LOWER)));
  Serial.print("Cabin (Upper): "); Serial.println(calibration_thermistor(analogRead(IN_TEMP_UPPER)));
  Serial.print("Ambient: "); Serial.println(calibration_thermistor(analogRead(IN_TEMP_AMBIENT)));
  Serial.print("Water: "); Serial.println(calibration_thermistor(analogRead(IN_TEMP_WATER)));
  Serial.print("Duct (Lower): "); Serial.println(calibration_thermistor(analogRead(IN_TEMP_DUCT_LOWER)));
  Serial.print("Duct (Upper): "); Serial.println(calibration_thermistor(analogRead(IN_TEMP_DUCT_UPPER)));
  Serial.print("Duct (Defrost): "); Serial.println(calibration_thermistor(analogRead(IN_TEMP_DUCT_DEFROST)));
  Serial.print("Module: "); Serial.println(analogRead(IN_TEMP_MODULE));

  Serial.println();
  Serial.println("Printing Input Positions: ");
  Serial.print("Split Adjustment: "); Serial.println(analogRead(IN_TEMP_SPLIT_ADJUST));
  Serial.print("Air Mix Door 1: "); Serial.println(analogRead(IN_POSITION_1));
  Serial.print("Air Mix Door 2: "); Serial.println(analogRead(IN_POSITION_2));

  Serial.println();
  Serial.println("Printing Other Data: ");
  Serial.print("Sunload: "); Serial.println(analogRead(IN_SUN_LOAD));
  Serial.print("Module Volts: "); Serial.println(calibration_voltage(analogRead(IN_BATTERY_VOLTAGE)));
  Serial.print("Illum Volts: "); Serial.println(calibration_voltage(analogRead(IN_ILLUM_BRIGHTNESS)));

}

void print_discrete_in() {
  Serial.println();
  Serial.println();
  Serial.println("Printing Discrete Data Block: ");
  Serial.print("Lighting State: "); Serial.println(digitalRead(IN_ILLUM_ENABLE));
  Serial.print("Superheat Timer: "); Serial.println(digitalRead(IN_SH_TIMER));
  Serial.print("Engine State: "); Serial.println(digitalRead(IN_BULB_CHECK));
  Serial.print("Ignition State: "); Serial.println(digitalRead(IN_IGNITION));

}

int calibration_thermistor(int adc_counts) {
  return -232 * adc_counts * 5 / 1023 + 876;
}

float calibration_voltage(int adc_counts) {
  return adc_counts * 5 / 1023.0 * ((150.0 + 35.7) / (35.7));
}

void tftPrintTest() {
  tft.setTextWrap(false);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.println("Hello World!");
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.println("Hello World!");
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(4);
  tft.print(1234.567);
  delay(1500);
  tft.setCursor(0, 0);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(0);
  tft.println("Hello World!");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.print(p, 6);
  tft.println(" Want pi ? ");
  tft.println(" ");
  tft.print(8675309, HEX); // print 8,675,309 out in HEX!
  tft.println(" Print HEX!");
  tft.println(" ");
  tft.setTextColor(ST77XX_WHITE);
  tft.println("Sketch has been");
  tft.println("running for : ");
  tft.setTextColor(ST77XX_MAGENTA);
  tft.print(millis() / 1000);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(" seconds.");
}
