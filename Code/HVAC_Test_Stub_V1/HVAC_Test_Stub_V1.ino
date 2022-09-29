#include <mcp_can.h>
#include <Keypad.h>
#include <OneWire.h>
#include <SPI.h>
#include <GxEPD2_BW.h>
#include "bitmaps.h"
#include <avr/interrupt.h>
#include <avr/sleep.h>'
#include "hvaclib.h"


#define GxEPD2_DISPLAY_CLASS GxEPD2_BW
#define GxEPD2_DRIVER_CLASS GxEPD2_290_T94_V2 // GDEM029T94  128x296, SSD1680, Waveshare 2.9" V2 variant

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
#define OUT_COMPRESSOR_RELAY 16

#define LIGHT_REC 18
#define LIGHT_LOW 20
#define LIGHT_HIGH 22
#define LIGHT_DEF 24
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

#define ENABLE_GxEPD2_GFX 0
#define MAX_DISPLAY_BUFFER_SIZE 5000 // e.g. full height for 128x296
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
GxEPD2_DISPLAY_CLASS<GxEPD2_DRIVER_CLASS, MAX_HEIGHT(GxEPD2_DRIVER_CLASS)> display(GxEPD2_DRIVER_CLASS(DISPLAY_BL_3 /*CS*/, DISPLAY_DATA_COMMAND /*DC*/, SPI_RESET /*RST*/, DISPLAY_BL_2 /*BUSY*/));


//keypad setup
const byte rows = 4;
const byte cols = 2;

char keys[rows][cols] = {
                          {'H', 'L'}, //Fan HI, Fan LO
                          {'U', 'D'}, //Temp UP, Temp Down
                          {'O', 'R'}, //CC Off, Mode Recirc
                          {'A', 'M'} //Auto, DeMist
                        };

byte rowPins[rows] = {KEY_ROW_1, KEY_ROW_2, KEY_ROW_3, KEY_ROW_4};
byte colPins[cols] = {KEY_COL_1, KEY_COL_2};
Keypad buttons = Keypad(makeKeymap(keys), rowPins, colPins, rows, cols);

//struct sTemp
//{ bool isOneWire
//  char pinAnalog;
//  DeviceAddress tempAddress;
//  float tempArray[HIST_ARRAY_SIZE];
//  float tempSum;
//  float tempAverage;
//  int tempPrevPos;
//};

MCP_CAN CAN0(CAN_CS);

MixDoor doorMix1 = MixDoor(IN_POSITION_1, OUT_DOOR_MIX_1_O, OUT_DOOR_MIX_1_C);
MixDoor doorMix2 = MixDoor(IN_POSITION_2, OUT_DOOR_MIX_2_O, OUT_DOOR_MIX_2_C);
ModeDoor doorMode = ModeDoor(OUT_DOOR_FLOOR, OUT_DOOR_HEAD);
IntakeDoor doorIntake = IntakeDoor(OUT_DOOR_INTAKE_1, OUT_DOOR_INTAKE_3, LIGHT_REC);
Blower fan = Blower(OUT_BLOWER_RELAY, OUT_BLOWER_SPEED, LIGHT_LOW, LIGHT_HIGH);

long time_current;

const int interval_print = 2 * 1000;
long time_last_print;

const int interval_max_ign = 10 * 1000;
long time_last_ign;
long time_last_can;

const int interval_cycle = 4 * 1000;
long time_last_cycle;
int cycle_state = 0;

const int interval_screen = 300;
long time_last_screen;



void setup() {

  // configure displays
  display.init(115200);
  display.setRotation(1);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);

  }
  while (display.nextPage());
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


  doorMix1.begin();
  doorMix2.begin();

  pinMode(OUT_EM_DEF, OUTPUT);

  pinMode(OUT_AUX_1, OUTPUT);
  pinMode(OUT_AUX_2, OUTPUT);
  pinMode(OUT_AUX_3, OUTPUT);

  doorMode.begin();
  doorIntake.begin();

  fan.begin();

  pinMode(OUT_VALVE_WATER, OUTPUT);
  pinMode(OUT_COMPRESSOR_RELAY, OUTPUT);

  //pinMode(LIGHT_REC, OUTPUT);
  //pinMode(LIGHT_LOW, OUTPUT);
  //pinMode(LIGHT_HIGH, OUTPUT);
  pinMode(LIGHT_DEF, OUTPUT);
  pinMode(LIGHT_ECON, OUTPUT);
  pinMode(LIGHT_AC, OUTPUT);
  pinMode(LIGHT_DEMIST, OUTPUT);

  // flash display LEDs
  digitalWrite(LIGHT_REC, HIGH);
  digitalWrite(LIGHT_LOW, HIGH);
  digitalWrite(LIGHT_HIGH, HIGH);
  digitalWrite(LIGHT_DEF, HIGH);
  digitalWrite(LIGHT_ECON, HIGH);
  digitalWrite(LIGHT_AC, HIGH);
  digitalWrite(LIGHT_DEMIST, HIGH);
  
  // one-wire bus config

  // flash display LEDs
  digitalWrite(LIGHT_REC, LOW);
  digitalWrite(LIGHT_LOW, LOW);
  digitalWrite(LIGHT_HIGH, LOW);
  digitalWrite(LIGHT_DEF, LOW);
  digitalWrite(LIGHT_ECON, LOW);
  digitalWrite(LIGHT_AC, LOW);
  digitalWrite(LIGHT_DEMIST, LOW);
  Serial.print("AM Door 1 Open: "); Serial.println(doorMix1.getOpenPos());
  Serial.print("AM Door 1 Close: "); Serial.println(doorMix1.getClosePos());
  Serial.print("AM Door 2 Open: "); Serial.println(doorMix2.getOpenPos());
  Serial.print("AM Door 2 Close: "); Serial.println(doorMix2.getClosePos());

  doorMix1.seekTarget((int)128);
  doorMix2.seekTarget((int)128);

  display.epd2.setBusyCallback(uiServicing);
}

void loop() {
  // put your main code here, to run repeatedly:
  time_current = millis();

  uiServicing();

  //debug print handling
  if (time_current - time_last_print > interval_print) {
    time_last_print = time_current;
    //print_analog_in();
    //print_discrete_in();
  }

  //power state handling
  if (digitalRead(IN_IGNITION)) {
    time_last_ign = time_current;
  }

  if ((time_current - time_last_ign > interval_max_ign) && (time_current - time_last_can > interval_max_ign )) {
    //poweroff();
  }

  if (time_current - time_last_cycle > interval_cycle) {
    time_last_cycle = time_current;
    doorMode.setMode(cycle_state);

    //doorMix1.seekTarget((int)255);


    cycle_state = cycle_state + 1;
    if (cycle_state > 4) {
      cycle_state = 0;
    };
  }

  if (time_current - time_last_screen > interval_screen) {

    
  };
  
  
  //canbus handling
}

void uiServicing() {
  
  char key = buttons.getKey();
  if (key) {
    Serial.println(key);

    switch (key) {
      case 'H':
        fan.toggleSpeedHigh();
        break;
      case 'L':
        fan.toggleSpeedLow();
        break;
      case 'O':
        fan.setBlowerOff();
        break;
    }
  }
  
}

void poweroff() {

  //show Z logo after poweroff
  do {
    display.drawInvertedBitmap(0, 0, epd_bitmap_logoZ, 296, 128, GxEPD_BLACK);
  }
  while (display.nextPage());
  display.powerOff();
  //can transceiver to standby (INH == high impedance)
  CAN0.setGPO(0b00000010);

  delay(1000);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_mode();
}


void print_analog_in() {
  Serial.println();
  Serial.println();
  Serial.println("Printing Temperature Data Block:");
  Serial.print("Cabin (Lower): "); Serial.println(calibration_thermistor(analogRead(IN_TEMP_LOWER)));
  Serial.print("Cabin (Upper): "); Serial.println(calibration_thermistor(analogRead(IN_TEMP_UPPER)));
  Serial.print("Ambient: "); Serial.println(calibration_thermistor(analogRead(IN_TEMP_AMBIENT)));
  Serial.print("Water: "); Serial.println(calibration_thermistor(analogRead(IN_TEMP_WATER)));
  Serial.print("Duct (Lower): "); Serial.println(calibration_thermistor(analogRead(IN_TEMP_DUCT_LOWER)));
  Serial.print("Duct (Upper): "); Serial.println(calibration_thermistor(analogRead(IN_TEMP_DUCT_UPPER)));
  Serial.print("Duct (Defrost): "); Serial.println(calibration_thermistor(analogRead(IN_TEMP_DUCT_DEFROST)));
  Serial.print("Module: "); Serial.println(analogRead(IN_TEMP_MODULE));

  Serial.println();
  Serial.println("Printing Input Positions:");
  Serial.print("Split Adjustment: "); Serial.println(analogRead(IN_TEMP_SPLIT_ADJUST));
  Serial.print("Air Mix Door 1: "); Serial.println(analogRead(IN_POSITION_1));
  Serial.print("Air Mix Door 2: "); Serial.println(analogRead(IN_POSITION_2));

  Serial.println();
  Serial.println("Printing Other Data:");
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

float calibration_thermistor(int adc_counts) {
  return -23.2 * adc_counts * 5 / 1023 + 87.6;
}

float calibration_voltage(int adc_counts) {
  return adc_counts * 5 / 1023 * (35.7 / (180.0 + 35.7));
}

void seek_target_position(byte pot, int target, byte actLow, byte actHigh) {

}
