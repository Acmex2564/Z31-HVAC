#ifndef hvaclib_h
#define hvaclib_h

#include "Arduino.h"
#include "OneWire.h"
#include "EEPROM.h"
#include "MCP_CAN.h"

#define HIST_ARRAY_SIZE 5

class MixDoor
{
	public:
		MixDoor(int feedback, int svOpen, int svClose);
		
		void begin();
		
		void begin(uint8_t addr);
		void calibrate();
		void end(uint8_t addr);
		
		void setTarget(int position);
		int  getTarget();
		int  getOpenPos();
		int  getClosePos();
		int  getCurrentPos();
		void service();
	private:
		void doorOpen();
		void doorClose();
		void doorStop();
		unsigned long _timeSeek;
		unsigned long _timeSeekMax = 2000;
		uint8_t _addrLimits;
		uint8_t _deadband = 6;
		uint8_t _feedback;
		uint8_t _svOpen;
		uint8_t _svClose;
		int _posOpen;
		int _posClose;
		int _target;
		
};

class ModeDoor
{
	public:
		ModeDoor(int svFloorDoor, int svHeadDoor);
		ModeDoor(int svFloorDoor, int svHeadDoor, int svDefDoor);
		void begin();
		void setMode(int mode);
		int getMode();
	private:
		uint8_t _svFloorDoor;
		uint8_t _svHeadDoor;
		uint8_t _svDefDoor;
		uint8_t _currentMode;
		
		/* Mode List:
		 * 0: Defrost (all valves released)
		 * 1: Head
		 * 2: Bi-Level
		 * 3: Foot
		 * 4: Demist
		*/
};		

class IntakeDoor
{
	public:
		IntakeDoor(int svIntake1, int svIntake2, int ledRecirc);
		void begin();
		void setMode(int mode);
		void toggleIntake();
		int getMode();
	private:
		uint8_t _svIntake1;
		uint8_t _svIntake2;
		uint8_t _ledRecirc;
		uint8_t _currentPos;
		bool _manualRecirc;
		long _timeManualRecirc;
		/* Mode List:
		 * 0: Full Outside Air
		 * 1: Partial Outside Air
		 * 2: Full Recirc
		*/
};

class Blower
{
	public:
		Blower(int rFan, int fanPWM, int ledLow, int ledHigh);
		void begin();
		void setSpeedAuto(uint8_t speed);
		int getSpeed();
		void toggleSpeedLow();
		void toggleSpeedHigh();
		void setBlowerOff();
	private:
		void setSpeed(uint8_t speed);
		uint8_t _rFan;
		uint8_t _fanPWM;
		uint8_t _ledLow;
		uint8_t _ledHigh;
		uint8_t _currentSpeed;
		uint8_t _currentSpeedAuto;
		bool _isManualSpeed;
		const uint8_t _speedFanUserHigh = 255;
		const uint8_t _speedFanUserLow = 64;
};


class Temperature
{
	public:
		Temperature(int analogPin, int conversionSlope, int conversionIntercept);
		Temperature(uint8_t *rxBuffer, uint8_t targetByte);
		void begin();
		void update();
		int getTemperature();
		bool validTemperature();
	private:
		const int ADC_MAX = 1023;
		
		int _sumTemperature;
		int _arrayTemperature[HIST_ARRAY_SIZE];
		uint8_t _posPrevTemperature = 0;
		
		uint8_t _pin;
		int _slope;
		int _intercept;
		
		uint8_t *_buffer;
		uint8_t _targetByte;
};

class Climate
{
	public:
		Climate(MixDoor *doorUpper, 
				MixDoor *doorLower, 
				Blower *fan,
				ModeDoor *doorMode,
				IntakeDoor *doorIntake,
				Temperature *tempUpper,
				Temperature *tempLower,
				Temperature *ductUpper,
				Temperature *ductLower,
				Temperature *ductDefrost,
				Temperature *tempAmbient,
				Temperature *tempWater,
				int splitAdjust,
				int sunload,
				int ledEcon,
				int ledAC,
				int ledDemist,
				int ledDefrost,
				int svWater,
				int rCompressor);
		
		void begin();
		void begin(uint8_t addr);
		void service();
		void end(uint8_t addr);
		
		void toggleModeA();
		void toggleModeD();
		void setModeOff();
		void increaseTemp();
		void decreaseTemp();
		
		int getTargetTemp();
		int getTargetFanSpeed();
		
	private:
		MixDoor *_doorUpper;
		MixDoor *_doorLower;
		Blower *_fan;
		ModeDoor *_doorMode;
		IntakeDoor *_doorIntake;
		Temperature *_tempUpper;
		Temperature *_tempLower;
		Temperature *_ductUpper;
		Temperature *_ductLower;
		Temperature *_ductDefrost;
		Temperature *_tempAmbient;
		Temperature *_tempWater;
		
		void setMode(int mode);
		
		void setCompressor(bool state);
		
		void autoDoorControl();
		void autoFanControl();
		void autoTempControl();
		void manualTempControl();
			
		uint8_t _ledEcon;
		uint8_t _ledAC;
		uint8_t _ledDemist;
		uint8_t _ledDefrost;
		uint8_t _svWater;
		uint8_t _rCompressor;
		uint8_t _splitAdjust;
		bool _lastState;
		unsigned long int _timeLastCompressorToggle;
		const unsigned int _minCompressorDur = 1000 * 30; //30s min on/off time
		
		unsigned long int _timeLastTempAdjust;
		const unsigned int _intervalTempAdjust = 2000;
		
		uint8_t _currentMode;
		int _currentTargetTemp;
		int _currentTargetFanSpeed;
		
		int maxTargetTemp = 350;
		int minTargetTemp = 150;
	
};

#endif
