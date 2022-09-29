#ifndef hvaclib_h
#define hvaclib_h

#include "Arduino.h"
#include "OneWire.h"

class MixDoor
{
	public:
		MixDoor(int feedback, int svOpen, int svClose);
		void begin();
		void seekTarget(int position);
		int getOpenPos();
		int getClosePos();
		bool checkTarget();
	private:
		void doorOpen();
		void doorClose();
		void doorStop();
		unsigned long _timeSeek;
		unsigned long _timeSeekMax = 2000;
		uint8_t _deadband = 3;
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
	private:
		uint8_t _svFloorDoor;
		uint8_t _svHeadDoor;
		uint8_t _svDefDoor;
		uint8_t _currentMode;
};		

class IntakeDoor
{
	public:
		IntakeDoor(int svIntake1, int svIntake2, int ledRecirc);
		void begin();
		void setMode(int mode);
		void toggleIntake();
	private:
		uint8_t _svIntake1;
		uint8_t _svIntake2;
		uint8_t _ledRecirc;
		uint8_t _currentPos;
		bool _manualRecirc;
		long _timeManualRecirc;
};

class Blower
{
	public:
		Blower(int fanRelay, int fanPWM, int ledLow, int ledHigh);
		void begin();
		void setSpeedAuto(uint8_t speed);
		void toggleSpeedLow();
		void toggleSpeedHigh();
		void setBlowerOff();
	private:
		void setSpeed(uint8_t speed);
		uint8_t _fanRelay;
		uint8_t _fanPWM;
		uint8_t _ledLow;
		uint8_t _ledHigh;
		uint8_t _currentSpeed;
		uint8_t _currentSpeedAuto;
		bool _isManualSpeed;
		const uint8_t _speedFanUserHigh = 255;
		const uint8_t _speedFanUserLow = 64;
};

#endif
