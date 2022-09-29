#include "Arduino.h"
#include "hvaclib.h"

MixDoor::MixDoor(int feedback, int svOpen, int svClose)
{
	_feedback = feedback;
	_svOpen = svOpen;
	_svClose = svClose;

};

void MixDoor::begin()
{
	pinMode(_feedback, INPUT);
	pinMode(_svOpen, OUTPUT);
	pinMode(_svClose, OUTPUT);

	doorClose();
	delay(2000);
	doorStop();
	_posClose = analogRead(_feedback);

	doorOpen();
	delay(5000);
	doorStop();
	_posOpen = analogRead(_feedback);
};

void MixDoor::doorOpen()
{
	digitalWrite(_svOpen, HIGH);
	digitalWrite(_svClose, LOW);
};
void MixDoor::doorClose()
{
	digitalWrite(_svOpen, HIGH);
	digitalWrite(_svClose, HIGH);
};
void MixDoor::doorStop()
{
	digitalWrite(_svOpen,LOW);
	digitalWrite(_svClose, LOW);
};
void MixDoor::seekTarget(int position)
{	int _posCurrent;
	_target = map(position, 0, 255, _posClose, _posOpen);
	
	_timeSeek = millis();
	do
	{
		_posCurrent = analogRead(_feedback);
		if (_target > _posCurrent)
		{
			doorOpen();
		}
		else if (_target < _posCurrent)
		{
			doorClose();
		}			
	}
	
	while ( ((_target >= _posCurrent + _deadband) | (_target <= _posCurrent - _deadband ) ) & ((millis() - _timeSeek) < _timeSeekMax) );
	Serial.print("Seek took "); Serial.print(millis() - _timeSeek); Serial.println("ms");
	doorStop();
};

int MixDoor::getOpenPos()
{
	int posOpen = _posOpen;
	return posOpen;
};

int MixDoor::getClosePos()
{
	int posClose = _posClose;
	return posClose;
};

ModeDoor::ModeDoor(int svFloorDoor, int svHeadDoor)
{
	_svFloorDoor = svFloorDoor;
	_svHeadDoor = svHeadDoor;
	_svDefDoor = 255;
};


ModeDoor::ModeDoor(int svFloorDoor, int svHeadDoor, int svDefDoor)
{

	_svFloorDoor = svFloorDoor;
	_svHeadDoor = svHeadDoor;
	_svDefDoor = svDefDoor;
};


void ModeDoor::begin()
{
	pinMode(_svFloorDoor, OUTPUT);
	pinMode(_svHeadDoor, OUTPUT);
	if (_svDefDoor < 255)
	{
		pinMode(_svDefDoor, OUTPUT);
	};
};

/* Mode List:
 * 0: Defrost (all valves released)
 * 1: Head
 * 2: Bi-Level
 * 3: Foot
 : 4: Demist
*/

void ModeDoor::setMode(int mode)
{
	if (_currentMode != mode) {
		switch (mode) {
			case 0:
				if (_svDefDoor < 255)
				{
					digitalWrite(_svDefDoor, LOW);
				};
				digitalWrite(_svHeadDoor, LOW);
				digitalWrite(_svFloorDoor, LOW);
				_currentMode = 0;
				break;
			case 1:
				if (_svDefDoor < 255)
				{
					digitalWrite(_svDefDoor, HIGH);
				};
				digitalWrite(_svHeadDoor, HIGH);
				digitalWrite(_svFloorDoor, LOW);
				_currentMode = 1;
				break;
			case 2:
				if (_svDefDoor < 255)
				{
					digitalWrite(_svDefDoor, HIGH);
				};
				digitalWrite(_svHeadDoor, HIGH);
				digitalWrite(_svFloorDoor, HIGH);
				_currentMode = 2;
				break;
			case 3:
				if (_svDefDoor < 255)
				{
					digitalWrite(_svDefDoor, HIGH);
				};
				digitalWrite(_svHeadDoor, LOW);
				digitalWrite(_svFloorDoor, HIGH);
				_currentMode = 3;
				break;
			case 4:
				if (_svDefDoor < 255)
				{
					digitalWrite(_svDefDoor, LOW);
				};
				digitalWrite(_svHeadDoor, LOW);
				digitalWrite(_svFloorDoor, HIGH);
				_currentMode = 4;
				break;
			
		}
	
	};
	
	
};

IntakeDoor::IntakeDoor(int svIntake1, int svIntake2, int ledRecirc)
{
	_svIntake1 = svIntake1;
	_svIntake2 = svIntake2;
	_ledRecirc = ledRecirc;
};

void IntakeDoor::begin()
{
	pinMode(_svIntake1, OUTPUT);
	pinMode(_svIntake2, OUTPUT);
	pinMode(_ledRecirc, OUTPUT);
};

void IntakeDoor::setMode(int mode)
{
	switch (mode) {
		case: 0
			digitalWrite(svIntake1, LOW);
			digitalWrite(svIntake2, LOW);
			digitalWrite(ledRecirc, LOW);
			_currentPos = 0;
		case: 1
			digitalWrite(svIntake1, HIGH);
			digitalWrite(svIntake2, LOW);
			digitalWrite(ledRecirc, HIGH);
			_currentPos = 1;
		case: 2
			digitalWrite(svIntake1, HIGH);
			digitalWrite(svIntake2, HIGH);
			digitalWrite(ledRecirc, HIGH);
			_currentPos = 2;
	}
};

void IntakeDoor::toggleIntake()
{
	if (_currentPos >= 2)
	{
		setMode(0);
	}
	else
	{
		setMode(_currentPos+1);
	}
};

Blower::Blower(int fanRelay, int fanPWM, int ledLow, int ledHigh)
{
	_fanRelay = fanRelay;
	_fanPWM = fanPWM;
	_ledLow = ledLow;
	_ledHigh = ledHigh;
};

void Blower::begin()
{
	pinMode(_fanRelay, OUTPUT);
	digitalWrite(_fanRelay, LOW);
	pinMode(_fanPWM, OUTPUT);
	pinMode(_ledLow, OUTPUT);
	pinMode(_ledHigh, OUTPUT);
};

void Blower::setSpeed(uint8_t speed)
{
	if (speed > 0) 
	{
		digitalWrite(_fanRelay, HIGH);
		analogWrite(_fanPWM, speed);
		_currentSpeed = speed;
		Serial.print("Fan Speed: "); Serial.println(_currentSpeed);
	}
	else 
	{
		digitalWrite(_fanRelay, LOW);
		analogWrite(_fanPWM, 0);
		_currentSpeed = 0;
		Serial.println("Fan off");
	}
};

void Blower::setSpeedAuto(uint8_t speed)
{
	_currentSpeedAuto = speed;
	if (!_isManualSpeed) 
	{
		setSpeed(_currentSpeedAuto);
	};
};

void Blower::setBlowerOff()
{
	setSpeed(0);
	digitalWrite(_ledLow, LOW);
	digitalWrite(_ledHigh, LOW);
	
};

void Blower::toggleSpeedLow()
{
	if (_isManualSpeed & (digitalRead(_ledLow) == HIGH)) 
	{
		_isManualSpeed = false;
		digitalWrite(_ledLow, LOW);
		digitalWrite(_ledHigh, LOW);
		setSpeed(_currentSpeedAuto);
		Serial.println("Returning to auto fan speed");
	}
	else
	{
		_isManualSpeed = true;
		digitalWrite(_ledLow, HIGH);
		digitalWrite(_ledHigh, LOW);
		setSpeed(_speedFanUserLow);
		Serial.println("Set to manual speed Low");
	}
	
};

void Blower::toggleSpeedHigh()
{
	if (_isManualSpeed & (digitalRead(_ledHigh) == HIGH)) 
	{
		_isManualSpeed = false;
		digitalWrite(_ledLow, LOW);
		digitalWrite(_ledHigh, LOW);
		setSpeed(_currentSpeedAuto);
		Serial.println("Returning to auto speed");
	}
	else
	{
		_isManualSpeed = true;
		digitalWrite(_ledLow, LOW);
		digitalWrite(_ledHigh, HIGH);
		setSpeed(_speedFanUserHigh);
		Serial.println("Set to manual speed High");
	}
};