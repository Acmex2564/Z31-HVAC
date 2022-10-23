#include "Arduino.h"
#include "EEPROM.h"
#include "hvaclib.h"
#include "MCP_CAN.h"

MixDoor::MixDoor(int feedback, int svOpen, int svClose, char doorId)
{
	_feedback = feedback;
	_svOpen = svOpen;
	_svClose = svClose;
	_id = doorId;

};

void MixDoor::begin()
{
	pinMode(_feedback, INPUT);
	pinMode(_svOpen, OUTPUT);
	pinMode(_svClose, OUTPUT);

	calibrate();
};

void MixDoor::begin(uint8_t addr)
{
	pinMode(_feedback, INPUT);
	pinMode(_svOpen, OUTPUT);
	pinMode(_svClose, OUTPUT);
	
	int x;
	
	_posClose = EEPROM.get(addr, x);
	_posOpen  = EEPROM.get(addr+sizeof(int), x);
	
};

void MixDoor::calibrate()
{
	doorClose();
	delay(2000);
	doorStop();
	_posClose = analogRead(_feedback);

	doorOpen();
	delay(5000);
	doorStop();
	_posOpen = analogRead(_feedback);
};

void MixDoor::end(uint8_t addr)
{
	EEPROM.put(addr, _posClose);
	EEPROM.put(addr+sizeof(int), _posOpen);
};

void MixDoor::doorOpen()
{
	digitalWrite(_svOpen, HIGH);
	digitalWrite(_svClose, LOW);
	Serial.print("MD: "); Serial.print(_id); Serial.println(" Op");
};
void MixDoor::doorClose()
{
	digitalWrite(_svOpen, HIGH);
	digitalWrite(_svClose, HIGH);
	Serial.print("MD: "); Serial.print(_id); Serial.println(" Cl");
};
void MixDoor::doorStop()
{
	digitalWrite(_svOpen,LOW);
	digitalWrite(_svClose, LOW);
	Serial.print("MD: "); Serial.print(_id); Serial.println(" St");
};
void MixDoor::setTarget(int position)
{	
	_target = map(position, 0, 255, _posClose, _posOpen);
	_timeSeek = millis();
	
}
int MixDoor::getTarget()
{
	return map(_target, _posClose, _posOpen, 0, 255);
}	
void MixDoor::service()
{
	int _posCurrent;
	if (((_target >= _posCurrent + _deadband) | (_target <= _posCurrent - _deadband ) ))
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
	else
	{
		doorStop();
	};
	//Serial.print("Seek took "); Serial.print(millis() - _timeSeek); Serial.println("ms");
	
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

int MixDoor::getCurrentPos()
{
	return map(analogRead(_feedback), _posClose, _posOpen, 0, 255);
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
 * 4: Demist
*/

int ModeDoor::getMode()
{
	return _currentMode;
}


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
				Serial.println("ModeDoor: 0");
				break;
			case 1:
				if (_svDefDoor < 255)
				{
					digitalWrite(_svDefDoor, HIGH);
				};
				digitalWrite(_svHeadDoor, HIGH);
				digitalWrite(_svFloorDoor, LOW);
				_currentMode = 1;
				Serial.println("ModeDoor: 1");
				break;
			case 2:
				if (_svDefDoor < 255)
				{
					digitalWrite(_svDefDoor, HIGH);
				};
				digitalWrite(_svHeadDoor, HIGH);
				digitalWrite(_svFloorDoor, HIGH);
				_currentMode = 2;
				Serial.println("ModeDoor: 2");
				break;
			case 3:
				if (_svDefDoor < 255)
				{
					digitalWrite(_svDefDoor, HIGH);
				};
				digitalWrite(_svHeadDoor, LOW);
				digitalWrite(_svFloorDoor, HIGH);
				_currentMode = 3;
				Serial.println("ModeDoor: 3");
				break;
			case 4:
				if (_svDefDoor < 255)
				{
					digitalWrite(_svDefDoor, LOW);
				};
				digitalWrite(_svHeadDoor, LOW);
				digitalWrite(_svFloorDoor, HIGH);
				_currentMode = 4;
				Serial.println("ModeDoor: 4");
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
		case 0:
			digitalWrite(_svIntake1, LOW);
			digitalWrite(_svIntake2, LOW);
			digitalWrite(_ledRecirc, LOW);
			_currentPos = 0;
			Serial.println("IntakeDoor: 0");
			break;
		case 1:
			digitalWrite(_svIntake1, HIGH);
			digitalWrite(_svIntake2, LOW);
			digitalWrite(_ledRecirc, HIGH);
			_currentPos = 1;
			Serial.println("IntakeDoor: 1");
			break;
		case 2:
			digitalWrite(_svIntake1, HIGH);
			digitalWrite(_svIntake2, HIGH);
			digitalWrite(_ledRecirc, HIGH);
			_currentPos = 2;
			Serial.println("IntakeDoor: 2");
			break;
	}
};

int IntakeDoor::getMode()
{
	return _currentPos;
}

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

Blower::Blower(int rFan, int fanPWM, int ledLow, int ledHigh)
{
	_rFan = rFan;
	_fanPWM = fanPWM;
	_ledLow = ledLow;
	_ledHigh = ledHigh;
};

void Blower::begin()
{
	pinMode(_rFan, OUTPUT);
	digitalWrite(_rFan, LOW);
	pinMode(_fanPWM, OUTPUT);
	pinMode(_ledLow, OUTPUT);
	pinMode(_ledHigh, OUTPUT);
};

void Blower::setSpeed(uint8_t speed)
{
	if (speed > 0) 
	{
		digitalWrite(_rFan, HIGH);
		analogWrite(_fanPWM, speed);
		_currentSpeed = speed;
		Serial.print("Fan Speed: "); Serial.println(_currentSpeed);
	}
	else 
	{
		digitalWrite(_rFan, LOW);
		analogWrite(_fanPWM, 0);
		_currentSpeed = 0;
		Serial.println("Fan off");
	}
};

int Blower::getSpeed()
{
	return _currentSpeed;
}

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


Temperature::Temperature(int analogPin, int conversionSlope, int conversionIntercept)
{
	_pin = analogPin;
	_slope = conversionSlope;
	_intercept = conversionIntercept;
	
};
Temperature::Temperature(uint8_t *rxBuffer, uint8_t targetByte)
{
	_pin = 255;
	_buffer = rxBuffer;
	_targetByte = targetByte;
};

void Temperature::begin()
{
	if (_pin < 255)
	{
		pinMode(_pin, INPUT);
		
		for (int i = 0; i <  HIST_ARRAY_SIZE - 1; i++)
		{
			_arrayTemperature[i] = (_slope * 5) + _intercept;
			_sumTemperature = ((_slope * 5) + _intercept) + _sumTemperature;
		}
	};
};

void Temperature::update()
{
	if (_posPrevTemperature == HIST_ARRAY_SIZE - 1)
	{
		_posPrevTemperature = 0;
	}
	else
	{
		++_posPrevTemperature;
	};
	
	_sumTemperature = _sumTemperature - _arrayTemperature[_posPrevTemperature];
	
	if (_pin < 255)
	{
		_arrayTemperature[_posPrevTemperature] = ((_slope * 5L * analogRead(_pin)) / ADC_MAX) + _intercept;
	}
	else
	{
		_arrayTemperature[_posPrevTemperature] = (_buffer[_targetByte] - 40)*10;
		Serial.println(_buffer[_targetByte]);
	};
	
	_sumTemperature = _sumTemperature + _arrayTemperature[_posPrevTemperature];
	
};

int Temperature::getTemperature()
{
	return _sumTemperature / HIST_ARRAY_SIZE;
};

bool Temperature::validTemperature()
{
	if ((_sumTemperature / HIST_ARRAY_SIZE) >= _intercept)
	{	
		return false;
	}
	else if ((_sumTemperature / HIST_ARRAY_SIZE) <=  (_slope * 5) + _intercept)
	{
		return false;
	}
	else
	{
		return true;
	};
	
};

Climate::Climate(MixDoor *doorUpper, 
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
				 int rCompressor)
{
	_doorUpper = doorUpper;
	_doorLower = doorLower;
	_fan = fan;
	_doorMode = doorMode;
	_doorIntake = doorIntake;
	
	_tempUpper = tempUpper;
	_tempLower = tempLower;
	_ductUpper = ductUpper;
	_ductLower = ductLower;
	_ductDefrost = ductDefrost;
	_tempAmbient = tempAmbient;
	_tempWater = tempWater;
	
	_ledEcon = ledEcon;
	_ledAC = ledAC;
	_ledDemist = ledDemist;
	_ledDefrost = ledDefrost;
	_svWater = svWater;
	_rCompressor = rCompressor;
	_splitAdjust = splitAdjust;
};

void Climate::begin()
{
	pinMode(_ledEcon, OUTPUT);
	pinMode(_ledAC, OUTPUT);
	pinMode(_ledDemist, OUTPUT);
	pinMode(_ledDefrost, OUTPUT);
	pinMode(_svWater, OUTPUT);
	pinMode(_rCompressor, OUTPUT);
	
	digitalWrite(_ledEcon, LOW);
	digitalWrite(_ledAC, LOW);
	digitalWrite(_ledDemist, LOW);
	digitalWrite(_ledDefrost, LOW);
	digitalWrite(_svWater, LOW);
	digitalWrite(_rCompressor, LOW);
	
	setMode(0);
	_currentTargetTemp = 200;
};

void Climate::begin(uint8_t addr)
{
	pinMode(_ledEcon, OUTPUT);
	pinMode(_ledAC, OUTPUT);
	pinMode(_ledDemist, OUTPUT);
	pinMode(_ledDefrost, OUTPUT);
	pinMode(_svWater, OUTPUT);
	pinMode(_rCompressor, OUTPUT);
	
	digitalWrite(_ledEcon, LOW);
	digitalWrite(_ledAC, LOW);
	digitalWrite(_ledDemist, LOW);
	digitalWrite(_ledDefrost, LOW);
	digitalWrite(_svWater, LOW);
	digitalWrite(_rCompressor, LOW);
	
	setMode(0);
	_currentTargetTemp = EEPROM.get(addr, _currentTargetTemp);
	if ((_currentTargetTemp < minTargetTemp) || (_currentTargetTemp > maxTargetTemp))
	{
		_currentTargetTemp = 200;
	};
}

void Climate::increaseTemp()
{
	_currentTargetTemp = _currentTargetTemp + 5;
	
	if (_currentTargetTemp > maxTargetTemp) {_currentTargetTemp = maxTargetTemp;}
	else if (_currentTargetTemp < minTargetTemp) {_currentTargetTemp = minTargetTemp;}
	
};

void Climate::decreaseTemp()
{
	_currentTargetTemp = _currentTargetTemp - 5;
	
	if (_currentTargetTemp > maxTargetTemp) {_currentTargetTemp = maxTargetTemp;}
	else if (_currentTargetTemp < minTargetTemp) {_currentTargetTemp = minTargetTemp;}
	
};

int Climate::getTargetTemp()
{
	return _currentTargetTemp;
}

void Climate::setMode(int mode)
{
	switch (mode)
	{
		case 0: //emergency defrost
			digitalWrite(_ledEcon, LOW);
			digitalWrite(_ledAC, LOW);
			digitalWrite(_ledDemist, LOW);
			digitalWrite(_ledDefrost, LOW);
			_currentTargetFanSpeed = 255;
			_doorIntake->setMode(0);
			_doorMode->setMode(0);
			_currentMode = 0;
			Serial.println("ClimateMode: 0");
			break;
			
		case 1: //normal functioning, without AC
			digitalWrite(_ledEcon, HIGH);
			digitalWrite(_ledAC, LOW);
			digitalWrite(_ledDemist, LOW);
			digitalWrite(_ledDefrost, LOW);
			_currentMode = 1;
			Serial.println("ClimateMode: 1");
			break;
			
		case 2: //normal functioning, with AC
			digitalWrite(_ledEcon, LOW);
			digitalWrite(_ledAC, HIGH);
			digitalWrite(_ledDemist, LOW);
			digitalWrite(_ledDefrost, LOW);
			_currentMode = 2;
			Serial.println("ClimateMode: 2");
			break;
			
		case 3: //defog/demist operation (warm air, with compressor)
			digitalWrite(_ledEcon, LOW);
			digitalWrite(_ledAC, LOW);
			digitalWrite(_ledDemist, HIGH);
			digitalWrite(_ledDefrost, LOW);
			_currentMode = 3;
			Serial.println("ClimateMode: 3");
			break;
			
		case 4: //defrost functioning, full hot, no AC
			digitalWrite(_ledEcon, LOW);
			digitalWrite(_ledAC, LOW);
			digitalWrite(_ledDemist, LOW);
			digitalWrite(_ledDefrost, HIGH);
			_currentMode = 4;
			Serial.println("ClimateMode: 4");
			break;
			
		
	}
}

void Climate::toggleModeA()
{
	if (_currentMode == 1)
	{
		setMode(2);
	}
	else
	{
		setMode(1);
	}
}

void Climate::toggleModeD()
{
	if (_currentMode == 3)
	{
		setMode(4);
	}
	else 
	{
		setMode(3);
	}
}

void Climate::setModeOff()
{
	setMode(0);
}

void Climate::service()
{
	switch (_currentMode)
	{
		case 0: //"off" (auto, without fan or AC)
			autoDoorControl();
			_fan->setSpeedAuto(0);
			setCompressor(false);
			if (_tempAmbient < _currentTargetTemp + 30)
			{
				digitalWrite(_svWater, LOW);
			}
			//disable if target+4 < ambient
			else if (_tempAmbient > _currentTargetTemp + 40)
			{
				digitalWrite(_svWater, HIGH);
			};
			//autoTempControl();
			manualTempControl();
			
			break;
		case 1: //auto/econ
			
			autoDoorControl();
			autoFanControl();
			setCompressor(false);
			
			//water cock control
			if (_tempAmbient < _currentTargetTemp + 30)
			{
				digitalWrite(_svWater, LOW);
			}
			//disable if target+4 < ambient
			else if (_tempAmbient > _currentTargetTemp + 40)
			{
				digitalWrite(_svWater, HIGH);
			};
			//autoTempControl();
			manualTempControl();
			break;
		case 2: //auto/full
			
			autoDoorControl();
			autoFanControl();
			//compressor control
			if ((_tempAmbient > _currentTargetTemp - 30) && (_tempAmbient > 10))
			{
				setCompressor(true);
			}
			else
			{
				setCompressor(false);
			};
			
			//water cock control
			//open if target is more than 3 degrees above ambient
			if (_tempAmbient < _currentTargetTemp + 30)
			{
				digitalWrite(_svWater, LOW);
			}
			//close if setpoint is more than 4 degrees above target
			else if (_tempAmbient + 40 < _currentTargetTemp)
			{
				digitalWrite(_svWater, HIGH);
			};
			//autoTempControl();
			manualTempControl();
			
			break;
		case 3: //defog/demist
			_doorMode->setMode(4);
			autoFanControl();
			
			if ((_tempAmbient > 10))
			{
				setCompressor(true);
			}
			else 
			{
				setCompressor(false);
			};
			//autoTempControl();
			manualTempControl();
			
			break;
		case 4: //defrost
			_doorMode->setMode(0); //windshield air target
			_doorIntake->setMode(0); //outside air source
			_doorUpper->setTarget(255); //full hot upper door
			_doorLower->setTarget(255); //full hot lower door
			_fan->setSpeedAuto(255); //full fan speed
			digitalWrite(_svWater, LOW); //open water cock
			
			if ((_tempAmbient > _currentTargetTemp - 30) && (_tempAmbient > 10))
			{
				setCompressor(true);
			}
			else
			{
				setCompressor(false);
			}
			
			
			break;
	};
	
	
};

void Climate::autoDoorControl()
{
	int currentMode = _doorMode->getMode();
	int currentTemp = _tempAmbient->getTemperature();
	if (currentMode == 0 || currentMode == 4)
	{
		_doorMode->setMode(2);
	}
	
	//per 1988 FSM HA-79
	//TODO: SUNLOAD COMPENSATION CALC
	if (currentMode == 3 && currentTemp > -90)
	{
		_doorMode->setMode(2);
	}
	else if (currentMode == 2 && currentTemp > 150)
	{
		_doorMode->setMode(1);
	}
	else if (currentMode == 1 && currentTemp < 110)
	{
		_doorMode->setMode(2);
	}
	else if (currentMode == 2 && currentTemp < -110)
	{
		_doorMode->setMode(3);
	}

}

void Climate::autoFanControl()
{
	
	uint8_t _vMin;
	uint8_t _vMax;
	uint8_t _outFan;
	uint8_t _diff;
	
	//calculate V_M(MAX), which is the same for all modes
	//based on ambient temperature - objective temperature
	_vMax = constrain(map(abs(_tempAmbient->getTemperature() - _currentTargetTemp), 0, 90, 200, 255), 200, 255);
	
	//calculate V_M(MIN),  which varies depending on the mode, air target, and sunload
	switch (_doorMode->getMode())
	{
		case 0:
			break;
		case 1:
			_vMin = 64;
			_diff = abs(_ductUpper->getTemperature() - _tempLower->getTemperature()); // TODO: INSTALL UPPER TEMPERATURE SENSOR
			break;
		case 2:
			_vMin = 100;
			_diff = abs(((_ductLower->getTemperature() - _tempLower->getTemperature()) + (_ductUpper->getTemperature() - _tempLower->getTemperature()))/2);
			break;
		case 3:
			_vMin = 100;
			_diff = abs(((_ductLower->getTemperature() - _tempLower->getTemperature()) + (_ductUpper->getTemperature() - _tempLower->getTemperature()))/2);
			break;
		case 4:
			_vMin = 128;
			_diff = abs(_ductDefrost->getTemperature() - _tempLower->getTemperature());
			break;
	};
	
	
	
	
	//resulting fan speed is a linear interpolation between VM_MIN and VM_MAX
	//based on objective temperature - current temperature for HEAD distribution
	
	//based on average temperature - objective temperature for BI-LEVEL / FLOOR distribution
	
	
	_outFan = constrain(map(_diff, _vMin,_vMax, 10, 80 ), _vMin, _vMax);
	_fan->setSpeedAuto(_outFan);
	
	
};

//this is the desired, fully automatic, temperature control (PID-Based)
//it doesn't work correctly at the moment
void Climate::autoTempControl()
{
	if (millis() - _timeLastTempAdjust >= _intervalTempAdjust)
	{
		_timeLastTempAdjust = millis();
		//calculate objective (duct) temperatures, upper/defrost and lower
		int8_t _ductObjUpper = _currentTargetTemp;
		
		//foot objective temperature has a 0-5 degree celsius modifier based on ambient temperature
		//no modifier at 20c, +5 at -5c
		int8_t _ductObjLower = _currentTargetTemp + constrain(map(_tempAmbient->getTemperature(), 200, -50, 0, 50  ), 0, 50);
		
		//modify based on temp split adjustment
		int8_t _split = constrain(map(analogRead(_splitAdjust), 0, 1023, -20, 20), -20, 20);
		_ductObjUpper = _ductObjUpper + _split;
		_ductObjLower = _ductObjLower - _split;
		
		
		//simple control proportional to observed error between target duct temperature (based on mode) and observed duct temperature
		/* Current: 200
		Objective: 180

		Need door to close (lower position)

		Objective - Current = adjustment
		180 - 200 = -20

		currenpos + adjustment = newpos
		180 + (-20) = 160; */
		
		int8_t diffUpper;
		int8_t diffLower;
		
		switch (_doorMode->getMode())
		{
			case 0: //upper is defrost, but you should never get here
				
				break;
			case 1: //upper is head, lower not used
				diffUpper = _ductObjUpper - _ductUpper->getTemperature();
				diffLower = _ductObjLower - _ductLower->getTemperature();
				break;
			case 2: //upper is head, lower is used
				diffUpper = _ductObjUpper - _ductUpper->getTemperature();
				diffLower = _ductObjLower - _ductLower->getTemperature();
				break;
			case 3: //upper is not used
				diffUpper = _ductObjUpper - _ductDefrost->getTemperature();
				diffLower = _ductObjLower - _ductLower->getTemperature();
				break;
			case 4: //upper is defrost, lower is used
				diffUpper = _ductObjUpper - _ductDefrost->getTemperature();
				diffLower = _ductObjLower - _ductLower->getTemperature();
				break;
			
		}
		Serial.print("Diffs: ");
		Serial.print(diffUpper);
		
		Serial.print(" | ");
		Serial.println(diffLower);
		
		_doorUpper->setTarget(constrain(_doorUpper->getTarget() + diffUpper, 0, 255));
		_doorLower->setTarget(constrain(_doorLower->getTarget() + diffLower, 0, 255));
		
		Serial.print("New Targets: ");
		Serial.print(_doorUpper->getTarget());
		Serial.print(" | ");
		Serial.println(_doorLower->getTarget());
		
		
	};
};

//this is a manual climate control stub that links the user requested temperature directly to A/M door position
void Climate::manualTempControl()
{
	if (millis() - _timeLastTempAdjust >= _intervalTempAdjust)
	{
		_doorUpper->setTarget(constrain(map(_currentTargetTemp, minTargetTemp, maxTargetTemp, 0, 255), 0, 255));
		_doorLower->setTarget(constrain(map(_currentTargetTemp, minTargetTemp, maxTargetTemp, 0, 255), 0, 255));
	}
}
//provides for minimum compressor cycle time, max frequency is approximate 1 cycle/minute
void Climate::setCompressor(bool state)
{
	if ((state != _lastState) & (millis()-_timeLastCompressorToggle > _minCompressorDur))
	{
		_timeLastCompressorToggle = millis();
		_lastState = state;
		if (state) {digitalWrite(_rCompressor, HIGH);}
		else {digitalWrite(_rCompressor, LOW);};
	};
	
};

void Climate::end(uint8_t addr)
{
	EEPROM.put(addr, _currentTargetTemp);
};
