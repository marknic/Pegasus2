#include "AltitudePressure.h"



AltitudePressure::AltitudePressure()
{
	int _totalCount = 0;
	int _currentPointer = 0;
	int _currentAltitude = 0;

}


AltitudePressure::~AltitudePressure()
{
}

void AltitudePressure::addValues(int altitude, double pressure)
{
	_altitude[_totalCount] = altitude;
	_pressure[_totalCount] = pressure;
	_totalCount++;
}

double AltitudePressure::getPressure(int altitude)
{
	
	if (altitude > _currentAltitude)
	{
		while (_altitude[_currentPointer] < altitude)
		{
			_currentPointer++;
		}	
		_currentPointer--;
	}
	else
	{
		while (_altitude[_currentPointer] > altitude)
		{
			_currentPointer--;
		}	
		_currentPointer++;
	}
	
	if (_currentPointer < 0) return 0.0;
	if (_currentPointer >= ALT_PRESS_ARRAY_COUNT) return 0.0;
	
	_currentAltitude = _altitude[_currentPointer];
	
	return _pressure[_currentPointer];
}
