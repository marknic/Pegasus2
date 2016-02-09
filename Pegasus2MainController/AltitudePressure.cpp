#include "AltitudePressure.h"



AltitudePressure::AltitudePressure()
{
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
	
	
}
