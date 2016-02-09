#pragma once



class AltitudePressure
{
private:
	int _altitude[8721];
	double _pressure[8721];
	int _totalCount = 0;
	int _currentPointer = 0;
	int _currentAltitude = 0;
	
public:
    AltitudePressure();
    ~AltitudePressure();
	
	void addValues(int altitude, double pressure);
	double getPressure(int altitude);
};

