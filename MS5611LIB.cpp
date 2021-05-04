#include <iostream>
#include <wiringPi.h>
#include "src/MS5611LIB.h"

int main()
{
	double timeuse;
	double s, e;
	wiringPiSetup();
	MS5611 test;
	double tmp[10];
	test.MS5611Init(20, 0.985, -1);
	test.MS5611Calibration(tmp, true);
	test.LocalPressureSetter(tmp[MS5611FilterPressure]);
	while (true)
	{
		s = micros();
		// test.MS5611PreReader(tmp);
		test.MS5611FastReader(tmp);
		std::cout << "Pressure:" << tmp[MS5611FilterPressure] << "   ";
		std::cout << "Altitude:" << tmp[MS5611Altitude] << "   ";
		std::cout << "Temp:" << tmp[MS5611Temp] << "   ";
		e = micros();
		timeuse = e - s;
		std::cout << "CPUtime = " << timeuse << "us  \n";
	}
}