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
	test.MS5611Init();
	test.MS5611Calibration(tmp);
	test.LocalPressureSetter(tmp[2], 5);
	while (true)
	{
		s = micros();
		// test.MS5611PreReader(tmp);
		test.MS5611FastReader(tmp);
		std::cout << "Pressure:" << tmp[2] << "   ";
		std::cout << "Altitude:" << tmp[4] << "   ";
		e = micros();
		timeuse = e - s;
		std::cout << "CPUtime = " << timeuse << "us  \n";
	}
}