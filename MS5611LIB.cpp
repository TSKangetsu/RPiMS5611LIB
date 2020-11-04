#include <iostream>
#include <sys/time.h>　
#include <wiringPi.h>
#include "src/MS5611LIB.h"

int main()
{
	clock_t start, end;
	struct timeval t1, t2;
	double timeuse;
	double s, e;

	MS5611 test;
	double tmp[2];
	test.MS5611Init();
	test.LocalPressureSetter(1023, 5);
	while (true)
	{
		s = micros();
		// test.MS5611PreReader(tmp);
		test.MS5611FastReader(tmp);
		std::cout << "Pressure:" << tmp[0] << "   ";
		std::cout << "Altitude:" << tmp[1] << "   ";
		e = micros();
		timeuse = e - s;
		std::cout << "CPUtime = " << timeuse << "us  \n";
	}
}