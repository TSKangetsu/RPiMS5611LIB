#include <iostream>
#include <sys/time.h>　
#include "src/MS5611LIB.h"

int main()
{
	clock_t start, end;
	struct timeval t1, t2;
	double timeuse;

	MS5611 test;
	double tmp[2];
	test.MS5611Init();
	test.LocalPressureSetter(1023);
	while (true)
	{
		gettimeofday(&t1, NULL);
		start = clock();
		test.MS5611PreReader(tmp);
		std::cout  << "Pressure:" << tmp[0] << "\n";
		std::cout << "Altitude:" << tmp[1] << "\n";
		end = clock();
		gettimeofday(&t2, NULL);
		timeuse = (t2.tv_sec - t1.tv_sec) + (double)(t2.tv_usec - t1.tv_usec) / 1000.0;
		std::cout << "CPUtime = " << double(end - start) / 1000.0 << "ms\n";
		std::cout << "Threadtime = " << timeuse << "ms\n";
	}
}