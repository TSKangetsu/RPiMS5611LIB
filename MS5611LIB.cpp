#include <iostream>
#include "MS5611LIB.h"

int main()
{
	MS5611 test;
	test.MS5611Init();
	test.MS5611PROMSettle();
	while (true)
	{
		test.MS5611PreReader();
		std::cout << test.Pressure << "\n";
	}
}
