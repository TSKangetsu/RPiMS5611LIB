#pragma once
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <fcntl.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <wiringPi.h>
#define OSR_4096 10000
#define CMD_PROM_READ 0xA2
#define MS5611_ADDRESS 0x77
#define CONV_D1_4096 0x48
#define CONV_D2_4096 0x58

#define MS5611RawPressure 0
#define MS5611FastPressure 1
#define MS5611FilterPressure 2
#define MS5611TmpData 3
#define MS5611Altitude 4
#define MS5611Temp 5

class MS5611
{
public:
	inline bool MS5611Init(int TabSize, double FilterBETA, double FASTJUMPBETA)
	{
		PresureAvaDataSec = new double[TabSize];
		for (size_t i = 0; i < TabSize; i++)
		{
			PresureAvaDataSec[i] = 0;
		}

		TableSize = TabSize;
		FilterBeta = FilterBETA;
		FastJumpBeta = FASTJUMPBETA;
		if ((MS5611FD = open("/dev/i2c-1", O_RDWR)) < 0)
		{
			std::cout << "Failed to open the bus.\n";
			return false;
		}
		if (ioctl(MS5611FD, I2C_SLAVE, MS5611_ADDRESS) < 0)
		{
			std::cout << "Failed to acquire bus access and/or talk to slave.\n";
			return false;
		}
		if (write(MS5611FD, &RESET, 1) != 1)
		{
			std::cout << "write reg 8 bit Failed to write to the i2c bus.\n";
			return false;
		}
		usleep(10000);
		MS5611PROMSettle();
		return true;
	}

	inline void MS5611Calibration(double result[10], bool FastMode)
	{
		double tmp[10] = {0};
		LocalPressureSetter();
		if (FastMode)
		{
			for (size_t i = 0; i < TableSize * 2; i++)
			{
				MS5611FastReader(tmp);
			}
		}
		else
		{
			for (size_t i = 0; i < TableSize * 2; i++)
			{
				MS5611PreReader(tmp);
			}
		}
		tmp[1] = tmp[0] - 10;
		tmp[2] = tmp[0] - 10;
		tmp[3] = tmp[0] - 10;
		while ((int)tmp[0] - 3 >= (int)tmp[2] && (int)tmp[0] + 3 <= (int)tmp[2])
		{
			MS5611FastReader(tmp);
		}

		result[0] = tmp[0];
		result[1] = tmp[1];
		result[2] = tmp[2];
		result[3] = tmp[3];
	}

	inline void LocalPressureSetter(double SeaLevelPressure = 1023)
	{
		LocalPressure = SeaLevelPressure;
	}

	inline int MS5611PreReader(double *result)
	{
		D1 = MS5611CONVReader(MS5611FD, CONV_D1_4096);
		D2 = MS5611CONVReader(MS5611FD, CONV_D2_4096);
		//cac
		dT = D2 - (uint32_t)C[4] * 256;
		TEMP = (2000 + (dT * (int64_t)C[5] / 8388608));
		uint32_t Temp = 0;
		Temp = TEMP;
		OFF = (int64_t)C[1] * 65536 + (int64_t)dT * C[3] / 128;
		SENS = (int32_t)C[0] * 32768 + (int64_t)dT * C[2] / 256;
		if (TEMP < 2000)
		{
			Temp -= (dT * dT) / (2 << 30);
			int64_t OFF1 = 0;
			int64_t SENS1 = 0;
			OFF1 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
			SENS1 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
			if (TEMP < -1500)
			{
				OFF1 = OFF1 + 7 * ((TEMP + 1500) * (TEMP + 1500));
				SENS1 = SENS1 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
			}
			OFF -= OFF1;
			SENS -= SENS1;
		}
		P = ((((int64_t)D1 * SENS) / 2097152.0 - OFF) / 32768.0);
		Pressure = (double)P / (double)100;
		result[MS5611RawPressure] = Pressure * 100;
		PresureAvaTotalSec -= PresureAvaDataSec[PresureClockSec];
		PresureAvaDataSec[PresureClockSec] = result[MS5611RawPressure];
		PresureAvaTotalSec += PresureAvaDataSec[PresureClockSec];
		PresureClockSec++;
		if (PresureClockSec >= TableSize)
			PresureClockSec = 0;
		result[MS5611FastPressure] = PresureAvaTotalSec / (double)TableSize;
		result[MS5611FilterPressure] = result[MS5611FilterPressure] * FilterBeta + (1.0 - FilterBeta) * result[MS5611FastPressure];
		if (FastJumpBeta > 0)
		{
			double diff = result[MS5611TmpData] - result[MS5611FastPressure];
			if (diff > 8)
				diff = 8;
			if (diff < -8)
				diff = -8;
			if (diff > 1 || diff < -1)
				result[MS5611FilterPressure] -= diff / FastJumpBeta;
		}
		double Altitudes = 44330.0f * (1.0f - pow((result[MS5611FilterPressure] / 100.f) / (LocalPressure / 100.f), 0.1902949f));
		result[MS5611Altitude] = Altitudes;
		result[MS5611Temp] = Temp;
	}

	inline int MS5611FastReader(double *result)
	{
		long ret = 0;
		uint8_t D[] = {0, 0, 0};
		int h;
		char zero = 0x0;
		char output;
		if (clockTimer >= TEMPSKIP)
		{
			output = 0x58;
			write(MS5611FD, &output, 1);
			usleep(9800);
			write(MS5611FD, &zero, 1);
			h = read(MS5611FD, &D, 3);
			if (h != 3)
			{
				return -2;
			}
			D2 = D[0] * (unsigned long)65536 + D[1] * (unsigned long)256 + D[2];
			dT = D2 - (uint32_t)C[4] * 256;
			TEMP = (2000 + (dT * (int64_t)C[5] / 8388608));
			uint32_t Temp = 0;
			Temp = TEMP;
			OFF = (int64_t)C[1] * 65536 + (int64_t)dT * C[3] / 128;
			SENS = (int32_t)C[0] * 32768 + (int64_t)dT * C[2] / 256;
			if (TEMP < 2000)
			{
				Temp -= (dT * dT) / (2 << 30);
				int64_t OFF1 = 0;
				int64_t SENS1 = 0;
				OFF1 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
				SENS1 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
				if (TEMP < -1500)
				{
					OFF1 = OFF1 + 7 * ((TEMP + 1500) * (TEMP + 1500));
					SENS1 = SENS1 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
				}
				OFF -= OFF1;
				SENS -= SENS1;
			}
			clockTimer = 0;
			result[MS5611Temp] = Temp;
		}
		else
		{
			output = 0x48;
			write(MS5611FD, &output, 1);
			usleep(9800);
			write(MS5611FD, &zero, 1);
			h = read(MS5611FD, &D, 3);
			if (h != 3)
			{
				return -1;
			}
			D1 = D[0] * (unsigned long)65536 + D[1] * (unsigned long)256 + D[2];
			P = ((((int64_t)D1 * SENS) / 2097152.0 - OFF) / 32768.0);
			Pressure = (double)P / (double)100;
			PresureAvaTotal -= PresureAvaData[PresureClock];
			PresureAvaData[PresureClock] = Pressure;
			PresureAvaTotal += PresureAvaData[PresureClock];
			PresureClock++;
			if (PresureClock >= TEMPSKIP)
				PresureClock = 0;
			result[MS5611RawPressure] = (PresureAvaTotal / TEMPSKIP) * 100;
			clockTimer++;

			PresureAvaTotalSec -= PresureAvaDataSec[PresureClockSec];
			PresureAvaDataSec[PresureClockSec] = result[MS5611RawPressure];
			PresureAvaTotalSec += PresureAvaDataSec[PresureClockSec];
			PresureClockSec++;
			if (PresureClockSec >= TableSize)
				PresureClockSec = 0;
			result[MS5611FastPressure] = PresureAvaTotalSec / (double)TableSize;
			result[MS5611FilterPressure] = result[MS5611FilterPressure] * FilterBeta + (1.0 - FilterBeta) * result[MS5611FastPressure];
			if (FastJumpBeta > 0)
			{
				double diff = result[MS5611TmpData] - result[MS5611FastPressure];
				if (diff > 8)
					diff = 8;
				if (diff < -8)
					diff = -8;
				if (diff > 1 || diff < -1)
					result[MS5611FilterPressure] -= diff / FastJumpBeta;
			}
			double Altitudes = 44330.0f * (1.0f - pow((result[MS5611FilterPressure] / 100.f) / (LocalPressure / 100.f), 0.1902949f));
			result[MS5611Altitude] = Altitudes;
		}
		return 0;
	}

private:
	int MS5611FD;
	char RESET = 0x1E;
	uint16_t C[7];
	uint32_t D1;
	uint32_t D2;
	//cac tmp
	int64_t dT;
	int32_t TEMP;
	int64_t OFF;
	int64_t SENS;
	int32_t P;
	//cac tmp-=
	double LocalPressure = 1023;
	double Altitude = 0;
	double Pressure = 0;
	//cac 100hz
	int clockTimer = 5;
	int TEMPSKIP = 5;

	int TableSize = 0;
	double FilterBeta = 0;
	double FastJumpBeta = 0;

	int PresureClock = 0;
	float PresureAvaData[10] = {0};
	float PresureAvaTotal = 0;

	int PresureClockSec = 0;
	double *PresureAvaDataSec;
	float PresureAvaTotalSec = 0;

	inline void MS5611PROMSettle()
	{
		for (int i = 0; i < 6; i++)
		{
			C[i] = MS5611PROMReader(MS5611FD, CMD_PROM_READ + (i * 2));
			usleep(1000);
		}
	}

	unsigned int MS5611PROMReader(int DA, char PROM_CMD)
	{
		uint16_t ret = 0;
		uint8_t r8b[] = {0, 0};
		if (write(DA, &PROM_CMD, 1) != 1)
		{
			std::cout << "read set reg Failed to write to the i2c bus.\n";
		}
		if (read(DA, r8b, 2) != 2)
		{
			std::cout << "Failed to read from the i2c bus.\n";
		}
		ret = r8b[0] * 256 + r8b[1];
		return ret;
	}

	long MS5611CONVReader(int DA, char CONV_CMD)
	{
		long ret = 0;
		uint8_t D[] = {0, 0, 0};
		int h;
		char zero = 0x0;
		if (write(DA, &CONV_CMD, 1) != 1)
		{
			std::cout << "write reg 8 bit Failed to write to the i2c bus.\n";
		}
		usleep(OSR_4096);
		if (write(DA, &zero, 1) != 1)
		{
			std::cout << "write reset 8 bit Failed to write to the i2c bus.\n";
		}
		h = read(DA, &D, 3);
		if (h != 3)
		{
			std::cout << "Failed to read from the i2c bus %d.\n";
		}
		ret = D[0] * (unsigned long)65536 + D[1] * (unsigned long)256 + D[2];
		return ret;
	}
};