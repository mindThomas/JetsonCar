#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>

#include <i2c_devices/VL53L0X.h>
#include <i2c_devices/ADXL345.h>
#include <i2c_devices/ITG3200.h>
#include <i2c_devices/HMC58X3.h>

#define LASER_TEST
//#define ACCELEROMETER_TEST
//#define GYRO_TEST
//#define MAGNETOMETER_TEST

int main(int argc, char** argv)
{
	printf("--- Starting I2C test ---\n");

#ifdef LASER_TEST
	VL53L0X laser;
	bool errCode;
	uint16_t millimeters;

	if (!laser.init()) {
		printf("errCode: %d\n", laser.last_status);
	}

	laser.setTimeout(500);
        laser.startContinuous();
#endif


#ifdef ACCELEROMETER_TEST   
	ADXL345 acc;
	int16_t xa,ya,za;

	acc.powerOn();
  	acc.setRangeSetting(2);       // Give the range settings
                                      // Accepted values are 2g, 4g, 8g or 16g
                                      // Higher Values = Wider Measurement Range
                                      // Lower Values = Greater Sensitivity
#endif


#ifdef GYRO_TEST   
	ITG3200 gyro;
	float xg,yg,zg,temperature;

	// Use ITG3200_ADDR_AD0_HIGH or ITG3200_ADDR_AD0_LOW as the ITG3200 address 
	// depending on how AD0 is connected on your breakout board, check its schematics for details
	gyro.init(ITG3200_ADDR_AD0_LOW); 

	printf("Gyro: Calibrating for 5 seconds. Please wait...\n");
  	gyro.zeroCalibrate(2500, 2);
#endif


#ifdef MAGNETOMETER_TEST   
	HMC58X3 mag;
	int xm,ym,zm;

	mag.init(); // Dont set mode yet, we'll do that later on.

	printf("Magnetometer: Calibrating for 10 seconds. Please wait...\n");
	mag.calibrate(0, 64);
	mag.setMode(0); // set continuous measurement mode
#endif

	while (1) {
		printf("\033[2J");
		printf("\033[0;0H");
		#ifdef LASER_TEST
		  millimeters = laser.readRangeContinuousMillimeters();
		  printf("Laser\tDist = %04d mm\n", millimeters);
		  if (laser.timeoutOccurred()) { printf("TIMEOUT\n"); }
		#endif

		#ifdef ACCELEROMETER_TEST
		  // Accelerometer Readings
		  acc.readAccel(&xa, &ya, &za);         // Read the accelerometer values and store them in variables declared above x,y,z
		  printf("Acc\tX,Y,Z = %03d,\t %03d,\t %03d\n", xa, ya, za);
		#endif		

		#ifdef GYRO_TEST 
		  if (gyro.isRawDataReady()) {
			//gyro.readTemp(&temperature);
			gyro.readGyro(&xg,&yg,&zg);
			printf("Gyro\tX,Y,Z = %04.1f,\t %04.1f,\t %04.1f\n", xg, yg, zg);
		  }
		#endif

		#ifdef MAGNETOMETER_TEST
		  mag.getValues(&xm,&ym,&zm);
		  printf("Mag\tX,Y,Z = %03d,\t %03d,\t %03d\n", xm, ym, zm);
		#endif
		usleep(10000);
	}

}
