#include "i2cdevice.h"

int main(){

	exploringBB::I2CDevice mpu6050(2,0x68);

	unsigned char ID = mpu6050.readRegister(0x75);
	printf("ID is %c", ID);




return 0;
}
