//#include "i2cdevice.h"
#include "GPIO.h"
//#include<iostream>

#define RightMotor_EnablePin    67
#define RightMotor_ForwardPin   68
#define RightMotor_BackwardPin  44

#define LeftMotor_EnablePin     26
#define LeftMotor_ForwardPin    46
#define LeftMotor_BackwardPin   65

#define I2C_bus     0
#define I2C_device  0

using namespace exploringBB;

	GPIO RM_EN(RightMotor_EnablePin); //RM_EN.setDirection(OUTPUT); RM_EN.setValue(HIGH);
	GPIO RM_FW(RightMotor_ForwardPin); //RM_FW.setDirection(OUTPUT); RM_FW.setValue(LOW);
	GPIO RM_BW(RightMotor_BackwardPin); //RM_BW.setDirection(OUTPUT); RM_BW.setValue(LOW);


	GPIO LM_EN(LeftMotor_EnablePin); //LM_EN.setDirection(OUTPUT); LM_EN.setValue(HIGH);
	GPIO LM_FW(LeftMotor_ForwardPin); //LM_FW.setDirection(OUTPUT); LM_FW.setValue(LOW);
	GPIO LM_BW(LeftMotor_BackwardPin); //LM_BW.setDirection(OUTPUT); LM_BW.setValue(LOW);
	
void stopmotors(){

	RM_FW.setValue(LOW);
        LM_FW.setValue(LOW);
	RM_BW.setValue(LOW);
        LM_BW.setValue(LOW);

}

void forward(){
	//1. stop backward movement
	stopmotors();

	//2. move forward
	RM_FW.setValue(HIGH);
	LM_FW.setValue(HIGH);	

}

void backward(){
	//1. stop forward movement
	stopmotors();

	//2. move backward
	RM_BW.setValue(HIGH);
	LM_BW.setValue(HIGH);
}

void right(){

	stopmotors();
	LM_FW.setValue(HIGH);
	//RM_BW.setValue(HIGH);
}

void left(){
	
	stopmotors();
	RM_FW.setValue(HIGH);
	//LM_BW.setValue(HIGH);
}

void readall(){




}
int main(){

//1. setup motors
while(1){
	RM_EN.setDirection(OUTPUT); RM_EN.setValue(HIGH);
	RM_FW.setDirection(OUTPUT); RM_FW.setValue(LOW);
	RM_BW.setDirection(OUTPUT); RM_BW.setValue(LOW);
	
	LM_EN.setDirection(OUTPUT); LM_EN.setValue(HIGH);
	LM_FW.setDirection(OUTPUT); LM_FW.setValue(LOW);
	LM_BW.setDirection(OUTPUT); LM_BW.setValue(LOW);
	}
//2. initialise I2C device

	//I2CDevice MPU6050(I2C_bus,I2C_device);
	//MPU6050.writeRegister(0x6B,0x00);	//disable sleep mode of MPU6050
//3. implement PID


	


	
return 0;

}
