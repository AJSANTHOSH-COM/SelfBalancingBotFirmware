//#include "init.h"

#include "GPIO.h"
#include "i2cdevice.h"
#include <stdint.h>
#include<cmath>
#include <unistd.h>	
#include  <sys/time.h>

#define RightMotor_EnablePin    67
#define RightMotor_ForwardPin   68
#define RightMotor_BackwardPin  44

#define LeftMotor_EnablePin     26
#define LeftMotor_ForwardPin    46
#define LeftMotor_BackwardPin   65

using namespace exploringBB;

exploringBB::I2CDevice mpu6050(2,0x68);

GPIO RM_EN(RightMotor_EnablePin); //RM_EN.setDirection(OUTPUT); RM_EN.setValue(HIGH);
GPIO RM_FW(RightMotor_ForwardPin); //RM_FW.setDirection(OUTPUT); RM_FW.setValue(LOW);
GPIO RM_BW(RightMotor_BackwardPin); //RM_BW.setDirection(OUTPUT); RM_BW.setValue(LOW);


GPIO LM_EN(LeftMotor_EnablePin); //LM_EN.setDirection(OUTPUT); LM_EN.setValue(HIGH);
GPIO LM_FW(LeftMotor_ForwardPin); //LM_FW.setDirection(OUTPUT); LM_FW.setValue(LOW);
GPIO LM_BW(LeftMotor_BackwardPin); //LM_BW.setDirection(OUTPUT); LM_BW.setValue(LOW);

long double delay = 10000;

void stopmotors(){

	RM_FW.setValue(LOW);
        LM_FW.setValue(LOW);
	RM_BW.setValue(LOW);
        LM_BW.setValue(LOW);
        
        RM_FW.setDirection(OUTPUT);
        LM_FW.setDirection(OUTPUT);
        RM_BW.setDirection(OUTPUT);
        LM_BW.setDirection(OUTPUT);

}

void forward(double duty){
	//1. stop backward movement
	stopmotors();

	//2. move forward
	RM_FW.setValue(HIGH);
	LM_FW.setValue(HIGH);	
	usleep(delay*duty);
	
	stopmotors();

}

void backward(double duty){
	//1. stop forward movement
	stopmotors();

	//2. move backward
	RM_BW.setValue(HIGH);
	LM_BW.setValue(HIGH);
	usleep(delay*duty);
	
	stopmotors();
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

void motors(double speed){
	if(speed>0){
		forward(speed);
	}
	
	else if(speed <0){
		backward(speed);
	}

}


// PID parameters
double Kp = 2.5;   // 2.5
double Ki = 0.8;   // 1.0
double Kd = 8.0;   // 8.0
double K  = 1.9*1.12;


// Complimentary Filter parameters
double K0 = (double) 0.98;
double K1 = (double) 0.02;

struct timeval tv, tv2;
unsigned long long      timer, t;
double deltaT;

int16_t  acclX, acclY, acclZ, gyroX, gyroY, gyroZ;
double accl_scaled_x, accl_scaled_y, accl_scaled_z, gyro_scaled_x, gyro_scaled_y, gyro_scaled_z;

double gyro_offset_x, gyro_offset_y;
double gyro_total_x, gyro_total_y;
double gyro_x_delta, gyro_y_delta;
double rotation_x, rotation_y;
double last_x, last_y;


unsigned long long  getTimestamp()
{
  gettimeofday(&tv, NULL);
  return (unsigned long long) tv.tv_sec * 1000000 + tv.tv_usec;
}

void readall(){

	acclX = (mpu6050.readRegister(0x3B) << 8) + (mpu6050.readRegister(0x3C));
        acclY = (mpu6050.readRegister(0x3D) << 8) + (mpu6050.readRegister(0x3E));
	acclZ = (mpu6050.readRegister(0x3F) << 8) + (mpu6050.readRegister(0x40));

	gyroX = (mpu6050.readRegister(0x43) << 8) + (mpu6050.readRegister(0x44));
        gyroY = (mpu6050.readRegister(0x45) << 8) + (mpu6050.readRegister(0x46));
        gyroZ = (mpu6050.readRegister(0x47) << 8) + (mpu6050.readRegister(0x48));

	accl_scaled_x = acclX / 16384.0;
        accl_scaled_y = acclY / 16384.0;
    	accl_scaled_z = acclZ / 16384.0;

	gyro_scaled_x = gyroX/131.0;
	gyro_scaled_y = gyroY/131.0;
	gyro_scaled_z = gyroZ/131.0;

}


double dist(double a, double b)
{
  return sqrt((a*a) + (b*b));
}

double get_y_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(x, dist(y, z));
  return -(radians * (180.0 / M_PI));
}

double get_x_rotation(double x, double y, double z)
{
  double radians;
  radians = atan2(y, dist(x, z));
  return (radians * (180.0 / M_PI));
}



double constrain(double v, double min_v, double max_v)
{
  if (v <= min_v)
    return (double)min_v;
  else if (v >= max_v)
    return (double)max_v;
  else
    return (double)v;
}

double GUARD_GAIN = 100.0;
double error, last_error, integrated_error;
double pTerm, iTerm, dTerm;
double angle;
double angle_offset = 2.0;  //1.5

double speed;


void pid()
{
  error = last_y - angle_offset;

  pTerm = Kp * error;

  integrated_error = 0.95*integrated_error + error;
  iTerm = Ki * integrated_error;
  
  dTerm = Kd * (error - last_error);
  last_error = error;

  speed = constrain(K*(pTerm + iTerm + dTerm), -GUARD_GAIN, GUARD_GAIN); 
  
}


int main(){
	
	mpu6050.writeRegister(0x6B, 0x00); //disable sleep mode
	stopmotors();
	timer = getTimestamp();
	
	readall();
	
	deltaT = (double) (getTimestamp() - timer)/1000000.0;
	
	last_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
  	last_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);


  	gyro_offset_x = gyro_scaled_x;
  	gyro_offset_y = gyro_scaled_y;

  	gyro_total_x = last_x - gyro_offset_x;
  	gyro_total_y = last_y - gyro_offset_y;
  	
  	while(1){
  	
  	t = getTimestamp();
    	deltaT = (double) (t - timer)/1000000.0;
    	timer = t;
	
	readall();


      	gyro_scaled_x -= gyro_offset_x;
      	gyro_scaled_y -= gyro_offset_y;

      	gyro_x_delta = (gyro_scaled_x * deltaT);
      	gyro_y_delta = (gyro_scaled_y * deltaT);

      	gyro_total_x += gyro_x_delta;
      	gyro_total_y += gyro_y_delta;

      	rotation_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);
      	rotation_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z);

     	last_x = K0 * (last_x + gyro_x_delta) + (K1 * rotation_x);
     	last_y = K0 * (last_y + gyro_y_delta) + (K1 * rotation_y);
    
    	if (last_y < -60.0 || last_y > 60.0){
      		stopmotors();
	}
    	pid();
    	printf("%lf\t%lf\t%lf\t%lf\t%lf\n", error, speed, pTerm, iTerm, dTerm);

    	motors(speed);
    
    	//delay(10);
  	}

  stopmotors();
    
return 0;
}
