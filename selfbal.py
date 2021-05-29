import smbus
import time
import math
import Adafruit_BBIO.PWM as PWM


PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

#PID parameters
Kp = 2.5
Ki = 0.8
Kd = 8.0
K  = 1.9*1.12


#Complimentary Filter parameters
K0 =  0.98
K1 =  0.02


#initializing motors
LF="P8_13"
LB="P8_19"
RF="P9_21"
RB="P9_22"

PWM.start(LF,0,1000)
PWM.start(LB,0,1000)
PWM.start(RF,0,1000)
PWM.start(RB,0,1000)

def stop_motors():
	PWM.set_duty_cycle(LF,0)
	PWM.set_duty_cycle(LB,0)
	PWM.set_duty_cycle(RF,0)
	PWM.set_duty_cycle(RB,0)

def motors(speed,left_offset,right_offset):
	left_speed = speed + left_offset
	right_speed = speed + right_offset
	
	#left motor
	if left_speed<0:
		PWM.set_duty_cycle(LF,-left_speed)
		PWM.set_duty_cycle(LB,0)
	elif left_speed>0:
		PWM.set_duty_cycle(LB,left_speed)                
		PWM.set_duty_cycle(LF,0)        

	#right motor
	if right_speed<0:
		PWM.set_duty_cycle(RF,-right_speed)
		PWM.set_duty_cycle(RB,0)
	elif right_speed>0:
		PWM.set_duty_cycle(RB,right_speed)
		PWM.set_duty_cycle(RF,0)

def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)
	

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


def dist(a,b):
	return math.sqrt((a*a) + (b*b))
	

def get_y_rotation(x,y,z):
	radians = math.atan2(x, dist(y, z))
	return -(radians * (180.0 / math.pi))

  	
def get_x_rotation(x,y,z):
	radians = math.atan2(y, dist(x, z))
	return (radians * (180.0 / math.pi))
  
	

def read_all():
	global accl_scaled_x, accl_scaled_y,accl_scaled_z
	global gyro_scaled_x, gyro_scaled_y, gyro_scaled_z 
	
	acclX = read_raw_data(ACCEL_XOUT_H)
	acclY = read_raw_data(ACCEL_YOUT_H)
	acclZ = read_raw_data(ACCEL_ZOUT_H)

	gyroX = read_raw_data(GYRO_XOUT_H)
	gyroY = read_raw_data(GYRO_YOUT_H)
	gyroZ = read_raw_data(GYRO_ZOUT_H)

	accl_scaled_x = acclX / 16384.0
	accl_scaled_y = acclY / 16384.0
	accl_scaled_z = acclZ / 16384.0

	gyro_scaled_x = gyroX/131.0
	gyro_scaled_y = gyroY/131.0
	gyro_scaled_z = gyroZ/131.0
	

	#print(accl_scaled_x, accl_scaled_y, accl_scaled_z, gyro_scaled_x, gyro_scaled_y, gyro_scaled_z)
	
	time.sleep(0.5)
	
def constrain(v, min_v, max_v):
	if (v <= min_v):
		return min_v
	elif (v >= max_v):
		return max_v
	else:
		return v
    		
GUARD_GAIN = 100.0
#angle_offset = 2.0   		
    		
def pid():
	global error,speed,pTerm,iTerm,dTerm
	error = last_y - angle_offset;
	pTerm = Kp * error
	integrated_error=0
	integrated_error = 0.95*integrated_error + error;
	iTerm = Ki * integrated_error;
	last_error=error;
	dTerm = Kd * (error - last_error);
	#last_error = error;
	speed = constrain(K*(pTerm + iTerm + dTerm), -GUARD_GAIN, GUARD_GAIN)	   		

def getTimestamp():
	return int(time.time())   		

if __name__ == "__main__":
	
	bus = smbus.SMBus(2) 	# or bus = smbus.SMBus(0) for older version boards

	Device_Address = 0x68   # MPU6050 device address

	MPU_Init()

	print (" Reading Data of Gyroscope and Accelerometer")
	timer = getTimestamp()
	#deltaT = (getTimestamp() - timer)/1000000.0
	read_all()
	deltaT = (getTimestamp() - timer)/1000000.0

	last_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z)
	last_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z)


	gyro_offset_x = gyro_scaled_x
	gyro_offset_y = gyro_scaled_y

	gyro_total_x = last_x - gyro_offset_x
	gyro_total_y = last_y - gyro_offset_y
	
	
	gyro_y_delta = (gyro_total_y * deltaT)
	angle_offset = K0 * (last_y + gyro_y_delta) + (K1 * last_y)
	
	while True:
		t = getTimestamp()
		deltaT = (t - timer)/1000000.0
		timer = t

		read_all()

		gyro_scaled_x -= gyro_offset_x
		gyro_scaled_y -= gyro_offset_y

		gyro_x_delta = (gyro_scaled_x * deltaT)
		gyro_y_delta = (gyro_scaled_y * deltaT)

		gyro_total_x += gyro_x_delta
		gyro_total_y += gyro_y_delta

		rotation_x = get_x_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z)
		rotation_y = get_y_rotation(accl_scaled_x, accl_scaled_y, accl_scaled_z)

		#    printf("[BEFORE] gyro_scaled_y=%f, deltaT=%lf, rotation_y=%f, last_y= %f\n", (double)gyro_scaled_y, (double)deltaT, (double)rotation_y, (double) last_y);

		#   printf("[1st part] = %f\n", (double) K0*(last_y + gyro_y_delta));
		#   printf("[2nd part] = %f\n", (double) K1*rotation_y);
		last_x = K0 * (last_x + gyro_x_delta) + (K1 * rotation_x)
		last_y = K0 * (last_y + gyro_y_delta) + (K1 * rotation_y)

		#    printf("[AFTER] gyro_scaled_y=%f, deltaT=%lf, rotation_y=%f, last_y=%f\n", (double)gyro_scaled_y, (double)deltaT, (double)rotation_y, (double) last_y);

		if (last_y < -60.0 || last_y > 60.0) 
		stop_motors()

		pid()
		print(error, speed, pTerm, iTerm, dTerm)

	    	motors(speed, 0.0, 0.0)
	    
	    	time.sleep(10)
	  

	  	stop_motors();

PWM.stop(LF)
PWM.stop(LB)
PWM.stop(RF)
PWM.stop(RB)
PWM.cleanup()
	  
    		
    	
