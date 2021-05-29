import time
import Adafruit_BBIO.PWM as PWM
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
		PWM.set_duty_cycle(LB,-left_speed)
		PWM.set_duty_cycle(LF,0)
	elif left_speed>0:
		PWM.set_duty_cycle(LF,left_speed)                
		PWM.set_duty_cycle(LB,0)        

	#right motor
	if right_speed<0:
		PWM.set_duty_cycle(RB,-right_speed)
		PWM.set_duty_cycle(RF,0)
	elif right_speed>0:
		PWM.set_duty_cycle(RF,right_speed)
		PWM.set_duty_cycle(RB,0)


while 1:
	#motors(30,0,0)
	#time.sleep(100)
	stop_motors()
#PWM.stop(myPWM)
PWM.cleanup()

