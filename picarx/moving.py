import picarx_improved as pcx
import time

def parkRight(car):
	car.stop()
	car.set_dir_servo_angle(0)
	car.forward(70)
	time.sleep(1.25)
	car.stop()

	angleRange = [-30,28]
	car.set_dir_servo_angle(angleRange[0])
	time.sleep(0.1)

	car.backward(70)
	for i in range(angleRange[0],angleRange[1]):
		car.set_dir_servo_angle(i)
		car.backward(70)
		time.sleep(.022)

	car.stop()
	car.set_dir_servo_angle(0)

def parkLeft(car):
	car.stop()
	car.set_dir_servo_angle(0)
	car.forward(70)
	time.sleep(1.2)
	car.stop()

	angleRange = [30,-26]
	car.set_dir_servo_angle(angleRange[0])
	time.sleep(0.1)

	car.backward(70)
	for i in range(angleRange[0],angleRange[1],-1):
		car.set_dir_servo_angle(i)
		car.backward(70)
		time.sleep(.022)

	car.stop()
	car.set_dir_servo_angle(0)

def goForward(car):
	car.stop()
	car.set_dir_servo_angle(0)
	car.forward(70)
	time.sleep(2)
	car.stop()

def veerRight(car):
	car.stop()
	car.set_dir_servo_angle(-15)
	car.forward(70)
	time.sleep(2)
	car.stop()
	car.set_dir_servo_angle(0)

def veerLeft(car):
	car.stop()
	car.set_dir_servo_angle(15)
	car.forward(70)
	time.sleep(2)
	car.stop()
	car.set_dir_servo_angle(0)

def goBackward(car):
	car.stop()
	car.set_dir_servo_angle(0)
	car.backward(70)
	time.sleep(2)
	car.stop()

def veerBackRight(car):
	car.stop()
	car.set_dir_servo_angle(-15)
	car.backward(70)
	time.sleep(2)
	car.stop()
	car.set_dir_servo_angle(0)

def veerBackLeft(car):
	car.stop()
	car.set_dir_servo_angle(15)
	car.backward(70)
	time.sleep(2)
	car.stop()
	car.set_dir_servo_angle(0)

def kTurnLeft(car):
	car.stop()
	car.set_dir_servo_angle(20)
	car.forward(70)
	time.sleep(1.2)
	car.stop()
	car.set_dir_servo_angle(-20)
	car.backward(70)
	time.sleep(1.2)
	car.stop()
	car.set_dir_servo_angle(0)
	car.forward(70)
	time.sleep(1)
	car.stop()

def kTurnRight(car):
	car.stop()
	car.set_dir_servo_angle(-20)
	car.forward(70)
	time.sleep(1.2)
	car.stop()
	car.set_dir_servo_angle(20)
	car.backward(70)
	time.sleep(1.2)
	car.stop()
	car.set_dir_servo_angle(0)
	car.forward(70)
	time.sleep(1)
	car.stop()

if __name__ == "__main__":
	car = pcx.Picarx()
	exitFlag = False
	while not exitFlag:
		command = input('Watchu need?\n')
		if command == 'parkRight':
				parkRight(car)
		elif command == 'parkLeft':
				parkLeft(car)
		elif command == 'goForward':
				goForward(car)
		elif command == 'veerRight':
				veerRight(car)
		elif command == 'veerLeft':
				veerLeft(car)
		elif command == 'goBackward':
				goBackward(car)
		elif command == 'veerBackRight':
				veerBackRight(car)
		elif command == 'veerBackLeft':
				veerBackLeft(car)
		elif command == 'kTurnLeft':
				kTurnLeft(car)
		elif command == 'kTurnRight':
				kTurnRight(car)
		elif command == 'exit':
				exitFlag = True
		else:
				print('Invalid command\n')