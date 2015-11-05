from mpu9150_driver import Mpu9150
import time

imu = Mpu9150(0x69);
imu.calibrate();

while True:
	imu.update();
	print imu.raw_data();
	time.sleep(0.2);
