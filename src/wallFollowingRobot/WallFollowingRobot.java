package wallFollowingRobot;

import lejos.hardware.Button;
import lejos.hardware.ev3.EV3;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import java.util.*;

public class WallFollowingRobot {
	// EV3 Motors
	BaseRegulatedMotor mA = new EV3LargeRegulatedMotor(MotorPort.A);
	BaseRegulatedMotor mB = new EV3LargeRegulatedMotor(MotorPort.B);
	BaseRegulatedMotor[] motors = new BaseRegulatedMotor[] { mB };
	int acceleration = 500;
	float motor_speed = 250;

	// EV3 Ultrasonic
	EV3UltrasonicSensor ultrasensor = new EV3UltrasonicSensor(SensorPort.S1);
	SensorMode sonic = (SensorMode) ultrasensor.getDistanceMode();

	EV3TouchSensor touchsensor = new EV3TouchSensor(SensorPort.S3);
	SensorMode touch = touchsensor.getTouchMode();
	float[] sample_touch = new float[touch.sampleSize()];

	float[] sample_ultrasonic = new float[1];
	float distance_from_wall = (float) .300;

	// EV3Gyro
	EV3GyroSensor gyro = new EV3GyroSensor(SensorPort.S2);

	// SensorMode gyroMode = (SensorMode) gyro.getAngleMode();
	float[] sample_gyro = new float[gyro.sampleSize()];
	//

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		// Set up the motors.
		WallFollowingRobot ev3 = new WallFollowingRobot();
		ev3.gyro.setCurrentMode("Angle");

		ev3.ultrasensor.setCurrentMode("Distance");

		ev3.touchsensor.setCurrentMode("Touch");
		ev3.setUpMotors(ev3.motor_speed, ev3.acceleration);

		System.out.println("Waiting for button press");
		Button.waitForAnyPress();
		// Go forward until hit wall
		ev3.hitWall();
		ev3.setUpMotors(ev3.motor_speed, ev3.acceleration);

		// turn right 90 degrees.
		ev3.turnRight();

		// reset motors.
		float old_motor = ev3.motor_speed;
		int old_acc = ev3.acceleration;
		ev3.setUpMotors(ev3.motor_speed , ev3.acceleration);

		// Go toward angled wall.
		//current_dist is how far the sensor is from the wall in m.
		float current_dist = ev3.followAngledWall();
		//convert to cm.
		current_dist = current_dist * 100;
		
		
		//Print current distance from wall.
		System.out.println(current_dist);
		//reset speed to forward speed.
		ev3.setUpMotors(old_motor, old_acc);
		ev3.turnLeft();
		//reset motor speed once again (changed in turnLeft)
		ev3.setUpMotors(old_motor, old_acc);
		//figure out how many rotations we want to rotate the wheel to get the sensor even with the wall.
		int degrees_to_rotate = ev3.convertCmToDegree(current_dist);
		ev3.goToGoal(degrees_to_rotate + 1530);

		ev3.close();

	}

	public int convertCmToDegree(float cm) {
		System.out.println(cm);
		float circumference = (float) 17.59;
		float rotations = cm / circumference;
		int degrees = Math.round(360 * rotations);
		return degrees;

	}

	public void goToGoal(int degrees) {
		System.out.println("Rotating " + degrees + " degrees.");
		mA.synchronizeWith(motors);
		mA.startSynchronization();
		mA.rotate(degrees);
		mB.rotate(degrees);
		mA.endSynchronization();
		this.mA.waitComplete();
		this.mB.waitComplete();
	}



	public float followAngledWall() {
		//To guarantee that we are 30 cm from the wall, aim low (roughly 10-15cm away from wall)
		float lower_bound_from_wall = (float) .14;
		float upper_bound_from_wall = (float) .20;
		ultrasensor.fetchSample(sample_ultrasonic, 0);
		float current_dist = (float) 0.0;
		mA.forward();
		mB.forward();
		while (sample_ultrasonic[sample_ultrasonic.length - 1] < Float.POSITIVE_INFINITY) {
			current_dist = sample_ultrasonic[sample_ultrasonic.length - 1];
			System.out.println(sample_ultrasonic[sample_ultrasonic.length - 1]);
			// Check below value
			if (sample_ultrasonic[sample_ultrasonic.length - 1] < lower_bound_from_wall) {
				System.out.println("Going away from wall");
				mA.setSpeed(motor_speed * (float) 1.5);
				mB.setSpeed(motor_speed);
			} else if (sample_ultrasonic[sample_ultrasonic.length - 1] > upper_bound_from_wall) {
				System.out.println("Going to wall");

				mA.setSpeed(motor_speed);
				mB.setSpeed(motor_speed * (float) 1.5);

			} else {
				System.out.println("Going straight");
				mA.setSpeed(motor_speed);
				mB.setSpeed(motor_speed);
			}
			ultrasensor.fetchSample(sample_ultrasonic, 0);
		}
		
		//Sensor has cleared wall.
		mA.stop();
		mB.stop();
		//Go forward 5 cm so wheel clears the wall.
		mA.synchronizeWith(motors);
		mA.startSynchronization();
		//5 cm is ~103 degrees for these wheels.
		mA.rotate(103);
		mB.rotate(103);
		mA.endSynchronization();
		mA.waitComplete();
		mB.waitComplete();

		return current_dist;

	}

	public void getCloseToWall(float distance_from_wall) {
		float old_speed = this.motor_speed;
		int old_acc = this.acceleration;

		mB.setSpeed(2 * motor_speed);
		mA.synchronizeWith(motors);
		mA.startSynchronization();
		mA.forward();
		mB.forward();
		mA.endSynchronization();
		long startTime = System.currentTimeMillis();

		while (true) {
			long movingTime = System.currentTimeMillis() - startTime;
			if (990 <= movingTime && movingTime <= 1001) {
				mA.stop(true);
				mB.stop(true);
				break;
			}
		}

		mA.waitComplete();
		mB.waitComplete();

		setUpMotors(old_speed, old_acc);
	}

	public void getAwayFromWall(float distance_from_wall) {
		float old_speed = this.motor_speed;
		int old_acc = this.acceleration;
		mA.setSpeed(2 * motor_speed);
		mA.synchronizeWith(motors);
		mA.startSynchronization();
		mA.forward();
		mB.forward();
		mA.endSynchronization();

		long startTime = System.currentTimeMillis();
		while (true) {
			long movingTime = System.currentTimeMillis() - startTime;
			if (990 <= movingTime && movingTime <= 1001) {
				mA.stop(true);
				mB.stop(true);
				break;
			}
		}
		mA.waitComplete();
		mB.waitComplete();

		setUpMotors(old_speed, old_acc);

	}

	public void setUpMotors(float motor_speed, int acceleration) {
		// Configures Motors with custom acceleration and speed.
		// System.out.println("(RE)SETTING MOTORS");
		mB.setAcceleration(acceleration);
		mA.setAcceleration(acceleration);
		mB.setSpeed(motor_speed);
		mA.setSpeed(motor_speed);
		this.motor_speed = motor_speed;
		this.acceleration = acceleration;
	}

	public void printStats() {
		ultrasensor.fetchSample(sample_ultrasonic, 0);
		gyro.fetchSample(sample_gyro, 0);
		System.out.println("Sample Distance: " + sample_ultrasonic[sample_ultrasonic.length - 1]);
		System.out.println("Sample Gyro: Angle: " + sample_gyro[sample_gyro.length - 1]);
	}

	public void hitWall() {

		mA.forward();
		mB.forward();
		while (mA.isMoving() && mB.isMoving()) {
			touch.fetchSample(sample_touch, 0);
			if (sample_touch[sample_touch.length - 1] == 1) {
				mA.stop(true);
				mB.stop(true);
			}
		}

		// go back 22 cm
		this.mA.synchronizeWith(motors);
		this.mA.startSynchronization();
		this.mA.rotate(-500);
		this.mB.rotate(-500);
		this.mA.endSynchronization();
		mA.waitComplete();
		mB.waitComplete();

	}

	public void turnRight() {
		//
		mA.stop(true);
		mB.stop(true);
		mA.waitComplete();
		mB.waitComplete();

		printStats();
		gyro.reset();
		printStats();
		mA.setSpeed(motor_speed / 2);
		mA.forward();
		//Using -88.5 rather than -90 to correct for over rotation. 
		while (sample_gyro[sample_gyro.length - 1] > -88.5) {
			gyro.fetchSample(sample_gyro, 0);
			System.out.println("Sample Gyro: Angle: " + sample_gyro[sample_gyro.length - 1]);

		}
		mA.stop(true);
	}

	public void turnLeft() {
		this.mA.stop(true);
		this.mB.stop(true);
		this.mA.waitComplete();
		this.mB.waitComplete();

		printStats();

		this.mB.setSpeed(motor_speed / 2);
		this.mB.forward();
		//Using -1 rather than 0 to correct for overrotation. 
		while (sample_gyro[sample_gyro.length - 1] < -1) {
			gyro.fetchSample(sample_gyro, 0);
			System.out.println("Sample Gyro: Angle: " + sample_gyro[sample_gyro.length - 1]);

		}
		mB.stop(true);

	}

	public void close() {
		mA.close();
		mB.close();
		ultrasensor.close();
		gyro.close();
	}

}
