package main.java;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class MemoryDriverTest {
	
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private EV3IRSensor frontIRSensor;
	private EV3UltrasonicSensor leftSonar, rightSonar;
	private EV3GyroSensor gyroscope;
	private SensorMode IRDistance;
	private SampleProvider gyroAngler;
	private String medium, soft, harsh;
	
	private int defaultMoveTime, startCountDown;
	private float[] frontWallDistance, leftWallDistance, rightWallDistance, gyroAngleResult;
	private float frontWallSafeDistance, sideWallSafeDistance, defaultMotorSpeed, 
	softAngle, mediumAngle, harshAngle, gyroOrientationAngle, gyroTotalAngleChange;
	
	
	public MemoryDriverTest() {
		leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
		rightMotor = new EV3LargeRegulatedMotor(MotorPort.D);
		frontIRSensor = new EV3IRSensor(SensorPort.S2);
		IRDistance = frontIRSensor.getDistanceMode();
		leftSonar = new EV3UltrasonicSensor(SensorPort.S4);
		rightSonar = new EV3UltrasonicSensor(SensorPort.S1);
		gyroscope = new EV3GyroSensor(SensorPort.S3);
		gyroAngler = gyroscope.getAngleMode();
		
		defaultMotorSpeed = leftMotor.getMaxSpeed();
		defaultMoveTime = 100;
		startCountDown = 5200;
		frontWallSafeDistance = 25.0f;
		sideWallSafeDistance = 0.20f;
		
		softAngle = 5.0f;
		mediumAngle = 40.0f;
		harshAngle = 45.0f;
		
		harsh = "harsh";
		medium = "medium";
		soft = "soft";
		
		frontWallDistance = new float[2];
		leftWallDistance = new float[2];
		rightWallDistance = new float[2];
		gyroAngleResult = new float[2];
	}
	
	public void drive(){
		//no memory run
		LCD.clear();
		writeScreenMSG("no-mem-run", 0, 0);
		writeScreenMSG("Press Enter!", 0, 2);
		Button.ENTER.waitForPress();
		writeScreenMSG("5sec count!", 0, 4);
		Delay.msDelay(startCountDown);
		writeScreenMSG("GO GO GO!!!", 0, 6);
		runNoMemory();

		leftMotor = null;
		rightMotor = null;
		frontIRSensor = null;
		gyroscope = null;
		
		System.gc();
		LCD.clear();
		LCD.drawString("Finished!", 0, 0);
		Delay.msDelay(5000);
		System.exit(0);
	}
	
	private void fetchSamples(){
		IRDistance.fetchSample(frontWallDistance, 0);
		leftSonar.fetchSample(leftWallDistance, 0);
		rightSonar.fetchSample(rightWallDistance, 0);
	}
	
	private void turnLeft(float motorSpeed, String turnType) {
		//TODO: CALIBRATE TURNS
		rightWallDistance[1] = rightWallDistance[0];
		gyroscope.reset();
		switch(turnType){
			case "harsh":
				while(rightWallDistance[0] <= rightWallDistance[1] && frontWallDistance[0] < frontWallSafeDistance && Button.ESCAPE.isUp()){
					leftMotor.setSpeed(motorSpeed);
					rightMotor.setSpeed(motorSpeed);
					leftMotor.backward();
					rightMotor.forward();
					Delay.msDelay(defaultMoveTime);
					moveForward(defaultMotorSpeed);
					rightSonar.fetchSample(rightWallDistance, 0);
					rightSonar.fetchSample(rightWallDistance, 1);
					frontIRSensor.fetchSample(frontWallDistance, 0);
				}
				break;
			case "medium":
				while(rightWallDistance[0] <= rightWallDistance[1] && frontWallDistance[0] < frontWallSafeDistance && Button.ESCAPE.isUp()){
					leftMotor.setSpeed(motorSpeed/6);
					rightMotor.setSpeed(motorSpeed);
					leftMotor.forward();
					rightMotor.forward();
					Delay.msDelay(defaultMoveTime);
					moveForward(defaultMotorSpeed);
					rightSonar.fetchSample(rightWallDistance, 0);
					rightSonar.fetchSample(rightWallDistance, 1);
					frontIRSensor.fetchSample(frontWallDistance, 0);
				}
				break;
			case "soft":
				while(rightWallDistance[0] <= rightWallDistance[1] && frontWallDistance[0] < frontWallSafeDistance && Button.ESCAPE.isUp()){
					leftMotor.setSpeed(motorSpeed/2);
					rightMotor.setSpeed(motorSpeed);
					leftMotor.forward();
					rightMotor.forward();
					Delay.msDelay(defaultMoveTime);
					moveForward(defaultMotorSpeed);
					rightSonar.fetchSample(rightWallDistance, 0);
					rightSonar.fetchSample(rightWallDistance, 1);
					frontIRSensor.fetchSample(frontWallDistance, 0);
				}
				break;
			default:
				break;
		}
		gyroTotalAngleChange += getGyroAngle(0);
		moveForward(defaultMotorSpeed);
	}
	
	private void turnRight(float motorSpeed, String turnType){
		//TODO: CALIBRATE TURNS
		leftWallDistance[1] = leftWallDistance[0];
		gyroscope.reset();
		switch(turnType){
			case "harsh":
				while(leftWallDistance[0] <= leftWallDistance[1] && frontWallDistance[0] < frontWallSafeDistance && Button.ESCAPE.isUp()){
					leftMotor.setSpeed(motorSpeed);
					rightMotor.setSpeed(motorSpeed);
					leftMotor.forward();
					rightMotor.backward();
					Delay.msDelay(defaultMoveTime);
					moveForward(defaultMotorSpeed);
					leftSonar.fetchSample(leftWallDistance, 0);
					leftSonar.fetchSample(leftWallDistance, 1);
					frontIRSensor.fetchSample(frontWallDistance, 0);
				}
				break;
			case "medium":
				while(leftWallDistance[0] <= leftWallDistance[1] && frontWallDistance[0] < frontWallSafeDistance && Button.ESCAPE.isUp()){
					leftMotor.setSpeed(motorSpeed);
					rightMotor.setSpeed(motorSpeed/6);
					leftMotor.forward();
					rightMotor.forward();
					Delay.msDelay(defaultMoveTime);
					moveForward(defaultMotorSpeed);
					leftSonar.fetchSample(leftWallDistance, 0);
					leftSonar.fetchSample(leftWallDistance, 1);
					frontIRSensor.fetchSample(frontWallDistance, 0);
				}
				break;
			case "soft":
				while(leftWallDistance[0] <= leftWallDistance[1] && frontWallDistance[0] < frontWallSafeDistance && Button.ESCAPE.isUp()){
					leftMotor.setSpeed(motorSpeed);
					rightMotor.setSpeed(motorSpeed/2);
					leftMotor.forward();
					rightMotor.forward();
					Delay.msDelay(defaultMoveTime);
					moveForward(defaultMotorSpeed);
					leftSonar.fetchSample(leftWallDistance, 0);
					leftSonar.fetchSample(leftWallDistance, 1);
					frontIRSensor.fetchSample(frontWallDistance, 0);
				}
				break;
			default:
				break;
		}
		gyroTotalAngleChange += getGyroAngle(0);
		moveForward(defaultMotorSpeed);
	}
	
	private void backPedal(float motorSpeed, int moveTime){
		leftMotor.setSpeed(motorSpeed);
		rightMotor.setSpeed(motorSpeed);
		leftMotor.backward();
		rightMotor.backward();
		Delay.msDelay(moveTime);
		stopMotors();
	}
	
	private void moveForward(float moveSpeed){
		leftMotor.setSpeed(moveSpeed);
		rightMotor.setSpeed(moveSpeed);
		leftMotor.forward();
		rightMotor.forward();
	}

	private void stopMotors(){
		leftMotor.setSpeed(0);
		rightMotor.setSpeed(0);
		leftMotor.stop();
		rightMotor.stop();
	}
	
	//TODO:
	private void runNoMemory(){
		gyroOrientationAngle = 0;
		gyroTotalAngleChange = 0;
		while(Button.ESCAPE.isUp()){
			gyroscope.reset();
			fetchSamples();
			if(gyroTotalAngleChange <= -95.0f && gyroTotalAngleChange >= -275.0f ){
				gyroOrientationAngle = -180;
			}else{
				gyroOrientationAngle = 0;
			}
			//check for walls in front
			if(frontWallDistance[0] >= frontWallSafeDistance){
				//now check your sides
				if(leftWallDistance[0] >= sideWallSafeDistance && 
						rightWallDistance[0] >= sideWallSafeDistance){
					//you are free to drive forward
					while(frontWallDistance[0] >= frontWallSafeDistance && 
							leftWallDistance[0] >= sideWallSafeDistance && 
							rightWallDistance[0] >= sideWallSafeDistance &&
							Button.ESCAPE.isUp()){
						moveForward(defaultMotorSpeed);
						fetchSamples();
					}
				}else{
					//something is not right at LEFT or RIGHT
					if(leftWallDistance[0] < sideWallSafeDistance){
						//left side wrong
						while(leftWallDistance[0] < sideWallSafeDistance && Button.ESCAPE.isUp()){
							turnRight(defaultMotorSpeed, soft);
							fetchSamples();
						}
					}else{
						//right side wrong
						while(rightWallDistance[0] < sideWallSafeDistance && Button.ESCAPE.isUp()){
							turnLeft(defaultMotorSpeed, soft);
							fetchSamples();
						}
					}
				}
			//front is blocked
			}else if(frontWallDistance[0] < frontWallSafeDistance){
				//check which wall is closer => move to opposite
				//slow down and scan for wall
				moveForward(defaultMotorSpeed/10);
				//start scan for 300ms
				for(int i = 0; i < 3; i++){
					fetchSamples();
					if(!Float.isInfinite(leftWallDistance[0]) && !Float.isInfinite(rightWallDistance[0])){
						leftWallDistance[1] = leftWallDistance[0];
						rightWallDistance[1] = rightWallDistance[0];
					}
					Delay.msDelay(defaultMoveTime);
				}
				//okay, scan is finished, check if you're in the wall
				frontIRSensor.fetchSample(frontWallDistance, 0);
				if(frontWallDistance[0] == 0.0){
					//if so - backPedal
					stopMotors();
					Delay.msDelay(200);
					backPedal(defaultMotorSpeed, defaultMoveTime*4);
				}
				//now check your scans and turn left or right
				if((rightWallDistance[1] < leftWallDistance[1]) || 
				  (Float.isInfinite(leftWallDistance[0]) && !Float.isInfinite(rightWallDistance[0]))){
					while(frontWallDistance[0] < frontWallSafeDistance+10 && Button.ESCAPE.isUp()){
						turnLeft(defaultMotorSpeed, medium);
						fetchSamples();
					}
				}else if((rightWallDistance[1] > leftWallDistance[1]) || 
						(Float.isInfinite(rightWallDistance[0]) && !Float.isInfinite(leftWallDistance[0]))){
					while(frontWallDistance[0] < frontWallSafeDistance+10 && Button.ESCAPE.isUp()){
						turnRight(defaultMotorSpeed, medium);
						fetchSamples();
					}
				}else{
					//no results were found => use gyro...
					if(gyroTotalAngleChange < gyroOrientationAngle){
						while(frontWallDistance[0] < frontWallSafeDistance+10){
							turnRight(defaultMotorSpeed, harsh);
						}
					}else if(gyroTotalAngleChange > gyroOrientationAngle){
						while(frontWallDistance[0] < frontWallSafeDistance+10){
							turnLeft(defaultMotorSpeed, harsh);
						}
					}
				}
				//good..
			}
		}
		stopMotors();
		leftSonar.disable();
		rightSonar.disable();
	}
	
	private float getGyroAngle(int pos){
		gyroAngler.fetchSample(gyroAngleResult, pos);
		return gyroAngleResult[pos];
	}
	
	private void writeScreenMSG(String msg, int x, int y){
		LCD.drawString(msg, x, y);
	}
	
	public static void main(String[] args) {
		MemoryDriverTest memoryDriver = new MemoryDriverTest();
		memoryDriver.drive();
	}

}
