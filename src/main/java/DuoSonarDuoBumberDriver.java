package main.java;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class DuoSonarDuoBumberDriver {
	/**
	 * LEFT-SONAR = PORT "4"
	 * RIGHT-SONAR = PORT "1"
	 * LEFT-BUMPER = PORT "2"
	 * RIGHT-BUMPER = PORT "3"
	 * LEFT-MOTOR = PORT "A"
	 * RIGHT-MOTOR = PORT "D"
	 * */
	private boolean stopped;
	private int defaultMotorSpeed, backpedalDistance;
	private float safeWallDistance;
	private boolean trigerredLeftBumper, trigerredRightBumper;
	private int countdown;
	private float leftWallDistance, rightWallDistance;
	
	//Implementation stuff,  \/ sensors; ^ listeners and properties
	
	private LeftBumperListener leftBumper;
	private RightBumperListener rightBumper;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private LeftSideSonar leftSonar;
	private RightSideSonar rightSonar;
	
	
	public DuoSonarDuoBumberDriver() {
		leftBumper = new LeftBumperListener();
		rightBumper = new RightBumperListener();
	}
	
	private void initialize(){
		leftMotor = new EV3LargeRegulatedMotor(MotorPort.A);
		rightMotor = new EV3LargeRegulatedMotor(MotorPort.D);
		
		leftSonar = new LeftSideSonar();
		rightSonar = new RightSideSonar();
		
		defaultMotorSpeed = 400;
		countdown = 5000;
		backpedalDistance = 1000;
		safeWallDistance = 0.2f;
		
		leftMotor.setSpeed(defaultMotorSpeed);
		rightMotor.setSpeed(defaultMotorSpeed);
	}
	
	//MAIN METHOD OF THIS OBJECT
	private void drive(){
		initialize();
		
		//Get'cha motor running!! yeeehaaa!
		LCD.drawString("!Press 'Enter'!", 0, 0);
		Button.ENTER.waitForPressAndRelease();
		//It's the final countdooooown !!
		Delay.msDelay(countdown);
		LCD.drawString("Start in 5sec!", 0, 2);
		LCD.drawString("GO! GO! GO!", 0, 4);
		
		while(!stopped){
			if(Button.ESCAPE.isDown()){
				stopped = true;
			}
			leftMotor.setSpeed(defaultMotorSpeed);
			rightMotor.setSpeed(defaultMotorSpeed);
			moveForward();
			//"wall-free" check
			if(leftBumper.getState() != 1.0 && rightBumper.getState() != 1.0){
				//no wall => fetch distances
				leftWallDistance = leftSonar.getDistance();
				rightWallDistance = rightSonar.getDistance();
				
				//do you have plenty of room to drive? 
				//(left-side)
				if(leftWallDistance > safeWallDistance){
					//(right-side)
					if(rightWallDistance > safeWallDistance){
						//YUP!!
						while((leftWallDistance > safeWallDistance && rightWallDistance > safeWallDistance) && 
								(leftBumper.getState() != 1.0 && rightBumper.getState() != 1.0)){
								moveForward();
								leftWallDistance = leftSonar.getDistance();
								rightWallDistance = rightSonar.getDistance();
						}
					}else{
						//(right-side) is close! => turn left!!
						while((leftWallDistance > safeWallDistance && rightWallDistance < safeWallDistance) && 
								(leftBumper.getState() != 1.0 && rightBumper.getState() != 1.0)){
							turnLeft();
							moveForward();
							leftWallDistance = leftSonar.getDistance();
							rightWallDistance = rightSonar.getDistance();
						}
					}
				}else{
					//that (left-side) is getting on you!! => turn right!!
					while((leftWallDistance < safeWallDistance && rightWallDistance > safeWallDistance) && 
							(leftBumper.getState() != 1.0 && rightBumper.getState() != 1.0)){
						turnRight();
						moveForward();
						leftWallDistance = leftSonar.getDistance();
						rightWallDistance = rightSonar.getDistance();
					}
				}
			}else{
				//you're literally IN a wall!
				//check your bumpers...
				if(rightBumper.getState() == 1.0){
					trigerredRightBumper = true;
				}else if(leftBumper.getState() == 1.0){
					trigerredLeftBumper = true;
				}
				
				//drive some distance backwards... slow and steady..
				moveBackward(defaultMotorSpeed);
				Delay.msDelay(backpedalDistance);
				
				//Now DEAL WITH IT!! GO!! GO!! GO!!!
				if(trigerredLeftBumper){
					//turn RIGHT
					turnRIGHT();
				}else if(trigerredRightBumper){
					//turn LEFT
					turnLEFT();
				}
				trigerredRightBumper = false;
				trigerredLeftBumper = false;
				moveForward();
				Delay.msDelay(backpedalDistance);
			}
		}
	}
	
	
	private void moveForward(){
		leftMotor.forward();
		rightMotor.forward();
	}
	
	private void moveBackward(int timeMilis){
		leftMotor.backward();
		rightMotor.backward();
	}
	
	private void turnLeft(){
		leftMotor.setSpeed(defaultMotorSpeed/2);
		rightMotor.setSpeed(defaultMotorSpeed);
	}
	
	private void turnLEFT(){
		leftMotor.setSpeed(defaultMotorSpeed/8);
		rightMotor.setSpeed(defaultMotorSpeed*2);
	}
	
	
	private void turnRight(){
		leftMotor.setSpeed(defaultMotorSpeed);
		rightMotor.setSpeed(defaultMotorSpeed/2);
	}
	
	private void turnRIGHT(){
		leftMotor.setSpeed(defaultMotorSpeed*2);
		rightMotor.setSpeed(defaultMotorSpeed/8);
	}
	
	private class RightSideSonar {
		SensorModes soundSensor;
		SampleProvider scanDistance;
		float[] distance;
		
		public RightSideSonar() {
			soundSensor = new EV3UltrasonicSensor(SensorPort.S1);
			scanDistance = soundSensor.getMode("Distance");
			distance = new float[scanDistance.sampleSize()];
		}
		
		public float getDistance(){
			scanDistance.fetchSample(distance, 0);
			return distance[0];
		}
	}
	
	private class LeftSideSonar {
		SensorModes soundSensor;
		SampleProvider scanDistance;
		float[] distance;
		
		public LeftSideSonar() {
			soundSensor = new EV3UltrasonicSensor(SensorPort.S4);
			scanDistance = soundSensor.getMode("Distance");
			distance = new float[scanDistance.sampleSize()];
		}
		
		public float getDistance(){
			scanDistance.fetchSample(distance, 0);
			return distance[0];
		}
	}
	
	private class RightBumperListener {
		EV3TouchSensor buttonSensor;
		SampleProvider buttonState;
		float[] state;
		
		public RightBumperListener() {
			buttonSensor = new EV3TouchSensor(SensorPort.S3);
			buttonState = buttonSensor.getTouchMode();
			state = new float[buttonSensor.sampleSize()];
		}
		public float getState(){
			buttonState.fetchSample(state, 0);
			return state[0];
		}
	}
	
	private class LeftBumperListener{
		EV3TouchSensor buttonSensor;
		SampleProvider buttonState;
		float[] state;
		
		public LeftBumperListener() {
			buttonSensor = new EV3TouchSensor(SensorPort.S2);
			buttonState = buttonSensor.getTouchMode();
			state = new float[buttonSensor.sampleSize()];
		}
		public float getState(){
			buttonState.fetchSample(state, 0);
			return state[0];
		}
	}
	
	public static void main(String[] args) {
		DuoSonarDuoBumberDriver duoSonarDuoBumberDriver = new DuoSonarDuoBumberDriver();
		duoSonarDuoBumberDriver.drive();
	}

}
