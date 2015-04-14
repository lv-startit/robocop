package main.java;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

public class BasicDriver {
	private static boolean exit;
	private static RegulatedMotor motor1 = Motor.A;
	private static RegulatedMotor motor2 = Motor.B;
	
	public static void main(String[] args) {
		Thread t1 = new Thread(new ExitListener());
		t1.start();
		
		initialize();
		Port port = LocalEV3.get().getPort("S2");
		SensorModes sensor = new EV3UltrasonicSensor(port);
		SampleProvider distance= sensor.getMode("Distance");
		float[] sample = new float[distance.sampleSize()];
		
		while(!exit){
			  distance.fetchSample(sample, 0);
			  motor1.forward();
			  motor2.forward();
			  
			  if(sample[0] < (float)0.50){
				  while(sample[0] < (float)1.00){
					  distance.fetchSample(sample, 0);
					  turnLeft();
				  }
				  motor1.setSpeed(500);
			  }
		}
	}
	
	
	private static void initialize(){
		motor1.setSpeed(500);
		motor2.setSpeed(500);
	}
	
	private static void turnLeft(){
		motor1.setSpeed(0);
		motor2.setSpeed(500);
		motor1.forward();
		motor2.forward();
	}
	
	private static class ExitListener implements Runnable{
		@Override
		public void run() {
			Button.waitForAnyPress();
			exit = true;
		}
	}
	
}
