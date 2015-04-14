package main.java;

import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.*;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.port.SensorPort;


public class ButtonTest {
	
	private static EV3TouchSensor first;
	private static EV3TouchSensor second;
	static RegulatedMotor motor1 = Motor.B;
	static RegulatedMotor motor2 = Motor.C;
	
	private static float[]sample;
	
	
	public static void main(String args[]){
		Sound.beep();
		LCD.drawString("Hello my robot", 0, 5);
		
	    
		first = new EV3TouchSensor(SensorPort.S1);
		second = new EV3TouchSensor(SensorPort.S2);
		
		motor1.setSpeed(789);
		motor2.setSpeed(789);
		sample = new float[10];
		
		while(!Button.ESCAPE.isDown()){
			first.fetchSample(sample, 0);
			float touchValue = sample[0];
			second.fetchSample(sample, 0);
			float second_touchValue = sample[0];
			
			LCD.drawString("Touch:" + touchValue + "t" + first.sampleSize(),0,3);
			LCD.drawString("Touch:" + second_touchValue + "t" + second.sampleSize(),0,3);
			
			if(sample[0] != 1){
				motor1.forward();
				motor2.forward();
			}
			
			else {
				motor1.stop();
				motor2.stop();
			}
			
			Delay.msDelay(100);
		}
	}
	 
}
