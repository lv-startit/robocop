package main.java;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class RobocopTestDrive {
	private static boolean exit;

	public static void main(String[] args) {
		Thread t1 = new Thread(new ExitListener());
		t1.start();
		
		Port port = LocalEV3.get().getPort("S2");

		// Get an instance of the Ultrasonic EV3 sensor
		SensorModes sensor = new EV3UltrasonicSensor(port);

		// get an instance of this sensor in measurement mode
		SampleProvider distance= sensor.getMode("Distance");

		// initialize an array of floats for fetching samples. 
		// Ask the SampleProvider how long the array should be
		float[] sample = new float[distance.sampleSize()];
		String outputString ="";
		// fetch a sample
		
		while(!exit) {
		  distance.fetchSample(sample, 0);
		  outputString = String.format("%.2f", sample[0]);
		  System.out.print(outputString);
		  Delay.msDelay(250);
		}
		System.exit(0);
	}
	
	private static class ExitListener implements Runnable{
		@Override
		public void run() {
			Button.waitForAnyPress();
			exit = true;
		}
		
	}
}
