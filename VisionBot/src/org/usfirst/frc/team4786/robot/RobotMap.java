package org.usfirst.frc.team4786.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	//drive train ports
	public static final int frontRightPort = 14;
	public static final int frontLeftPort = 13;
	
	//vision constants for image size
	public static final int minimumArea = 0;
	public static final double contourToRectPercentage = 0;
	
	//thresholds for pixel RGB
	public static final int lowBlueValue = 250;
	public static final int lowRedValue = 250;
	public static final int lowGreenValue = 250;
	public static final int highBlueValue = 255;
	public static final int highGreenValue = 255;
	public static final int highRedValue = 255;
	
	//thresholds for pixel HSV
	public static final int lowHue = 55;
	public static final int lowSat = 225;
	public static final int lowVal = 0;
	public static final int highHue = 65;
	public static final int highSat = 255;
	public static final int highVal = 255;
	
	//camera stream size
	public static final int width = 320;
	public static final int height = 240;
	
	//vision PID
	public static final double visionP = .05;
	public static final double visI = 0;
	public static final double visD = 0;
	public static final double allowableError = 10;
}
