package org.usfirst.frc.team4786.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static final int frontRightPort = 0;
	public static final int frontLeftPort = 0;
	
	//vision constants for image
	public static final int minimumArea = 0;
	public static final int lowBlueValue = 250;
	public static final int lowRedValue = 250;
	public static final int lowGreenValue = 250;
	public static final int highBlueValue = 255;
	public static final int highGreenValue = 255;
	public static final int highRedValue = 255;
	public static final int width = 320;
	public static final int height = 240;
	
	//vision PID
	public static final double visionP = .05;
	public static final double visI = 0;
	public static final double visD = 0;
	public static final double allowableError = 10;
}
