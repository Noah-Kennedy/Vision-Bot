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
	public static final double contourToRectPercentage = .5;
	
	//camera settings
	public static final int cameraFPS = 15;
	public static final int cameraExposure = 1;
	
	//camera stream size
	public static final int width = 320;
	public static final int height = 240;
	
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
	
	//vision PID
	public static final double visionP = .05;
	public static final double visI = 0;
	public static final double visD = 0;
	public static final double allowableError = 10;
	
	public static final double MESHBOT_ROBOT_LENGTH = 2.34375; //In feet
	
	//General PID Constants
	public static final int ERROR_CONSTANT_LEFT = 100; //In native units
	public static final int ERROR_CONSTANT_RIGHT = 100;
	public static final int ALLOWABLE_TURN_ERROR = 1; //In degrees
	public static final int DRIVETRAIN_ENCODER_CODES_PER_REV_LEFT = 360;
	public static final int DRIVETRAIN_ENCODER_CODES_PER_REV_RIGHT = 360;
	public static final double CLOSED_LOOP_RAMP_RATE = 0.015625;
	public static final int IZONE = 0;
	public static final int DRIVEBASE_PROFILE = 0;
	public static final double MAXIMUM_SPEED_VELOCITY_PID = 0.6;
	public static final double fudgeFactor = 1;
	public static final double turnFudgeFactor = 0.75;
	
	//Wheel Radius measured in feet
	public static final double WHEEL_RADIUS = 0.25;
	
	//Distance between wheels measured in feet
	public static final double WHEEL_SEPARATION = 2;

	public static final double LeftP = 0.0001;
	public static final double LeftI = 0.000010;
	public static final double LeftD = 0.0;
	public static final double LeftF = 0.0;
	//Right GearBox PID Constants
	public static final double RightP = 0.0001;
	public static final double RightI = 0.000010;
	public static final double RightD = 0.0;
	public static final double RightF = 0.0;
	
	public static final double TurnP = 0.02;
	public static final double TurnI = 0.0;
	public static final double TurnD = 0.0;
	public static final double TurnF = 0.0;
}
