package org.usfirst.frc.team4786.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// camera settings
	public static final int cameraFPS = 15;
	public static final int cameraExposure = 1;
	public static final int width = 320;
	public static final int height = 240;
	public static final double cameraFOVWidthInDegrees = 43.84;
	public static final double cameraFOVHeightInDegrees = 24.66;

	// thresholds for pixel HSV
	public static final int lowHue = 55;
	public static final int lowSat = 230;
	public static final int lowVal = 0;
	public static final int highHue = 70;
	public static final int highSat = 255;
	public static final int highVal = 255;

	//geometries constants
	public static final double heightOfTargetInFeet = 5.0 / 12;
	public static final double MESHBOT_ROBOT_LENGTH = 2.34375; // In feet
	public static final double distanceOfCamFromFrontOfBot = 0;
	public static final double degreesPerPixelWidth = RobotMap.cameraFOVWidthInDegrees / RobotMap.width;
	public static final double degreesPerPixelHeight = RobotMap.cameraFOVHeightInDegrees / RobotMap.height;
	public static final double bottomHeight = 11.0 / 12;
	public static final double cameraAngle = 15; //TODO figure out what this really is
	public static final double cameraHeight = 7.0/12;
	public static final double distanceBetweenCentersOfTargets = 8.25 / 12;
	
	//constants for second stage filtering
	public static final double lowAspectRatio = .35;
	public static final double highAspectRatio = .5;
	public static final double contourToRectUpperPercentage = 1;
	public static final double contourToRectLowerPercentage = .6;


	// drive train ports
	public static final int frontRightPort = 14;
	public static final int frontLeftPort = 13;

	// PID for driveTo position
	// left side PID constants
	public static final double LeftP = 0.0001;
	public static final double LeftI = 0.000010;
	public static final double LeftD = 0.0;
	public static final double LeftF = 0.0;	
	// Right GearBox PID Constants
	public static final double RightP = 0.0001;
	public static final double RightI = 0.000010;
	public static final double RightD = 0.0;
	public static final double RightF = 0.0;

	// turn PID constants
	public static final double TurnP = 0.1;
	public static final double TurnI = 0.0;
	public static final double TurnD = 0.0;
	public static final double TurnF = 0.0;
	
	// General PID Constants
	public static final double WHEEL_RADIUS = 0.25;	// Wheel Radius measured in feet
	public static final int ERROR_CONSTANT_LEFT = 100; // In native units
	public static final int ERROR_CONSTANT_RIGHT = 100;
	public static final int ALLOWABLE_TURN_ERROR = 1; // In degrees
	public static final int DRIVETRAIN_ENCODER_CODES_PER_REV_LEFT = 360;
	public static final int DRIVETRAIN_ENCODER_CODES_PER_REV_RIGHT = 360;
	public static final double CLOSED_LOOP_RAMP_RATE = 0.015625;
	public static final int IZONE = 0;
	public static final int DRIVEBASE_PROFILE = 0;
	public static final double fudgeFactor = 1;
	public static final double turnFudgeFactor = 0.75;

}
