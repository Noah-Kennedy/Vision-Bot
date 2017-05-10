package org.usfirst.frc.team4786.robot.subsystems;

import org.usfirst.frc.team4786.robot.RobotMap;
import org.usfirst.frc.team4786.robot.commands.Drive;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The DriveTrainMecanum is a subsystem for a mecanum drive base. It uses the
 * RobotDrive wpilib class
 */
public class DriveTrain extends Subsystem implements PIDOutput {

	// instance fields

	// Drive base cantalons
	private CANTalon frontLeft = new CANTalon(RobotMap.frontLeftPort);
	private CANTalon frontRight = new CANTalon(RobotMap.frontRightPort);

	// NavX and PIDController objects
	private AHRS navX;
	private PIDController turnController;
	private double turnToAngleRate;

	public DriveTrain() {
		frontLeft.enable();
		frontRight.enable();
		frontLeft.setInverted(true);

		frontLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		frontRight.setFeedbackDevice(FeedbackDevice.QuadEncoder);

		// The encoders need to be reversed
		frontLeft.reverseSensor(true);
		frontRight.reverseSensor(true);

		// Beginning of the world of PID!!!!!!!!

		// Set Up the Encoder Revolutions!
		frontLeft.configEncoderCodesPerRev(RobotMap.DRIVETRAIN_ENCODER_CODES_PER_REV_LEFT);
		frontRight.configEncoderCodesPerRev(RobotMap.DRIVETRAIN_ENCODER_CODES_PER_REV_RIGHT);
		try {
			Thread.sleep(10);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		frontLeft.configEncoderCodesPerRev(RobotMap.DRIVETRAIN_ENCODER_CODES_PER_REV_LEFT);
		frontRight.configEncoderCodesPerRev(RobotMap.DRIVETRAIN_ENCODER_CODES_PER_REV_RIGHT);

		frontLeft.setPosition(0);
		frontRight.setPosition(0);

		frontLeft.setEncPosition(0);
		frontRight.setEncPosition(0);

		frontLeft.setAllowableClosedLoopErr(RobotMap.ERROR_CONSTANT_LEFT);
		frontRight.setAllowableClosedLoopErr(RobotMap.ERROR_CONSTANT_RIGHT);

		// Make sure the CANTalons are looking at the right stored PID values
		// with the Profile
		// Set our PID Values
		/*
		 * Set how fast of a rate the robot will accelerate Do not remove or you
		 * get a fabulous prize of a Flipping robot - CLOSED_LOOP_RAMP_RATE
		 */
		frontLeft.setPID(RobotMap.LeftP, RobotMap.LeftI, RobotMap.LeftD, RobotMap.LeftF, RobotMap.IZONE,
				RobotMap.CLOSED_LOOP_RAMP_RATE, RobotMap.DRIVEBASE_PROFILE);
		frontRight.setPID(RobotMap.RightP, RobotMap.RightI, RobotMap.RightD, RobotMap.RightF, RobotMap.IZONE,
				RobotMap.CLOSED_LOOP_RAMP_RATE, RobotMap.DRIVEBASE_PROFILE);

		// Initialize NavX and turnController objects
		navX = new AHRS(SPI.Port.kMXP);
		turnController = new PIDController(RobotMap.TurnP, RobotMap.TurnI, RobotMap.TurnD, RobotMap.TurnF, navX, this);
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(RobotMap.ALLOWABLE_TURN_ERROR);
		turnController.setContinuous(true);
		/*
		 * Add the PID Controller to the Test-mode dashboard, allowing manual
		 * tuning of the Turn Controller's P, I and D coefficients. Typically,
		 * only the P value needs to be modified.
		 */
		LiveWindow.addActuator("DriveTrain", "TurnController", turnController);


	}

	/**
	 * Drives the mecanum base through the robotDrive method
	 */
	public void drive(double left, double right) {
		frontLeft.set(left);
		frontRight.set(right);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new Drive());
	}

	public void driveToPositionInit(double distanceToDrive) {
		// Change Talon modes to "position" just in case
		// they were in another mode before
		frontLeft.changeControlMode(TalonControlMode.Position);
		frontRight.changeControlMode(TalonControlMode.Position);

		// Set Encoder Position to 0
		frontLeft.setEncPosition(0);
		frontRight.setEncPosition(0);
		try {
			Thread.sleep(10);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		frontLeft.setEncPosition(0);
		frontRight.setEncPosition(0);

		// Run convertToRotations function
		double rot = convertToRotations(distanceToDrive);

		// Make motors drive number of rotations
		// calculated before by convertToRotations()
		frontLeft.set(-rot);
		frontRight.set(rot);
		try {
			Thread.sleep(10);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		// Make sure we inverse this right side,
		// otherwise, you have a spinning robot on your hands
		frontLeft.set(-rot);
		frontRight.set(rot);
	}

	public boolean driveToPositionIsFinished() {
		return Math.abs(frontLeft.getError()) <= RobotMap.ERROR_CONSTANT_LEFT
				&& Math.abs(frontRight.getError()) <= RobotMap.ERROR_CONSTANT_RIGHT;
	}

	public void driveToPositionEnd() {
		frontLeft.setEncPosition(0);
		frontRight.setEncPosition(0);
	}

	public double getLeftEncoderPosition() {
		// Make sure graph isn't upside down (The stocks are going into the
		// toilet!!)
		return -frontLeft.getEncPosition();
	}

	public double getRightEncoderPosition() {
		return frontRight.getEncPosition();
	}

	public void turnToAngleInit(double targetAngle) {
		/*
		 * PIDController calculates a rate of motor output, so the CANTalons
		 * need to be in PercentVbus mode
		 */
		frontLeft.changeControlMode(TalonControlMode.PercentVbus);
		frontRight.changeControlMode(TalonControlMode.PercentVbus);

		// Initialize turnController and set the target
		navX.reset();
		turnController.enable();
		turnController.setSetpoint(targetAngle);
	}

	public void turnToAngleExecute() {
		// Set the CANTalons to the speed calculated by PIDController
		frontLeft.set(-turnToAngleRate);
		frontRight.set(turnToAngleRate);
	}

	// Another weird variable check for if turning should stop
	public boolean turnToAngleIsFinished() {
		return turnController.onTarget();
	}

	public void turnToAngleEnd() {
		turnController.disable();
	}

	// Take a distance in feet and convert to
	// rotations that CANTalons can take as input
	private double convertToRotations(double distanceInFeet) {
		return (distanceInFeet) / (Math.PI * (RobotMap.WHEEL_RADIUS * 2));
	}

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		turnToAngleRate = output;
	}
}
