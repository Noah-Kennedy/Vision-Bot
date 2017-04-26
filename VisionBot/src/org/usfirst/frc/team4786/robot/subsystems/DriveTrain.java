package org.usfirst.frc.team4786.robot.subsystems;

import org.usfirst.frc.team4786.robot.Robot;
import org.usfirst.frc.team4786.robot.RobotMap;
import org.usfirst.frc.team4786.robot.commands.Drive;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 *The DriveTrainMecanum is a subsystem for a mecanum drive base. It uses the RobotDrive wpilib class
 */
public class DriveTrain extends Subsystem implements PIDOutput{

	//instance fields
	
	//Drive base cantalons
    private CANTalon frontLeft = new CANTalon(RobotMap.frontLeftPort);
    private CANTalon frontRight = new CANTalon(RobotMap.frontRightPort);
    private double visionTurnRate;
    private PIDController visionController;
    
    public DriveTrain(){
    	frontLeft.enable();
		frontRight.enable();
		frontLeft.setInverted(true);
		
		visionController = new PIDController(RobotMap.visionP,RobotMap.visI,RobotMap.visD, Robot.vision, this);
		visionController.setInputRange(-(RobotMap.width - 1), RobotMap.width - 1);
		visionController.setOutputRange(-1, 1);
		visionController.setAbsoluteTolerance(RobotMap.allowableError);
		visionController.setContinuous(true);
		
		LiveWindow.addActuator("DriveTrain", "VisionController", visionController);

    }
    
    /**
     * Drives the mecanum base through the robotDrive method
     */    
    public void drive(double left, double right){
    	frontLeft.set(left);
    	frontRight.set(right);
    }
    
    public void initDefaultCommand() {
        setDefaultCommand(new Drive());
    }
    
    public void centerOnPegInit(){
    	/* PIDController calculates a rate of motor output,
		 * so the CANTalons need to be in PercentVbus mode */
		frontLeft.changeControlMode(TalonControlMode.PercentVbus);
		frontRight.changeControlMode(TalonControlMode.PercentVbus);
		//visionController.reset();
		visionController.enable();
		visionController.setSetpoint(0);
    }
    
    public void centerOnPegExecute(){
    	frontLeft.set(-visionTurnRate);
    	frontRight.set(visionTurnRate);
    }
    
    public void centerOnPegEnd(){
    	visionController.reset();
    }

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		visionTurnRate = output;
	}
}

