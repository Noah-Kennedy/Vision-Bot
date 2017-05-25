package org.usfirst.frc.team4786.robot;

public class MotionLib {
	private double getS(double sr, double sl){
		return (sr + sl) / 2.0;
	}
	
	private double getTheta(double sr, double sl, double theta){
		return (sr - sl) / RobotMap.meshbotWidth + theta;
	}
	public double getCurrentOffsetX(double sr, double sl, double x0, double theta){
		return getS(sr, sl) * Math.cos(Math.toRadians(getTheta(sr,sl,theta))) + x0;
	}
	
	public double getCurrentOffsetY(double sr, double sl, double y0, double theta){
		return getS(sr, sl) * Math.sin(Math.toRadians(getTheta(sr,sl,theta))) + y0;
	}
}
