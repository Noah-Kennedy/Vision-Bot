package org.usfirst.frc.team4786.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public XboxController xbox;
	Button aButton;
	
	public OI(){
		xbox = new XboxController(0);
		aButton = new JoystickButton(xbox, Buttons.A);
		//aButton.whenPressed(new CenterOnPeg());
	}
	
}
