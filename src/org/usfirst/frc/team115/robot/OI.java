package org.usfirst.frc.team115.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	Joystick joystick;
	JoystickButton intake;
	JoystickButton quickTurn;
	JoystickButton outtake;
	
	public OI() {
		joystick = new Joystick(0);
		intake = new JoystickButton(joystick, 0);	//check placement
		quickTurn = new JoystickButton(joystick, 1); //check this
		outtake = new JoystickButton(joystick, 2); //check placement
	}
	
	public Joystick getJoystick() {
		return joystick;
	}
	public boolean intakePressed() {
		return intake.get();
	}
	
	public boolean outtakePressed() {
		return outtake.get();
	}

	public double getWheel() {
		// TODO Auto-generated method stub
		return joystick.getX();
	}
	
	public boolean getQuickTurn() {
		return quickTurn.get();//check
	}

}

