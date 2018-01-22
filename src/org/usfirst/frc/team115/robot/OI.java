package org.usfirst.frc.team115.robot;


import org.usfirst.frc.team115.robot.commands.IntakeCommand;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;


/**
 * This class is the glue that binds the controls on the physical operatorPanel
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	Joystick driverJoystick;
	JoystickButton intake;
	JoystickButton outtake;
	JoystickButton carriage;
	
	Joystick operatorPanel;
	JoystickButton scaleElevate;
	JoystickButton switchElevate;
	JoystickButton manualElevate;
	JoystickButton lowerElevator;
	
	public OI() {
		driverJoystick = new Joystick(0);
		intake = new JoystickButton(driverJoystick, 0);	
		outtake = new JoystickButton(driverJoystick, 1); 
		
		operatorPanel = new Joystick(1);
		scaleElevate = new JoystickButton(operatorPanel, 0);
		switchElevate = new JoystickButton(operatorPanel, 1);
		manualElevate = new JoystickButton(operatorPanel, 2);
		lowerElevator = new JoystickButton(operatorPanel, 3);
		
		intake.whenPressed(new IntakeCommand());
		//need to connect buttons to rest of commands
	}
	
	public double getThrottle() {
		return driverJoystick.getRawAxis(0);
	}
	
	public double getWheel() {
		return driverJoystick.getRawAxis(5);
	}
	
	public boolean getQuickTurn() {
		return driverJoystick.getRawButton(Constants.kQuickTurn);
	}
	
	public boolean intakePressed() {
		return intake.get();
	}
	
	public boolean outtakePressed() {
		return outtake.get();
	}
	
	public boolean scaleElevatePressed() {
		return scaleElevate.get();
	}
	
	public boolean switchElevatePressed() {
		return switchElevate.get();
	}
	
	public boolean manualElevatePressed() {
		return manualElevate.get();
	}
}

