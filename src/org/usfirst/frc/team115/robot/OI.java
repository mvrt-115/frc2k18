package org.usfirst.frc.team115.robot;


import org.usfirst.frc.team115.robot.commands.HighScaleElevate;
import org.usfirst.frc.team115.robot.commands.HighSwitchElevate;
import org.usfirst.frc.team115.robot.commands.IntakeCommand;
import org.usfirst.frc.team115.robot.commands.LowScaleElevate;
import org.usfirst.frc.team115.robot.commands.LowSwitchElevate;
import org.usfirst.frc.team115.robot.commands.MiddleScaleElevate;
import org.usfirst.frc.team115.robot.commands.OuttakeCommand;

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
	JoystickButton lowSwitchElevate;
	JoystickButton highSwitchElevate;
	JoystickButton lowScaleElevate;
	JoystickButton middleScaleElevate;
	JoystickButton highScaleElevate;
	
	JoystickButton manualElevate;
	JoystickButton lowerElevator;

	public OI() {
		driverJoystick = new Joystick(0);
		intake = new JoystickButton(driverJoystick, 0);	
		outtake = new JoystickButton(driverJoystick, 1); 
		
		operatorPanel = new Joystick(1);
		lowSwitchElevate = new JoystickButton(operatorPanel, 0);
		highSwitchElevate = new JoystickButton(operatorPanel, 1);
		lowScaleElevate = new JoystickButton(operatorPanel, 2);
		middleScaleElevate = new JoystickButton(operatorPanel, 3);
		highScaleElevate = new JoystickButton(operatorPanel, 4);
		manualElevate = new JoystickButton(operatorPanel, 5);
		lowerElevator = new JoystickButton(operatorPanel, 6);
		
		intake.whenPressed(new IntakeCommand());
		outtake.whenPressed(new OuttakeCommand());
		lowSwitchElevate.whenPressed(new LowSwitchElevate());
		highSwitchElevate.whenPressed(new HighSwitchElevate());
		lowScaleElevate.whenPressed(new LowScaleElevate());
		middleScaleElevate.whenPressed(new MiddleScaleElevate());
		highSwitchElevate.whenPressed(new HighScaleElevate());

		
		
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
	
//	public boolean lowScaleElevatePressed() {
//		return lowScaleElevate.get();
//	}
//	
//	public boolean middleScaleElevatePressed() {
//		return middleScaleElevate.get();
//	}
//	
//	public boolean highScaleElevatePressed() {
//		return highScaleElevate.get();
//	}
//	
//	public boolean lowSwitchElevatePressed() {
//		return lowSwitchElevate.get();
//	}
//	
//	public boolean highSwitchElevatePressed() {
//		return highScaleElevate.get();
//	}
	
	public boolean manualElevatePressed() {
		return manualElevate.get();
	}
}

