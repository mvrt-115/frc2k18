package org.usfirst.frc.team115.robot;

import org.usfirst.frc.team115.robot.Constants;

import org.usfirst.frc.team115.robot.commands.IntakeCommand;
import org.usfirst.frc.team115.robot.commands.OuttakeCommand;
import org.usfirst.frc.team115.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team115.robot.commands.ElevateToScale;
import org.usfirst.frc.team115.robot.commands.ZeroElevator;
import org.usfirst.frc.team115.robot.commands.ManualElevate;

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
	JoystickButton defaultSwitchElevate;
	JoystickButton highSwitchElevate;
	JoystickButton lowScaleElevate;
	JoystickButton defaultScaleElevate;
	JoystickButton highScaleElevate;
	
	JoystickButton manualElevate;
	JoystickButton zeroElevator;

	public OI() {
		driverJoystick = new Joystick(0);
		intake = new JoystickButton(driverJoystick, Constants.kIntake);	
		outtake = new JoystickButton(driverJoystick, Constants.kOuttake); 
		
		operatorPanel = new Joystick(1);
		
		defaultSwitchElevate = new JoystickButton(operatorPanel, 3);
		highSwitchElevate = new JoystickButton(operatorPanel, 4);
		
		lowScaleElevate = new JoystickButton(operatorPanel, 8);
		defaultScaleElevate = new JoystickButton(operatorPanel, 6);
		highScaleElevate = new JoystickButton(operatorPanel, 7);
		
		manualElevate = new JoystickButton(operatorPanel, 5);
		zeroElevator = new JoystickButton(operatorPanel, 9);
		
		intake.whenPressed(new IntakeCommand());
		outtake.whenPressed(new OuttakeCommand());
		
		defaultSwitchElevate.whenPressed(new ElevateToSwitch("default"));
		highSwitchElevate.whenPressed(new ElevateToSwitch("high"));
		lowScaleElevate.whenPressed(new ElevateToScale("low"));
		
		defaultScaleElevate.whenPressed(new ElevateToScale("default"));
		highSwitchElevate.whenPressed(new ElevateToScale("high"));

		// manualElevate.whenPressed(new ManualElevate());
		zeroElevator.whenPressed(new ZeroElevator());
	}

	public double getManualElevate() {
		return operatorPanel.getRawAxis(Constants.kManualElevate);
//		return operatorPanel.getY();
	} 
	
	public double getThrottle() {
		return driverJoystick.getRawAxis(Constants.kThrottle);
	}
	
	public double getWheel() {
		return driverJoystick.getRawAxis(Constants.kWheel);
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
		
	public boolean manualElevatePressed() {
		return manualElevate.get();
	}
}
