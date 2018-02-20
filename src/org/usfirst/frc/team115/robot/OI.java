package org.usfirst.frc.team115.robot;

import org.usfirst.frc.team115.robot.commands.ElevateToScale;
import org.usfirst.frc.team115.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team115.robot.commands.IntakeCommand;
import org.usfirst.frc.team115.robot.commands.ManualElevate;
import org.usfirst.frc.team115.robot.commands.OuttakeCommand;
import org.usfirst.frc.team115.robot.commands.WideIntakeCommand;
import org.usfirst.frc.team115.robot.commands.ZeroElevator;

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
	JoystickButton wideIntake;
	
	Joystick operatorPanel;
	JoystickButton defaultSwitchElevate;
	JoystickButton highSwitchElevate;
	JoystickButton lowScaleElevate;
	JoystickButton defaultScaleElevate;
	JoystickButton highScaleElevate;
	
	JoystickButton manualElevate;
	JoystickButton manualMode;
	JoystickButton zeroElevator;

	public OI() {
		driverJoystick = new Joystick(0);
		intake = new JoystickButton(driverJoystick, Constants.kIntake);	
		wideIntake = new JoystickButton(driverJoystick, 4);
		
		operatorPanel = new Joystick(1);
		
		outtake = new JoystickButton(operatorPanel, Constants.kOuttake); 
		
		defaultSwitchElevate = new JoystickButton(operatorPanel, Constants.kSwitch);
		
		lowScaleElevate = new JoystickButton(operatorPanel, 11);
		defaultScaleElevate = new JoystickButton(operatorPanel, 9);
		highScaleElevate = new JoystickButton(operatorPanel, 7);
		
		zeroElevator = new JoystickButton(operatorPanel, Constants.kZero);
		manualMode = new JoystickButton(operatorPanel, Constants.kManualMode);
		
		intake.whenPressed(new IntakeCommand());
		wideIntake.whenPressed(new WideIntakeCommand());
		outtake.whenPressed(new OuttakeCommand());
		
		defaultScaleElevate.whenPressed(new ElevateToScale("default"));
		highScaleElevate.whenPressed(new ElevateToScale("high"));
		lowScaleElevate.whenPressed(new ElevateToScale("low"));
		
		defaultSwitchElevate.whenPressed(new ElevateToSwitch());

		zeroElevator.whenPressed(new ZeroElevator());
		manualMode.whenPressed(new ManualElevate());
	}

	public boolean getManualMode() {
		return manualMode.get();
	}
	
	public double getManualElevate() {
		return operatorPanel.getRawAxis(Constants.kManualElevate);
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
	
	public boolean getHoldPosition() {
		return operatorPanel.getRawButton(2);
	}
	
	public boolean intakePressed() {
		return intake.get();
	}
	
	public boolean wideIntake() {
		return wideIntake.get();
	}
	
	public boolean outtakePressed() {
		return outtake.get();
	}
		
	public boolean manualElevatePressed() {
		return manualElevate.get();
	}

	public boolean getElevateToSwitch() {
		return defaultSwitchElevate.get();
	}

	public double getLeftIntake() {
		// TODO Auto-generated method stub
		return this.driverJoystick.getRawAxis(2);
	}
	
	public double getRightIntake() {
		// TODO Auto-generated method stub
		return this.driverJoystick.getRawAxis(3);
	}
	
	public boolean getShiftButton() {
		return driverJoystick.getRawButton(2);
	}
}
