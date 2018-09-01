package org.usfirst.frc.team115.robot;

import org.usfirst.frc.team115.robot.commands.ElevateToScale;
import org.usfirst.frc.team115.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team115.robot.commands.IntakeCommand;
import org.usfirst.frc.team115.robot.commands.LiftIntake;
import org.usfirst.frc.team115.robot.commands.ManualElevate;
import org.usfirst.frc.team115.robot.commands.OuttakeCommand;
import org.usfirst.frc.team115.robot.commands.WideIntakeCommand;
import org.usfirst.frc.team115.robot.commands.ZeroElevator;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.TimedCommand;


/**
 * This class is the glue that binds the controls on the physical operatorPanel
 * interface to the commands and command groups that allow control of the robot.
 */

public class OI {
	Joystick driverJoystick;
	JoystickButton intake;
	JoystickButton outtake;
	JoystickButton drop;
	JoystickButton carriage;
	JoystickButton wideIntake;
	JoystickButton raiseIntake;
	
	Joystick operatorPanel;
	JoystickButton defaultSwitchElevate;
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
		raiseIntake = new JoystickButton(driverJoystick, 1);
		
		operatorPanel = new Joystick(1);
		outtake = new JoystickButton(operatorPanel, Constants.kOuttake) { 
			public boolean get() {
				return operatorPanel.getRawAxis(Constants.kOuttake) >= 0.3;
			}
		};
		drop = new JoystickButton(operatorPanel, 6);
		drop.whenPressed(new TimedCommand(0.75) {
			public void execute() {
				Robot.intake.softDropCube();
			}
			
			public void end() {
				Robot.intake.stop();
			}
		});
		
		
		defaultSwitchElevate = new JoystickButton(operatorPanel, Constants.kSwitch);
		
		lowScaleElevate = new JoystickButton(operatorPanel, 1);
		defaultScaleElevate = new JoystickButton(operatorPanel, 3);
		highScaleElevate = new JoystickButton(operatorPanel, 4);
		
		zeroElevator = new JoystickButton(operatorPanel, Constants.kZero) {
			public boolean get() {
				return operatorPanel.getRawAxis(Constants.kZero) >= 0.3;
			}
		};
		manualMode = new JoystickButton(operatorPanel, Constants.kManualMode);
		
		intake.whenPressed(new IntakeCommand());
		wideIntake.whenPressed(new WideIntakeCommand());
		outtake.whenPressed(new OuttakeCommand());
		
		defaultScaleElevate.whenPressed(new ElevateToScale("default", false));
		highScaleElevate.whenPressed(new ElevateToScale("high", false));
		lowScaleElevate.whenPressed(new ElevateToScale("low", false));
		
		defaultSwitchElevate.whenPressed(new ElevateToSwitch(false));
		manualMode.whenPressed(new ManualElevate());
		zeroElevator.whenPressed(new ZeroElevator());
		raiseIntake.whenPressed(new LiftIntake());
	}

	public void rumbleJoystick() {
		operatorPanel.setRumble(RumbleType.kLeftRumble, 0.5);
		operatorPanel.setRumble(RumbleType.kRightRumble, 0.5);
	}
	
	public void stopRumble() {
		operatorPanel.setRumble(RumbleType.kLeftRumble, 0);
		operatorPanel.setRumble(RumbleType.kRightRumble, 0);
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
		return operatorPanel.getRawAxis(2) >= 0.3;
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
		return this.driverJoystick.getRawAxis(2);
	}
	
	public double getRightIntake() {
		return this.driverJoystick.getRawAxis(3);
	}
	
	public boolean getShiftButton() {
		return driverJoystick.getRawButton(2);
	}
}
