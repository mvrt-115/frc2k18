package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ElevateToScale extends InstantCommand {

	private String configuration;
	boolean isAuton;

	public ElevateToScale(String configuration, boolean isAuton) {
		this.configuration = configuration;
		this.isAuton = isAuton;
		requires(Robot.elevator);
	}

	public void initialize() {
		Robot.intake.extendIntake();
		if (configuration == "high")
			Robot.elevator.setElevatorSetpoint(Constants.kHighScaleHeight); //placing from higher than regular height (eg cubes already stacked on first layer)
		else if (configuration == "default")
			Robot.elevator.setElevatorSetpoint(Constants.kDefaultScaleHeight); //regular height
		else if (configuration == "low")
			Robot.elevator.setElevatorSetpoint(Constants.kLowScaleHeight); //placing at lower height (eg if scale in possession)
		Robot.elevator.enable(true);
	}
	
	public void execute() {
//		if(Robot.elevator.getError() <= UnitConverter.convertInchesToTicks(1.0)) {
//			Robot.elevator.hold();
//			if(isAuton) {
//				Robot.intake.outtakeCube();
//			}
//		}
	}

//	@Override
//	protected boolean isFinished() {
//		return false;
//	}

	public void end() {
//		Robot.elevator.zero();
//		if(isAuton) {
//			Robot.intake.stop();
//		}
	}

}
