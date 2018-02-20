package org.usfirst.frc.team115.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;

public class ElevateToScale extends Command {

	private String configuration;

	public ElevateToScale(String configuration) {
		this.configuration = configuration;
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
	}
	
	public void execute() {
		if(Robot.elevator.getError() <= Robot.elevator.convertInchesToTicks(1.0)) {
			Robot.elevator.hold();
		}
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	public void end() {
		
	}

}
