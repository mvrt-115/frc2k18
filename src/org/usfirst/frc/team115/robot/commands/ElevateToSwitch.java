package org.usfirst.frc.team115.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;

public class ElevateToSwitch extends Command {

	private String configuration;

	public ElevateToSwitch(String configuration) {
		this.configuration = configuration;
		requires(Robot.elevator);
	}
	public void execute() {
		if (configuration == "high")
			Robot.elevator.setElevatorSetpoint(Constants.kHighSwitchHeight);
		else if (configuration == "default")
			Robot.elevator.setElevatorSetpoint(Constants.kDefaultSwitchHeight);
	}	

	@Override
	protected boolean isFinished() {
		return Robot.oi.manualElevatePressed();
	}

	public void end() {
		Robot.elevator.home();
	}
}
