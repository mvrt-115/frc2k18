package org.usfirst.frc.team115.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;

public class ElevateToSwitch extends Command {

	public ElevateToSwitch() {
		requires(Robot.elevator);
	}

	public void initialize() {
		Robot.intake.extendIntake();
		Robot.elevator.setElevatorSetpoint(Constants.kDefaultSwitchHeight);
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
