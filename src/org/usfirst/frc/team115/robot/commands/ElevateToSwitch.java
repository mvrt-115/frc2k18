package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.auton.DriveTimedAutoLine;

import edu.wpi.first.wpilibj.command.Command;

public class ElevateToSwitch extends Command {

	public ElevateToSwitch() {
		requires(Robot.elevator);
	}

	public void initialize() {
		Robot.elevator.setElevatorSetpoint(Constants.kDefaultSwitchHeight);
	}

	public void execute() {
		if(Robot.elevator.getError() <= 455.0) {
			Robot.elevator.hold();
			Robot.carriage.outtakeCube(1.0);
		} 
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	public void end() {
	}
}
