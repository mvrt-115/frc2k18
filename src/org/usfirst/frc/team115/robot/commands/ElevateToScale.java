package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ElevateToScale extends Command{

	private double setpoint;

	public ElevateToScale() {
		requires(Robot.elevator);
	}
	public void execute() {
		Robot.elevator.setElevatorSetpoint(setpoint);
	}

	protected boolean isFinished() {
		return Robot.oi.manualElevatePressed();
	}

	public void end() {
		Robot.elevator.home();
	}

}