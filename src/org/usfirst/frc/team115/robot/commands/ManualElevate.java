package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ManualElevate extends Command{

	public ManualElevate() {
		requires(Robot.elevator);
	}

	protected void initialize() {
		Robot.intake.extendIntake();		
	}

	public void execute() {
		Robot.elevator.manualElevate(Robot.oi.getManualElevate());
	}

	protected boolean isFinished() {
		return false;
	}

	public void end() {}

}
