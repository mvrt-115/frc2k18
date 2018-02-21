package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class IntakeDown extends InstantCommand {

	public IntakeDown() {
		requires(Robot.intake);
	}

	protected void execute() {
		Robot.intake.intakeDown();
	}
	
}
