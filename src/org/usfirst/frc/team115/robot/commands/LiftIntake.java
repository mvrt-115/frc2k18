package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class LiftIntake extends InstantCommand {

	public LiftIntake() {
		requires(Robot.intake);
	}

	protected void execute() {
		Robot.intake.stowIntake();
	}
	
}
