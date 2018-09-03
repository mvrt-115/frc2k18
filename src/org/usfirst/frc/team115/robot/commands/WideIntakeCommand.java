package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class WideIntakeCommand extends Command {

	public WideIntakeCommand() {
		requires(Robot.intake);
	}

	protected void initialize() {
		Robot.intake.intakeDown();
	}

	protected void execute() {
		Robot.intake.intakeCube(true);
	}

	protected boolean isFinished() {
		return !(Robot.oi.wideIntake());
	}

	protected void end() {
		Robot.intake.stop();
	}

	protected void interrupted() {
		end();
	}
	
}
