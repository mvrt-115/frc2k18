package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * An example command.  You can replace me with your own command.
 */
public class IntakeCommand extends Command {
	
	public IntakeCommand() {
		requires(Robot.intake);
	}

	protected void initialize() {
	}

	protected void execute() {
		Robot.intake.intakeCube();
	}

	protected boolean isFinished() {
		return !(Robot.oi.intakePressed());
	}

	protected void end() {
		Robot.intake.stop();
	}

	protected void interrupted() {
		end();
	}
}
