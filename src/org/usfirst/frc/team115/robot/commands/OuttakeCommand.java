package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * An example command.  You can replace me with your own command.
 */
public class OuttakeCommand extends Command {
	
	public OuttakeCommand() {
		requires(Robot.intake);
	}

	protected void initialize() {
	}

	protected void execute() {
		Robot.intake.outtakeCube();
	}

	protected boolean isFinished() {
		return !(Robot.oi.outtakePressed());
	}

	protected void end() {
		Robot.intake.stop();
	}

	protected void interrupted() {
		end();
	}
}
