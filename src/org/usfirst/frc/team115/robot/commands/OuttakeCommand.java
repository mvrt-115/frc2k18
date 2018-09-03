package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * An example command.  You can replace me with your own command.
 */
public class OuttakeCommand extends Command {
	
	boolean drop = false;
	
	public OuttakeCommand() {
		requires(Robot.intake);
	}
	
	public OuttakeCommand(boolean drop) {
		this.drop = drop;
	}

	protected void initialize() {
	}

	protected void execute() {
		if(drop) {
			Robot.intake.dropCube();
		} else {
			Robot.intake.outtakeCube();
		}
	}

	protected boolean isFinished() {
		return !(Robot.oi.outtakePressed());
	}

	protected void end() {
		Robot.intake.stop();
		Robot.carriage.stop();
	}

	protected void interrupted() {
		end();
	}
}