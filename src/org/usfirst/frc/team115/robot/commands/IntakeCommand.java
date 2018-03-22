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
		Robot.intake.intakeDown();
	}

	protected void execute() {
		Robot.intake.intakeCube(false);
	}

	protected boolean isFinished() {
		return (!(Robot.oi.intakePressed())) || Robot.carriage.cubeDetected();
	}

	protected void end() {
		if(Robot.carriage.cubeDetected()) {
			Robot.oi.rumbleJoystick();
			Robot.intake.stallIntake();
		} else {
			Robot.intake.stop();
		}
		Robot.oi.stopRumble();
	}

	protected void interrupted() {
		end();
	}
}