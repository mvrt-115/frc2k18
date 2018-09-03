package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * An example command.  You can replace me with your own command.
 */
public class IntakeCommand extends Command {

	
	boolean isAuton = false;
	
	public IntakeCommand(boolean isAuton) {
		requires(Robot.intake);
		this.isAuton = isAuton;
	}
	
	public IntakeCommand() {
		requires(Robot.intake);
		isAuton = false;
	}

	protected void initialize() {
		Robot.intake.intakeDown();
	}

	protected void execute() {
		Robot.intake.intakeCube(false);
	}

	protected boolean isFinished() {
		if(isAuton)
			return false;//Robot.carriage.cubeDetected();
		else
			return (!(Robot.oi.intakePressed())) || Robot.carriage.cubeDetected();
	}

	protected void end() {
		if(Robot.carriage.cubeDetected()) {
			Robot.intake.stop();
			for(int i=0; i<200;i ++) {
				Robot.oi.rumbleJoystick();
			}
			Robot.oi.stopRumble();
		} else {
			Robot.intake.stop();
		}

	}

	protected void interrupted() {
		end();
	}
}