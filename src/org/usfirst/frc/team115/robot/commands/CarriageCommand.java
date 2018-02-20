package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class CarriageCommand extends Command {

	public CarriageCommand() {
		requires(Robot.carriage);
	}

	protected void initialize() {
		
	}

	protected void execute() {
		if (Robot.oi.intakePressed())
			Robot.carriage.intakeCube(1); //values tbd
		if (Robot.oi.outtakePressed())
			Robot.carriage.outtakeCube(-1);
	}

	protected boolean isFinished() {
		return (!Robot.oi.intakePressed() || !Robot.oi.outtakePressed());
	}

	protected void end() {
		Robot.carriage.stop();
	}

	protected void interrupted() {
		end();
	}
}