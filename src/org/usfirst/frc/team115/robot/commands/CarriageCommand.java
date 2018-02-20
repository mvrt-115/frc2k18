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
		Robot.carriage.intakeCube(0.25); //values tbd
	}

	protected boolean isFinished() {
		return !(Robot.oi.intakePressed());
	}

	protected void end() {
		Robot.carriage.stop();
	}

	protected void interrupted() {
		end();
	}
}