package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class CheesyDriveJoystick extends Command {
	
	public CheesyDriveJoystick() {
		requires(Robot.drivetrain);
	}
	
	public void execute() {
		Robot.drivetrain.drive(Robot.oi.getThrottle(), Robot.oi.getWheel(), Robot.oi.getQuickTurn());
	}

	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return false;
	}
}
