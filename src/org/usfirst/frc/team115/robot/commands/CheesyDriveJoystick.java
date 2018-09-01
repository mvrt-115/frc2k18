package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class CheesyDriveJoystick extends Command {
	
	public CheesyDriveJoystick() {
		requires(Robot.drivetrain);
	}
	
	public void execute() {
//		if(Robot.oi.getShiftButton()) {
//			Robot.drivetrain.shift();
//		}
		Robot.drivetrain.drive(Robot.oi.getThrottle() * 0.90, -Robot.oi.getWheel() * 0.90, Robot.oi.getQuickTurn());
	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
