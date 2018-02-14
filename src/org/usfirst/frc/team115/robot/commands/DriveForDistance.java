package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveForDistance extends Command {
	
	private double setpoint;
	
	public DriveForDistance(double setpoint) {
		this.setpoint = setpoint;
	}
	
	public void execute() {
		System.out.println("In DriveForDistance-->execute()");
		Robot.drivetrain.setDistanceSetpoint(setpoint);
	}
	
	public boolean isFinished() {
		if (Math.abs(Robot.drivetrain.getError() - setpoint) < 200.0) {
				return true;
		}
		return false;
	}

}
