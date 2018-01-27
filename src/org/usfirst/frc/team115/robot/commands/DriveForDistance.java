package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class DriveForDistance extends InstantCommand {
	
	private double setpoint;
	
	public DriveForDistance(double setpoint) {
		this.setpoint = setpoint;
	}
	
	public void execute() {
		Robot.drivetrain.setDistanceSetpoint(setpoint);
	}

}
