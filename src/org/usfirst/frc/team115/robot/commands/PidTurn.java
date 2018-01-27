package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class PidTurn extends Command{
	
	private double setpoint;
	private int tolerance;
	
	public PidTurn(double setpoint) {
		this.setpoint = setpoint;
	}

	protected boolean isFinished() {
		tolerance = 1;
		return Math.abs(setpoint - Robot.drivetrain.PIDController.getError()) < tolerance;
	}
	
	public void end() {
		Robot.drivetrain.PIDController.disable();
	}
}
