package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PidTurn extends Command {
	
	private double setpoint;
	private double tolerance = 0.3;
//	private double tolerance = 0.0;
	
	public PidTurn(double setpoint) {
		this.setpoint = setpoint;
	}
	
	public void initialize() {
		Robot.drivetrain.resetPidState();
		Robot.drivetrain.setTurnSetpoint(setpoint);
		Robot.drivetrain.turnController.enable();
		System.out.println("Calling PIDTURN...");
	}
	
	public void execute() {
		System.out.println("TURNING...");
	}

	protected boolean isFinished() {
		//Math.abs(setpoint - Robot.drivetrain.getCurrentAngle()) <= tolerance;
		return (Math.abs(Robot.drivetrain.turnController.getError()) <= tolerance);
	}
	
	public void end() {
		Robot.drivetrain.turnController.disable();
		Robot.drivetrain.resetPidState();
	}
}
