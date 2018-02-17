package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PidTurn extends Command {
	
	private double setpoint;
	private double tolerance = 0.25;
	
	public PidTurn(double setpoint) {
		this.setpoint = setpoint;
	}
	
	public void initialize() {		
		Robot.drivetrain.setTurnSetpoint(setpoint);
		Robot.drivetrain.turnController.enable();
	}
	
	public void execute() {
		SmartDashboard.putNumber("Turn Error", Robot.drivetrain.getError());
		SmartDashboard.putNumber("Turn Output", Robot.drivetrain.getTurnOutput());
	}

	protected boolean isFinished() {
		return Math.abs(Robot.drivetrain.turnController.getError()) == tolerance;//Math.abs(setpoint - Robot.drivetrain.getCurrentAngle()) <= tolerance;
	}
	
	public void end() {
		Robot.drivetrain.turnController.disable();
	}
}
