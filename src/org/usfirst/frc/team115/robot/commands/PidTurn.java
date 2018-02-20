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
	}
	
	public void execute() {
	}

	protected boolean isFinished() {
		boolean ret = Math.abs(Robot.drivetrain.turnController.getError()) <= tolerance;
		if (ret)
			System.out.println("TURN ENDED EARLY");
		return ret; //Math.abs(setpoint - Robot.drivetrain.getCurrentAngle()) <= tolerance;
	}
	
	public void end() {
		Robot.drivetrain.turnController.disable();
		Robot.drivetrain.resetPidState();
	}
}
