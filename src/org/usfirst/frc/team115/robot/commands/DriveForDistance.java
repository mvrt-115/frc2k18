package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveForDistance extends Command {
	
	private double setpoint;
	private double heading;
	private double tolerance = 0.02;
	
	public DriveForDistance(double setpoint, double heading) {
		this.setpoint = setpoint;
		this.heading = heading;
		Robot.drivetrain.zeroPosition();
//		Robot.drivetrain.setPeakSpeed(Math.min(setpoint/4.0, 0.7));
	}
	
	public void initialize() {
		Robot.drivetrain.zeroPosition();
		Robot.drivetrain.pidDriveDistance(setpoint);
		Robot.drivetrain.pidDriveStraight(heading, 0.0);
		Robot.drivetrain.driveDistanceController.enable();
		Robot.drivetrain.driveStraightController.enable();
		System.out.println("Calling DriveForDistance with setpoint = " + setpoint + ", heading = " + heading);
	}
	
	public void execute() {
//		Robot.drivetrain.setDistanceSetpoint(setpoint);
	}
	
	public boolean isFinished() {
		boolean ret = (Math.abs(Robot.drivetrain.getError()) <= tolerance);
		if (ret)
			System.out.println("FINISHING EARLY");
		return ret;
	}
	
	public void end() {
		Robot.drivetrain.driveDistanceController.disable();
		Robot.drivetrain.stop();
		Robot.drivetrain.resetPidState();
	}

}
