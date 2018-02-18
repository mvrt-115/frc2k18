package org.usfirst.frc.team115.robot.auton;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTimedAutoLine extends TimedCommand {
	private double heading;
	private double speed;
	
	public DriveTimedAutoLine(double timeout, double heading, double speed) {
		super(timeout);
		requires(Robot.drivetrain);
		this.heading = heading;
		this.speed = speed;
	}
	
	public void initialize() {
		Robot.drivetrain.pidDriveStraight(heading);
	}
	
	private double turnOutput;
	
	public void execute() {
		turnOutput = Robot.drivetrain.getDriveStraightOutput();
		SmartDashboard.putNumber("DriveTimedAutoLine turnOutput", turnOutput);

		Robot.drivetrain.setLeftRightMotorOutputs((speed + turnOutput), (speed - turnOutput));
	}

	public void end() {
		Robot.drivetrain.turnController.disable();
		Robot.drivetrain.stop();
	}
	
}
