package org.usfirst.frc.team115.robot.auton;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTimedAutoLine extends TimedCommand {
	private double heading;
	
	public DriveTimedAutoLine(double timeout, double heading) {
		super(timeout);
		requires(Robot.drivetrain);
		this.heading = heading;
	}
	
	public void initialize() {
		Robot.drivetrain.pidDriveStraight(heading);
	}
	
	private double turnOutput;
	
	public void execute() {
		turnOutput = Robot.drivetrain.getDriveStraightOutput();
		SmartDashboard.putNumber("DriveTimedAutoLine turnOutput", turnOutput);

		Robot.drivetrain.setLeftRightMotorOutputs((0.3 + turnOutput), (0.3 - turnOutput));
//		Robot.drivetrain.setLeftRightMotorOutputs(turnOutput, -turnOutput);
	}

	public void end() {
		Robot.drivetrain.turnController.disable();
		Robot.drivetrain.stop();
	}
	
}
