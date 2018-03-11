package org.usfirst.frc.team115.robot.commands.auton;

import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.commands.DriveForDistance;
import org.usfirst.frc.team115.robot.commands.IntakeDown;
import org.usfirst.frc.team115.robot.commands.LiftIntake;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class DriveAutoLine extends CommandGroup {

	// need "PidTurn", "DriveForDistance"
	public DriveAutoLine() {
		Robot.drivetrain.zeroDrive();
		addSequential(new IntakeDown());
		addSequential(new TimedCommand(0.3));
		addSequential(new LiftIntake());
		addSequential(new DriveForDistance((146.5)/12.0, 0.0));
	}
}