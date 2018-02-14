package org.usfirst.frc.team115.robot.auton;

import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.commands.DriveForDistance;
import org.usfirst.frc.team115.robot.commands.PidTurn;
import org.usfirst.frc.team115.robot.subsystems.DriveTrain;
import org.usfirst.frc.team115.robot.commands.OuttakeCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveAutoLine extends CommandGroup {

	// need "PidTurn", "DriveForDistance"
	public DriveAutoLine() {
		
		System.out.println("In Drive AutoLine");
		addSequential(new DriveForDistance(14.5));

		
		if(Robot.gameRobotStartingConfig == 'B') { //Need to turn first
			addSequential(new DriveForDistance(3.472));
			addSequential(new PidTurn(-90));
			addSequential(new DriveForDistance(5.625));
			addSequential(new PidTurn(-90));
			addSequential(new DriveForDistance(12.4655));
		}
		
		else
			addSequential(new DriveForDistance(14.5));
	}
}