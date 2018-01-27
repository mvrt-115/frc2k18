package org.usfirst.frc.team115.robot.auton;

import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.commands.DriveForDistance;
import org.usfirst.frc.team115.robot.commands.ElevateToScale;
import org.usfirst.frc.team115.robot.commands.PidTurn;
import org.usfirst.frc.team115.robot.commands.OuttakeCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveScale extends CommandGroup {
	
	// need "PidTurn", "DriveForDistance"
	public DriveScale() {
		if (Robot.gameRobotStartingConfig == 'A' || Robot.gameRobotStartingConfig == 'C') {
			addSequential(new DriveForDistance(21.943));
		}
		else {
			if (Robot.gameScaleConfig == 'L') {
				addSequential(new DriveForDistance(5));
				addSequential(new PidTurn(90));
				addSequential(new DriveForDistance(10));
				addSequential(new PidTurn(-90));
				addSequential(new DriveForDistance(15));
				
			} else {
				addSequential(new DriveForDistance(5));
				addSequential(new PidTurn(-90));
				addSequential(new DriveForDistance(13));
				addSequential(new PidTurn(90));
				addSequential(new DriveForDistance(15));
			}
		}
		addSequential(new ElevateToScale("default"));
		addSequential(new OuttakeCommand());
	}
}