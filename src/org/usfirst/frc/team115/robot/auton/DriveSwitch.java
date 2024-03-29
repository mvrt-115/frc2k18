package org.usfirst.frc.team115.robot.auton;

import org.usfirst.frc.team115.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;

public class DriveSwitch {
	
	// need "PidTurn", "DriveForDistance"
	public DriveSwitch() {
		if (Robot.gameRobotStartingConfig == 'A') {
			if (Robot.gameSwitchConfig == 'L') {
				addSequential(new DriveForDistance(16.5));
				addSequential(new PidTurn(-90));
				addSequential(new DriveForDistance(3.4375));
				addSequential(new PidTurn(-90));
			} else {
				addSequential(new DriveForDistance(16.5));
				addSequential(new PidTurn(-90));
				addSequential(new DriveForDistance(9.4375));
				addSequential(new PidTurn(-90));
			}
		} else if (Robot.gameRobotStartingConfig == 'C') {
			if (Robot.gameSwitchConfig == 'L') {	
				addSequential(new DriveForDistance(16.5));
				addSequential(new PidTurn(90));
				addSequential(new DriveForDistance(9.4375));
				addSequential(new PidTurn(90));
			} else {
				addSequential(new DriveForDistance(16.5));
				addSequential(new PidTurn(90));
				addSequential(new DriveForDistance(3.4375));
				addSequential(new PidTurn(90));
			}
		} else {
			if (Robot.gameSwitchConfig == 'L') {
				addSequential(new DriveForDistance(5));
				addSequential(new PidTurn(90));
				addSequential(new DriveForDistance(13));
				addSequential(new PidTurn(-90));
				addSequential(new DriveForDistance(11.5));
				addSequential(new PidTurn(-90));
				addSequential(new DriveForDistance(4.5));
				addSequential(new PidTurn(-90));
			} else {
				addSequential(new DriveForDistance(5));
				addSequential(new PidTurn(-90));
				addSequential(new DriveForDistance(7.525));
				addSequential(new PidTurn(90));
				addSequential(new DriveForDistance(11.5));
				addSequential(new PidTurn(90));
				addSequential(new DriveForDistance(4.5));
				addSequential(new PidTurn(90));
			}
		}
		addSequential(new ElevateToSwitch("default")); //Regular height
		addSequential(new Outtake());
	}
}