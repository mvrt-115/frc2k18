package org.usfirst.frc.team115.robot.commands.auton;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.commands.DriveForDistance;
import org.usfirst.frc.team115.robot.commands.ElevateToScale;
import org.usfirst.frc.team115.robot.commands.IntakeDown;
import org.usfirst.frc.team115.robot.commands.LiftIntake;
import org.usfirst.frc.team115.robot.commands.PidTurn;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class DriveScale extends CommandGroup {

	public String profileName;
	public String startingPos;

	public DriveScale(String profileName, String startingPos) {
		Robot.drivetrain.zeroDrive();
		this.profileName = profileName;
		this.startingPos = startingPos;

		addSequential(new IntakeDown());
		addSequential(new TimedCommand(0.5));
		addSequential(new LiftIntake());

		if(profileName == "left" && startingPos == "A") {

			// Drive To Scale
			// addSequential(new DriveForDistance(285.0/12.0, 0.0));
			addSequential(new DriveForDistance(Constants.distanceFromWallToScale, 0.0));
			// Turn
			 addSequential(new PidTurn(60), 5);
//			addSequential(new PidTurn(45), 3);
			// Shoot
			addSequential(new ElevateToScale("high", true), 3);
		
		}
		else if (profileName == "right" && startingPos == "C") {
			
			// Drive To Scale
			// addSequential(new DriveForDistance(285.0/12.0, 0.0));
			addSequential(new DriveForDistance(Constants.distanceFromWallToScale, 0.0));
			// Turn
			 addSequential(new PidTurn(-60.0), 5);
//			addSequential(new PidTurn(-45), 3);
			//Shoot
			addSequential(new ElevateToScale("high", true), 3);
		
		}
		else if (profileName == "left" && startingPos == "C") {

			// Drive to alley
			addSequential(new DriveForDistance(Constants.distanceFromWallToAlley, 0.0));
			addSequential(new PidTurn(-90), 3);
			// Drive in alley
			addSequential(new DriveForDistance(Constants.alleyDistanceToScale, -90.0));
			// Turn
			addSequential(new PidTurn(0), 3);
			addSequential(new DriveForDistance(3.0, 0.0));
			addSequential(new PidTurn(45), 3);
			// Shoot
			addSequential(new ElevateToScale("high", true), 3);

		}
		else if (profileName == "right" && startingPos == "A") {

			// Drive to alley
			addSequential(new DriveForDistance(Constants.distanceFromWallToAlley, 0.0));
			addSequential(new PidTurn(90), 3);
			// Drive in alley
			addSequential(new DriveForDistance(Constants.alleyDistanceToScale, 90.0));
			// Turn
			addSequential(new PidTurn(0), 3);
			addSequential(new DriveForDistance(3.0, 0.0));
			addSequential(new PidTurn(-45), 3);
			// Shoot
			addSequential(new ElevateToScale("high", true), 3);

		}
	}
}