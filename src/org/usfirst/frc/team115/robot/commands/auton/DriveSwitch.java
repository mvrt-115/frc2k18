package org.usfirst.frc.team115.robot.commands.auton;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.commands.DriveForDistance;
import org.usfirst.frc.team115.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team115.robot.commands.IntakeDown;
import org.usfirst.frc.team115.robot.commands.LiftIntake;
import org.usfirst.frc.team115.robot.commands.PidTurn;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class DriveSwitch extends CommandGroup {

	public String profileName;
	public String startingPos;

	public DriveSwitch(String profileName, String startingPos) {
		
		Robot.drivetrain.zeroDrive();
		this.profileName = profileName;
		this.startingPos = startingPos;

		addSequential(new IntakeDown());
		addSequential(new TimedCommand(0.5));
		addSequential(new LiftIntake());

		if(profileName == "left" && startingPos == "A") {
			
			// Drive to switch
			// addSequential(new DriveForDistance((146.5)/12.0, 0.0));
			addSequential(new DriveForDistance(Constants.distanceFromWallToSwitch, 0.0));
			// Turn to face switch
			addSequential(new PidTurn(90), 3);
			// Drive up to switch
			addSequential(new DriveTimedAutoLine(2, 90.0, 0.4));
			// Shoot
			addSequential(new ElevateToSwitch(true), 1.5);
			// addParallel(new DeadReckonElevateToSwitch(4));
		
		} else if (profileName == "right" && startingPos == "A") {
			
			// Drive to alley
			addSequential(new DriveForDistance(Constants.distanceFromWallToAlley, 0.0));
			// Turn to enter into alley
			addSequential(new PidTurn(90), 3);
			// Drive in alley until switch
			addSequential(new DriveForDistance(Constants.alleyDistanceToSwitch, 90.0)); //24.0 inch less to switch?
			// Turn backwards to face switch
			addSequential(new PidTurn(180), 3);
			addSequential(new DriveForDistance(Constants.distanceFromAlleyToSwitch, 180.0));
			addSequential(new PidTurn(-90), 3);
			addParallel(new DriveTimedAutoLine(2, -90.0, 0.4));
			// Shoot
			addParallel(new ElevateToSwitch(true), 2);
			// addSequential(new DeadReckonElevateToSwitch(4));
			
		} else if (profileName == "left" && startingPos == "B") {
			
			// Drive and turn
			addSequential(new DriveForDistance(1.0, 0.0));
			addSequential(new PidTurn(-30.0), 3);
			// Drive straight to make contact with switch
			addSequential(new DriveTimedAutoLine(3.0, -30.0, 0.45));
			// Shoot
			addSequential(new ElevateToSwitch(true), 4);
			// addSequential(new DeadReckonElevateToSwitch(4));
		
		}  else if (profileName == "right" && startingPos == "B") {
			
			// Drive and turn
			addSequential(new DriveForDistance(1.0, 0.0));
			addSequential(new PidTurn(25.0), 3);
			// Drive straight to make contact with switch
			addSequential(new DriveTimedAutoLine(3.0, 25.0, 0.45));
			// Shoot
			addSequential(new ElevateToSwitch(true), 4);
			// addSequential(new DeadReckonElevateToSwitch(4));
		
		} else if (profileName == "left" && startingPos == "C") {
			
			// Drive to alley
			addSequential(new DriveForDistance(Constants.distanceFromWallToAlley, 0.0));
			// Turn to enter into alley
			addSequential(new PidTurn(-90), 3);
			// Drive in alley until switch
			addSequential(new DriveForDistance(Constants.alleyDistanceToSwitch, -90.0)); //24.0 inch less to switch?
			// Turn backwards to face switch
			addSequential(new PidTurn(-180), 3);
			addSequential(new DriveForDistance(Constants.distanceFromAlleyToSwitch, -180.0));
			addSequential(new PidTurn(90), 3);
			addParallel(new DriveTimedAutoLine(2, 90.0, 0.4));
			// Shoot
			addParallel(new ElevateToSwitch(true), 2);
			// addSequential(new DeadReckonElevateToSwitch(4));
			
		} else if (profileName == "right" && startingPos == "C") {
			
			// Drive to switch
			// addSequential(new DriveForDistance((146.5)/12.0, 0.0));
			addSequential(new DriveForDistance(Constants.distanceFromWallToSwitch, 0.0));
			// Turn to face switch
			addSequential(new PidTurn(-90.0), 3);
			// Drive up to switch
			addSequential(new DriveTimedAutoLine(2, -90.0, 0.4));
			// Shoot
			addSequential(new ElevateToSwitch(true), 1.5);
			// addParallel(new DeadReckonElevateToSwitch(4));
			
		}
	}
}