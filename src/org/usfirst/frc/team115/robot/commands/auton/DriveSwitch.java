package org.usfirst.frc.team115.robot.commands.auton;

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
		this.profileName = profileName;
		this.startingPos = startingPos;

		addSequential(new IntakeDown());
		addSequential(new TimedCommand(0.5));
		addSequential(new LiftIntake());
		
		if(profileName == "left" && startingPos == "A") {
			addSequential(new DriveForDistance((146.5)/12.0, 0.0));
			addSequential(new PidTurn(90));
			addParallel(new DriveTimedAutoLine(3.5, 90.0, 0.4));
			addParallel(new ElevateToSwitch(true), 2);
		} else if (profileName == "left" && startingPos == "B") {
			addSequential(new DriveForDistance(1.0, 0.0));
			addSequential(new PidTurn(-30.0));
			addParallel(new DriveTimedAutoLine(3.0, -30.0, 0.45));
			addParallel(new ElevateToSwitch(true), 10.0);
		} else if (profileName == "left" && startingPos == "C") {

		} else if (profileName == "right" && startingPos == "A") {

		} else if (profileName == "right" && startingPos == "B") {
			addSequential(new DriveForDistance(3.0, 0.0));
			addSequential(new PidTurn(20.0));
			addSequential(new DriveTimedAutoLine(3.0, 20.0, 0.6));
			addSequential(new ElevateToSwitch(true), 2);
		} else if (profileName == "right" && startingPos == "C") {
			addSequential(new DriveForDistance((146.5)/12.0, 0.0));
			addSequential(new PidTurn(-90.0));
			addParallel(new DriveTimedAutoLine(3.5, -90.0, 0.4));
			addParallel(new ElevateToSwitch(true), 2);
		}
	}
}