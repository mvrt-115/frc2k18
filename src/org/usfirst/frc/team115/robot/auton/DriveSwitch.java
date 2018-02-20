package org.usfirst.frc.team115.robot.auton;

import org.usfirst.frc.team115.robot.commands.DriveForDistance;
import org.usfirst.frc.team115.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team115.robot.commands.PidTurn;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveSwitch extends CommandGroup {

	public String profileName;
	public String startingPos;

	public DriveSwitch(String profileName, String startingPos) {
		this.profileName = profileName;
		this.startingPos = startingPos;

		if(profileName == "left" && startingPos == "A") {
			addSequential(new DriveForDistance((146.5)/12.0, 0.0));
			addSequential(new PidTurn(90));
			addParallel(new DriveTimedAutoLine(3.5, 90.0, 0.3));
			addParallel(new ElevateToSwitch());
			addSequential(new OuttakeCommand());
		}

	}
}