package org.usfirst.frc.team115.robot.auton;

import org.usfirst.frc.team115.robot.commands.DriveForDistance;
import org.usfirst.frc.team115.robot.commands.ElevateToScale;
import org.usfirst.frc.team115.robot.commands.PidTurn;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveScale extends CommandGroup {
	
	// need "PidTurn", "DriveForDistance"
	public String profileName;
	public String startingPos;
	
	public DriveScale(String profileName, String startingPos) {
		this.profileName = profileName;
		this.startingPos = startingPos;
		
		if(profileName == "left" && startingPos == "A") {
			addSequential(new DriveForDistance(285.0/12.0, 0.0));
			addSequential(new PidTurn(60));
			addSequential(new ElevateToScale("default"));
		}
	}
}