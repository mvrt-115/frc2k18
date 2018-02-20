package org.usfirst.frc.team115.robot.auton;

import org.usfirst.frc.team115.robot.commands.PidTurn;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class TimedScale extends CommandGroup {

	public String profileName;
	public String startingPos;
	
	public TimedScale(String profileName, String startingPos) {
		this.profileName = profileName;
		this.startingPos = startingPos;
		
		if(profileName == "left" && startingPos == "A") {
			addSequential(new DriveTimedAutoLine((319.0/12.0)/(0.7*15) + 0.33, 0.0, 0.7));
			addSequential(new PidTurn(75));
		}
		else if(profileName == "right" && startingPos == "A") {
			addSequential(new DriveTimedAutoLine(2.24 + 0.10, 0.0, 0.7));
			addSequential(new PidTurn(93));
			addSequential(new DriveTimedAutoLine(2.507, 92.0, 0.7));
			addSequential(new PidTurn(0));
			addSequential(new DriveTimedAutoLine(1.5, 0.0, 0.3));
			addSequential(new PidTurn(-60));
			addSequential(new DriveTimedAutoLine(0.5, -60.0, -0.2));
		}
		
	}
	
}
