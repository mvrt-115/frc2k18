package org.usfirst.frc.team115.robot.auton;

import org.usfirst.frc.team115.robot.commands.PidTurn;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class TimedSwitch extends CommandGroup {

	public String profileName;
	public String startingPos;
	
	public TimedSwitch(String profileName, String startingPos) {
		this.profileName = profileName;
		this.startingPos = startingPos;
		
		if(profileName == "left" && startingPos == "A") {
			addSequential(new DriveTimedAutoLine((171.0/12.0)/(0.6*17) + 0.5, 0.0, 0.6));
			addSequential(new PidTurn(90));
			addSequential(new DriveTimedAutoLine((22.0/12)/6 + 1.0, 90.0, 0.3));
		}
		
	}
	
}
