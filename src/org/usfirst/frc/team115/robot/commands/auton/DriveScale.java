package org.usfirst.frc.team115.robot.commands.auton;

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
		this.profileName = profileName;
		this.startingPos = startingPos;
		
		addSequential(new IntakeDown());
		addSequential(new TimedCommand(0.5));
		addSequential(new LiftIntake());
		
		if(profileName == "left" && startingPos == "A") {
			addSequential(new DriveForDistance(285.0/12.0, 0.0));
			addSequential(new PidTurn(60));
			addSequential(new ElevateToScale("high", true), 3);
		}
		else if (profileName == "right" && startingPos == "C") {
			addSequential(new DriveForDistance(285.0/12.0, 0.0));
			addSequential(new PidTurn(-60.0));
			addSequential(new ElevateToScale("high", true), 3);
		}
	}
}