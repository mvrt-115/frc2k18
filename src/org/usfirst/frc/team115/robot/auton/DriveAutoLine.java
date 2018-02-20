package org.usfirst.frc.team115.robot.auton;

import org.usfirst.frc.team115.robot.commands.DriveForDistance;
import org.usfirst.frc.team115.robot.commands.PidTurn;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveAutoLine extends CommandGroup {

	// need "PidTurn", "DriveForDistance"
	public DriveAutoLine() {
		
		//Working Kitbot leftSwitchA
		//addSequential(new DriveTimedAutoLine(2.6, 0.0, 0.4)); //driving up to switch
		//addSequential(new PidTurn(90));
		//addSequential(new DriveTimedAutoLine((22.0/12)/6 + 0.5, 90.0, 0.4)); //contacting switch

		//Working Kitbot leftScaleA
		//addSequential(new DriveTimedAutoLine(3.38, 0.0, 0.6));
		//addSequential(new PidTurn(90));
		
		//Working Kitbot rightScaleA
		//addSequential(new DriveTimedAutoLine(2.5, 0.0, 0.6));
		//addSequential(new PidTurn(90));
		//addSequential(new DriveTimedAutoLine(2.3, 90.0, 0.6));
		//addSequential(new PidTurn(0));
		//addSequential(new DriveTimedAutoLine(0.25, 0.0, 0.6));
		
		//Working practiceBot leftSwitchA
//		addSequential(new DriveTimedAutoLine((171.0/12.0)/(0.6*17) + 0.5, 0.0, 0.6));
//		addSequential(new PidTurn(90));
//		addSequential(new DriveTimedAutoLine((22.0/12)/6 + 1.0, 90.0, 0.3));
		
		
		
//		addSequential(new DriveTimedAutoLine((319.0/12.0)/(0.7*15), 0.0, 0.7);
//		addSequential(new PidTurn(90));
		
//		addSequential(new DriveForDistance(3.0, 90.0));
	}
}