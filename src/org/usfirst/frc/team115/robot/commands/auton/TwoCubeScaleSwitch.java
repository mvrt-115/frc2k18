package org.usfirst.frc.team115.robot.commands.auton;

import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.commands.DriveForDistance;
import org.usfirst.frc.team115.robot.commands.ElevateToScale;
import org.usfirst.frc.team115.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team115.robot.commands.IntakeCommand;
import org.usfirst.frc.team115.robot.commands.IntakeDown;
import org.usfirst.frc.team115.robot.commands.OuttakeCommand;
import org.usfirst.frc.team115.robot.commands.PidTurn;
import org.usfirst.frc.team115.robot.commands.ZeroElevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class TwoCubeScaleSwitch extends CommandGroup {

	public String profileName;
	public String startingPos;
	
	public TwoCubeScaleSwitch(String profileName, String startingPos) {
		Robot.drivetrain.zeroDrive();
		this.profileName = profileName;
		this.startingPos = startingPos;
		
		addSequential(new IntakeDown());
//		addSequential(new TimedCommand(0.5));
//		addSequential(new LiftIntake());
		
		if(profileName == "left" && startingPos == "A") {
			addSequential(new ElevateToScale("high", true), 3);
			addSequential(new DriveForDistance(285.0/12.0, 0.0));
			addSequential(new PidTurn(60));
			addSequential(new OuttakeCommand());
			addSequential(new ZeroElevator());
			addSequential(new PidTurn(155));
			addParallel(new IntakeCommand());
			addSequential(new DriveTimedAutoLine(2.0, 155.0, 0.6));
			addSequential(new ElevateToSwitch(true));
			addSequential(new OuttakeCommand(), 2);
		} else if (profileName == "right" && startingPos == "C") {
			addSequential(new ElevateToScale("high", true));
			addSequential(new DriveForDistance(285.0/12.0, 0.0));
			addSequential(new PidTurn(-60.0));
			addSequential(new OuttakeCommand());
			addSequential(new ZeroElevator());
			addSequential(new PidTurn(-155));
			addParallel(new IntakeCommand());
			addSequential(new DriveTimedAutoLine(4.0, -155.0, 0.6));
			addParallel(new ElevateToSwitch(true));
			addSequential(new OuttakeCommand(), 2);
		}
	}

}
