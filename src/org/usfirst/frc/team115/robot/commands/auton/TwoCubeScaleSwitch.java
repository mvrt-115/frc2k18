package org.usfirst.frc.team115.robot.commands.auton;

import org.usfirst.frc.team115.robot.Constants;
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
		addSequential(new TimedCommand(0.8));
//		addSequential(new LiftIntake());
		
		if(profileName == "left" && startingPos == "A") {
			addSequential(new ElevateToScale("high", true), 3);
			addSequential(new DriveForDistance(Constants.distanceFromWallToScale, 0.0));
			addSequential(new PidTurn(60));
			addSequential(new TimedCommand(0.25) {
				public void execute() {
					Robot.intake.outtakeCube();
				}

				public void end() {
					Robot.intake.stop();
				}
			});
			addSequential(new ZeroElevator());
			addSequential(new PidTurn(162));
			addParallel(new IntakeCommand());
			addSequential(new DriveTimedAutoLine(2.0, 162.0, 0.6));
			addSequential(new ElevateToSwitch(true));
			addSequential(new TimedCommand(0.5) {
				public void execute() {
					Robot.intake.outtakeCube();
				}

				public void end() {
					Robot.intake.stop();
				}
			});
		} else if (profileName == "right" && startingPos == "C") {
//			addSequential(new ElevateToSwitch(true));
			addSequential(new ElevateToScale("high", true));
			addSequential(new DriveForDistance(Constants.distanceFromWallToScale, 0.0));
			addSequential(new PidTurn(-60.0), 1.5);
			addSequential(new TimedCommand(0.50) {
				public void execute() {
					Robot.intake.outtakeCube();
				}

				public void end() {
					Robot.intake.stop();
					System.out.println("stop outtaking");
				}
			});
			addSequential(new ZeroElevator());
			addSequential(new PidTurn(-158.0), 1.5);
			addParallel(new IntakeCommand(true), 5.0);
//			addSequential(new TimedCommand(0.25));
//			addSequential(new DriveForDistance(95.0/12.0, -158.0), 3.0);
			addSequential(new DriveTimedAutoLine(4.0, -158.0, 0.2));
			addSequential(new TimedCommand(0.5));
			addSequential(new ElevateToSwitch(true));
			addSequential(new DriveForDistance(1.5, -158.0), 1.0);
			addSequential(new TimedCommand(2.0) {
				
				public void execute() {
					Robot.intake.outtakeCube();
				}

				public void end() {
					Robot.intake.stop();
				}
			});
		}
	}

}
