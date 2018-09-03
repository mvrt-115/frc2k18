package org.usfirst.frc.team115.robot.commands.auton;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.commands.DriveForDistance;
import org.usfirst.frc.team115.robot.commands.ElevateToScale;
import org.usfirst.frc.team115.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team115.robot.commands.IntakeDown;
import org.usfirst.frc.team115.robot.commands.PidTurn;
import org.usfirst.frc.team115.robot.commands.ZeroElevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class DriveScale extends CommandGroup {

	public String profileName;
	public String startingPos;

	public DriveScale(String profileName, String startingPos) {
		Robot.drivetrain.zeroDrive();
		this.profileName = profileName;
		this.startingPos = startingPos;

		addSequential(new IntakeDown());
		addSequential(new TimedCommand(1.0));
		//	addSequential(new LiftIntake());

		if(profileName == "left" && startingPos == "A") {

			// Drive To Scale
			// addSequential(new DriveForDistance(285.0/12.0, 0.0));
			addSequential(new ElevateToScale("high", true), 3);
			addSequential(new DriveForDistance(Constants.distanceFromWallToScale, 0.0));
			// Turn
			addSequential(new PidTurn(60), 2);
			// addSequential(new PidTurn(45), 3);
			// Shoot
			addSequential(new TimedCommand(1.5) {
				public void execute() {
					Robot.intake.outtakeCube();
				}

				public void end() {
					Robot.intake.stop();
				}
			});
			// addSequential(new ElevateToScale("high", true), 3);

		}
		else if (profileName == "right" && startingPos == "C") {

			// Drive To Scale
			// addSequential(new DriveForDistance(285.0/12.0, 0.0));
			addSequential(new ElevateToScale("high", true), 3);
			addSequential(new DriveForDistance(Constants.distanceFromWallToScale, 0.0));
			// Turn
			addSequential(new PidTurn(-60.0), 2);
			// addSequential(new PidTurn(-45), 3);
			// Shoot
			addSequential(new TimedCommand(1.5) {
				public void execute() {
					Robot.intake.outtakeCube();
				}

				public void end() {
					Robot.intake.stop();
				}
			});
			// addSequential(new ElevateToScale("high", true), 3);

		}
		else if (profileName == "left" && startingPos == "C") {

			// Drive to alley
			addSequential(new ElevateToSwitch(true), 3);
			addSequential(new DriveForDistance(Constants.distanceFromWallToAlley - 3.5, 0.0));
			addSequential(new InstantCommand() {
				
				public void execute() {
					Robot.intake.retractIntake();
				}

			});
			//			addSequential(new PidTurn(-90), 2);
			// Drive in alley
//			addSequential(new DriveForDistance(Constants.alleyDistanceToScale + 5.0, -90.0));
			addSequential(new DriveForDistance(Constants.alleyDistanceToScale + 6.0, -90.0));
			// Turn
			//			addSequential(new PidTurn(0), 2);
			addSequential(new ElevateToScale("high", true), 3);
			addSequential(new DriveForDistance(5.5, 30.0));
			//			addSequential(new PidTurn(45), 2);
			// Shoot
			addSequential(new TimedCommand(0.5) {
				public void execute() {
					Robot.intake.softDropCube();
				}

				public void end() {
					Robot.intake.stop();
				}
			});

//			addSequential(new DriveForDistance(-1.0, 30.0));
//			addSequential(new ZeroElevator());

			// addSequential(new ElevateToScale("high", true), 3);

		}
		else if (profileName == "right" && startingPos == "A") {

			// Drive to alley
			addSequential(new ElevateToSwitch(true), 3);
			addSequential(new DriveForDistance(Constants.distanceFromWallToAlley - 3.5, 0.0));
			addSequential(new InstantCommand() {
				
				public void execute() {
					Robot.intake.retractIntake();
				}

			});
			//			addSequential(new PidTurn(-90), 2);
			// Drive in alley
//			addSequential(new DriveForDistance(Constants.alleyDistanceToScale + 5.0, 90.0));
			addSequential(new DriveForDistance(Constants.alleyDistanceToScale + 6.0, 90.0));
			// Turn
			//			addSequential(new PidTurn(0), 2);
			addSequential(new ElevateToScale("high", true), 3);
			addSequential(new DriveForDistance(5.5, -30.0));
			//			addSequential(new PidTurn(45), 2);
			// Shoot
			addSequential(new TimedCommand(0.5) {
				public void execute() {
					Robot.intake.softDropCube();
				}

				public void end() {
					Robot.intake.stop();
				}
			});

//			addSequential(new DriveForDistance(-1.0, -30.0));
//			addSequential(new ZeroElevator());

			// addSequential(new ElevateToScale("high", true), 3);
			// addSequential(new ElevateToScale("high", true), 3);

		}
	}
}