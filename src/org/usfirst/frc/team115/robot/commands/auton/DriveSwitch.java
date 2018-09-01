package org.usfirst.frc.team115.robot.commands.auton;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.commands.DriveForDistance;
import org.usfirst.frc.team115.robot.commands.ElevateToSwitch;
import org.usfirst.frc.team115.robot.commands.IntakeCommand;
import org.usfirst.frc.team115.robot.commands.IntakeDown;
import org.usfirst.frc.team115.robot.commands.LiftIntake;
import org.usfirst.frc.team115.robot.commands.OuttakeCommand;
import org.usfirst.frc.team115.robot.commands.PidTurn;
import org.usfirst.frc.team115.robot.commands.ZeroElevator;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class DriveSwitch extends CommandGroup {

	public String profileName;
	public String startingPos;

	public DriveSwitch(String profileName, String startingPos) {
		
		Robot.drivetrain.zeroDrive();
		this.profileName = profileName;
		this.startingPos = startingPos;

		addSequential(new IntakeDown());
		addSequential(new TimedCommand(1.0));
//		addSequential(new LiftIntake());

		if(profileName == "left" && startingPos == "A") {
			
			// Drive to switch
			// addSequential(new DriveForDistance((146.5)/12.0, 0.0));
			addSequential(new ElevateToSwitch(true), 1.2);
			addSequential(new DriveForDistance(Constants.distanceFromWallToSwitch, 0.0));
			// Turn to face switch
			addSequential(new PidTurn(90), 3);
			addSequential(new LiftIntake());
			// Drive up to switch
			addSequential(new DriveTimedAutoLine(2, 90.0, 0.4));
			addSequential(new TimedCommand(0.5) {
				public void execute() {
					Robot.intake.outtakeCube();
				}

				public void end() {
					Robot.intake.stop();
					System.out.println("stop outtaking");
				}
			});
//			addSequential(new OuttakeCommand());
			// Shoot
//			addSequential(new ElevateToSwitch(true), 1.5);
			// addParallel(new DeadReckonElevateToSwitch(4));
		
		} else if (profileName == "right" && startingPos == "A") {
			
			// Drive to alley
			addSequential(new ElevateToSwitch(true), 1.2);
			addSequential(new DriveForDistance(Constants.distanceFromWallToAlley, 0.0));
			// Turn to enter into alley
			addSequential(new PidTurn(90), 3);
			// Drive in alley until switch
			addSequential(new DriveForDistance(Constants.alleyDistanceToSwitch, 90.0)); //24.0 inch less to switch?
			// Turn backwards to face switch
			addSequential(new PidTurn(180), 3);
			addSequential(new DriveForDistance(Constants.distanceFromAlleyToSwitch, 180.0));
			addSequential(new PidTurn(-90), 3);
			addSequential(new DriveTimedAutoLine(2, -90.0, 0.4));
			// Shoot
			addSequential(new TimedCommand(0.5) {
				public void execute() {
					Robot.intake.outtakeCube();
				}

				public void end() {
					Robot.intake.stop();
					System.out.println("stop outtaking");
				}
			});
//			addSequential(new OuttakeCommand());
//			addParallel(new ElevateToSwitch(true), 2);
			// addSequential(new DeadReckonElevateToSwitch(4));
			
		} else if (profileName == "left" && startingPos == "B") {
			
			// Drive and turn
			addSequential(new ElevateToSwitch(true), 1.2);
//			addSequential(new DriveForDistance(1.5, 0.0));
//			addSequential(new PidTurn(-25.0), 2);
			addSequential(new LiftIntake());
			// Drive straight to make contact with switch
			addSequential(new DriveTimedAutoLine(2.3, -30.0, 0.6));
			// Shoot
			addSequential(new TimedCommand(0.70) {
				public void execute() {
					Robot.intake.outtakeCube();
				}

				public void end() {
					Robot.intake.stop();
					System.out.println("stop outtaking");
				}
			});
			addSequential(new ZeroElevator());
			
//			addSequential(new DriveForDistance(-130.0/12.0, -25.0));
//			addSequential(new IntakeDown());
//			addSequential(new PidTurn(0.0), 1);
//			addParallel(new IntakeCommand(true), 4.0);
			
//			addSequential(new DriveForDistance(70.0/12.0, 0.0));
//			
//			addSequential(new DriveForDistance(-36.0/12.0, 0.0));
//			addSequential(new ElevateToSwitch(true), 1.2);
////			addSequential(new PidTurn(0.0), 2);
//			addSequential(new DriveTimedAutoLine(1.0, 0.55, -25.00));
//			addSequential(new TimedCommand(0.25) {
//				public void execute() {
//					Robot.intake.outtakeCube();
//				}
//
//				public void end() {
//					Robot.intake.stop();
//					System.out.println("stop outtaking");
//				}
//			});
//			addSequential(new ElevateToSwitch(true), 4);
			// addSequential(new DeadReckonElevateToSwitch(4));
		
		}  else if (profileName == "right" && startingPos == "B") {
			
			// Drive and turn
			addSequential(new ElevateToSwitch(true), 1.2);
			addSequential(new LiftIntake());
//			addSequential(new DriveForDistance(1.0, 0.0));
//			addSequential(new PidTurn(25.0), 2);
			// Drive straight to make contact with switch
			addSequential(new DriveTimedAutoLine(2.3, 25.0, 0.6));
			// Shoot
			addSequential(new TimedCommand(0.70) {
				public void execute() {
					Robot.intake.outtakeCube();
				}

				public void end() {
					Robot.intake.stop();
					System.out.println("stop outtaking");
				}
			});
			
//			addSequential(new DriveForDistance(-130.0/12.0, 20.0));
//			addSequential(new IntakeDown());
//			addSequential(new PidTurn(0.0), 1);
//			addParallel(new IntakeCommand(true), 4.0);
			

			//			addSequential(new DriveForDistance(70.0/12.0, 0.0));
//			
//			addSequential(new DriveForDistance(-36.0/12.0, 0.0));
//			addSequential(new ElevateToSwitch(true), 1.2);
////			addSequential(new PidTurn(0.0), 2);
//			addSequential(new LiftIntake());
//			addSequential(new DriveTimedAutoLine(1.0, 0.55, 20.0));
//			addSequential(new TimedCommand(0.25) {
//				public void execute() {
//					Robot.intake.outtakeCube();
//				}
//
//				public void end() {
//					Robot.intake.stop();
//					System.out.println("stop outtaking");
//				}
//			});
//			addSequential(new ElevateToSwitch(true), 4);
			// addSequential(new DeadReckonElevateToSwitch(4));
		
		} else if (profileName == "left" && startingPos == "C") {
			
			// Drive to alley
			addSequential(new ElevateToSwitch(true), 1.2);
			addSequential(new DriveForDistance(Constants.distanceFromWallToAlley, 0.0));
			// Turn to enter into alley
			addSequential(new PidTurn(-90), 3);
			// Drive in alley until switch
			addSequential(new DriveForDistance(Constants.alleyDistanceToSwitch, -90.0)); //24.0 inch less to switch?
			// Turn backwards to face switch
			addSequential(new PidTurn(-180), 3);
			addSequential(new DriveForDistance(Constants.distanceFromAlleyToSwitch, -180.0));
			addSequential(new PidTurn(90), 3);
			addSequential(new DriveTimedAutoLine(2, 90.0, 0.4));
			addSequential(new TimedCommand(0.5) {
				public void execute() {
					Robot.intake.outtakeCube();
				}

				public void end() {
					Robot.intake.stop();
					System.out.println("stop outtaking");
				}
			});
//			addSequential(new OuttakeCommand());
			// Shoot
//			addParallel(new ElevateToSwitch(true), 2);
//			 addSequential(new DeadReckonElevateToSwitch(4));
			
		} else if (profileName == "right" && startingPos == "C") {
			
			// Drive to switch
			addSequential(new ElevateToSwitch(true), 1.2);
			// addSequential(new DriveForDistance((146.5)/12.0, 0.0));
			addSequential(new DriveForDistance(Constants.distanceFromWallToSwitch, 0.0));
			// Turn to face switch
			addSequential(new PidTurn(-90.0), 3);
			// Drive up to switch
			addSequential(new DriveTimedAutoLine(2, -90.0, 0.4));
			// Shoot
			addSequential(new TimedCommand(0.5) {
				public void execute() {
					Robot.intake.outtakeCube();
				}

				public void end() {
					Robot.intake.stop();
					System.out.println("stop outtaking");
				}
			});
//			addSequential(new OuttakeCommand());
//			addSequential(new ElevateToSwitch(true), 1.5);
			// addParallel(new DeadReckonElevateToSwitch(4));
			
		}
	}
}