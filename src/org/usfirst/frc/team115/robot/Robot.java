package org.usfirst.frc.team115.robot;

import org.usfirst.frc.team115.robot.auton.DriveScale;
import org.usfirst.frc.team115.robot.subsystems.Carriage;
import org.usfirst.frc.team115.robot.subsystems.DriveTrain;
import org.usfirst.frc.team115.robot.subsystems.Elevator;
import org.usfirst.frc.team115.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	DigitalInput BreakBeam = new DigitalInput(0);
	public static OI oi;
	public static DriveTrain drivetrain;
	public static Intake intake;
	public static Carriage carriage;
	public static Elevator elevator;

	public static char gameRobotStartingConfig;
	public static char gameSwitchConfig;
	public static char gameScaleConfig;


	DigitalInput breakBeam;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {


		drivetrain = new DriveTrain();
		intake = new Intake();
		carriage = new Carriage();
		elevator = new Elevator();
		oi = new OI();

		//		breakBeam = new DigitalInput(0);

		//		limitSwitch = new Digita  lInput(0);

		// chooser.addDefault("Do Nothing", null);
		// chooser.addObject("Drive Auto Line", new DriveAutoLine());
		// chooser.addObject("Drive Scale", new DriveScale());
		// chooser.addObject("Drive Switch", new DriveSwitch());

		SmartDashboard.putData("Auto mode", chooser);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		SmartDashboard.putNumber("navX output", Robot.drivetrain.getYaw());
		SmartDashboard.putNumber("DistTicks", drivetrain.getCurrentDist());
		SmartDashboard.putNumber("Distance", drivetrain.getCurrentDist());
		SmartDashboard.putNumber("Distance Error", drivetrain.getError());
		SmartDashboard.putNumber("DriveDistanceController value", drivetrain.driveDistanceController.get());
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {

		drivetrain.zeroYaw();
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		//Read game configuration via FMS and robot position via Driverstation
		//https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/826278-2018-game-data-details
		//		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		//		String robotStartingPos = "";
		//		SmartDashboard.getString("Starting Position: ", robotStartingPos); //A, B, C
		//
		//		int scalePriority, switchPriority, driveLinePriority;
		//		String tmp = "";
		//		SmartDashboard.getString("Scale Priority Value (1-3): ", tmp);
		//		scalePriority = Integer.parseInt(tmp);
		//		SmartDashboard.getString("Switch Priority Value (1-3): ", tmp);
		//		switchPriority = Integer.parseInt(tmp);
		//		SmartDashboard.getString("DriveLine Priority Value (1-3): ", tmp);
		//		driveLinePriority = Integer.parseInt(tmp);
		//		
		//		gameRobotStartingConfig = robotStartingPos.charAt(0); //A,B,C from left to right
		//		gameSwitchConfig = gameData.charAt(0); //L,R from driver view
		//		gameScaleConfig = gameData.charAt(1); //L,R from driver view
		//
		//		if (driveLinePriority == 1) {
		//			(new DriveAutoLine()).start();
		//		}
		//		else {
		//			if (scalePriority == 1) {
		//				//check if Scale is too far
		//				if ((gameRobotStartingConfig == 'A' && gameScaleConfig == 'R') || 
		//					(gameRobotStartingConfig == 'C' && gameScaleConfig == 'L')) {
		//					if (driveLinePriority == 2)
		//						(new DriveAutoLine()).start();
		//					else if (switchPriority == 2) {
		//						(new DriveSwitch()).start();
		//					}
		//				}
		//				else
		//					(new DriveScale()).start();
		//			}
		//			else if (switchPriority == 1) {
		//				(new DriveSwitch()).start();
		//			}
		//		}
		//		new DriveTimedAutoLine(1, 0.0).start();
		//		(new DriveAutoLine()).start();
//		(new DriveSwitch("left", "A")).start();
				(new DriveScale("left", "A")).start();
		//		(new TimedSwitch("left", "A")).start();
		//		(new TimedScale("right", "A")).start();

		// schedule the autonomous command (example)
		// if (autonomousCommand != null)
		//	autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {

		Scheduler.getInstance().run();
		//SmartDashboard.putNumber("Encoder R", drivetrain.frontRight.getSelectedSensorPosition(0));
		//SmartDashboard.putNumber("Encoder L", drivetrain.frontLeft.getSelectedSensorPosition(0));
		//SmartDashboard.putNumber("turnController error", Robot.drivetrain.turnController.getError());
		//SmartDashboard.putNumber("driveStraightController error", Robot.drivetrain.driveStraightController.getError());
		//SmartDashboard.putNumber("Pid Driving State", Robot.drivetrain.state);
		SmartDashboard.putNumber("Distance", drivetrain.getCurrentDist());
		SmartDashboard.putNumber("Distance Error", drivetrain.getError());
		SmartDashboard.putNumber("DriveDistanceController value", drivetrain.driveDistanceController.get());
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();

	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();

		//		SmartDashboard.putBoolean("BreakBeam Value", breakBeam.get());

		SmartDashboard.putNumber("Distance", drivetrain.getCurrentDist());
		SmartDashboard.putNumber("Distance Error", drivetrain.getError());
		SmartDashboard.putNumber("DriveDistanceController value", drivetrain.driveDistanceController.get());

		SmartDashboard.putNumber("Encoder Left Distance", drivetrain.convertTicksToFeet(drivetrain.frontLeft.getSelectedSensorPosition(0)));
		SmartDashboard.putNumber("Encoder Right Distance", drivetrain.convertTicksToFeet(drivetrain.frontRight.getSelectedSensorPosition(0)));

		//		SmartDashboard.putBoolean("BreakBeam Value", breakBeam.get());

		/*	SmartDashboard.putNumber("Right Dist", drivetrain.getRightDist());
		SmartDashboard.putNumber("Left Dist", drivetrain.getLeftDist());
		SmartDashboard.putNumber("Right Vel", drivetrain.getRightVelk                                                                                                   	1`());
		SmartDashboard.putNumber("Left Vel", drivetrain.getLeftVel());
		SmartDashboard.putNumber("Current Dist", drivetrain.getCurrentDist());
		SmartDashboard.putNumber("Current Vel", drivetrain.getCurrentVel());
		 */

		//		SmartDashboard.putNumber("Elevator Encoder value", elevator.left.getSelectedSensorPosition(0));

		elevator.log();

		//	 chooser.addDefault("Do Nothing", null);
		// chooser.addObject("Drive Auto Line", new DriveAutoLine());
		// chooser.addObject("Drive Scale", new DriveScale());
		// chooser.addObject("Drive Switch", new DriveSwitch());

	}


	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
