package org.usfirst.frc.team115.robot;


import org.usfirst.frc.team115.robot.commands.auton.DriveAutoLine;
import org.usfirst.frc.team115.robot.commands.auton.DriveScale;
import org.usfirst.frc.team115.robot.commands.auton.DriveSwitch;
import org.usfirst.frc.team115.robot.commands.auton.TwoCubeScaleSwitch;
import org.usfirst.frc.team115.robot.subsystems.Carriage;
import org.usfirst.frc.team115.robot.subsystems.DriveTrain;
import org.usfirst.frc.team115.robot.subsystems.Elevator;
import org.usfirst.frc.team115.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
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

	public static OI oi;
	public static DriveTrain drivetrain;
	public static Intake intake;
	public static Carriage carriage;
	public static Elevator elevator;

	public static char gameRobotStartingConfig;
	public static char gameSwitchConfig;
	public static char gameScaleConfig;
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

		// chooser.addDefault("Do Nothing", null);
		// chooser.addObject("Drive Auto Line", new DriveAutoLine());
		// chooser.addObject("Drive Scale", new DriveScale());
		// chooser.addObject("Drive Switch", new DriveSwitch());

		SmartDashboard.putData("Auto mode", chooser);
		SmartDashboard.putString(Constants.autonChoiceString, "DriveLine");
		SmartDashboard.putString("Starting Position(A, B, C):", "N");
		
		Compressor compressor = new Compressor(1);
		compressor.setClosedLoopControl(true);
		compressor.start();

		
		CameraServer.getInstance().startAutomaticCapture(0);
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		SmartDashboard.putString(Constants.autonChoiceString, "DriveLine");
		SmartDashboard.putString("Starting Position(A, B, C):", "N");
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		elevator.log();
		drivetrain.log();
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
		elevator.zero();
		// autonomousCommand = chooser.getSelected();
		drivetrain.zeroDrive();
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		String gameData = DriverStation.getInstance().getGameSpecificMessage();
		String robotStartingPos = "";
		robotStartingPos = SmartDashboard.getString("Starting Position(A, B, C):", "N"); //A, B, C

		String autonChoice = "";
		autonChoice = SmartDashboard.getString(Constants.autonChoiceString, "DriveLine");
		autonChoice = autonChoice.trim();

		gameRobotStartingConfig = robotStartingPos.charAt(0); //A,B,C from left to right
		gameSwitchConfig = gameData.charAt(0); //L,R from driver view
		gameScaleConfig = gameData.charAt(1); //L,R from driver view

		switch (robotStartingPos) {
		case "A":
			if (autonChoice.equalsIgnoreCase("2Cube")) {
				if (gameSwitchConfig == 'L') {
					if (gameScaleConfig == 'L')
						(new TwoCubeScaleSwitch("left", "A")).start();
					else
						(new DriveSwitch("left", "A")).start();
				}
				else if (gameScaleConfig == 'L')
					(new DriveScale("left", "A")).start();
				else
					(new DriveAutoLine()).start();
			}
			else if (autonChoice.equalsIgnoreCase("Switch")) {
				if (gameSwitchConfig == 'L')
					(new DriveSwitch("left", "A")).start();
				else if (gameScaleConfig == 'L') {
					// (new DriveScale()).start();
					(new DriveAutoLine()).start();
				}
				else
					(new DriveAutoLine()).start();
			}
			else if (autonChoice.equalsIgnoreCase("Scale")) {
				if (gameScaleConfig == 'R')
					(new DriveScale("right", "A")).start();
				else
					(new DriveScale("left", "A")).start();
			}
			else
				(new DriveAutoLine()).start();
			break;
		case "B":
			if(gameSwitchConfig == 'L')
				(new DriveSwitch("left", "B")).start();
			else if(gameSwitchConfig == 'R')
				(new DriveSwitch("right", "B")).start();
			break;
		case "C":
			System.out.println("Case C");
			System.out.println("autonChoice: " + autonChoice);
			System.out.println("Game switch config: " + gameSwitchConfig);
			if (autonChoice.equalsIgnoreCase("2Cube")) {
				if (gameSwitchConfig == 'R') {
					if (gameScaleConfig == 'R')
						(new TwoCubeScaleSwitch("right", "C")).start();
					else
						(new DriveSwitch("right", "C")).start();
				}
				else if (gameScaleConfig == 'R')
					(new DriveScale("right", "C")).start();
				else
					(new DriveAutoLine()).start();
			}
			else if (autonChoice.equalsIgnoreCase("Switch")) {
				System.out.println("C switch");
				if (gameSwitchConfig == 'R') {
					System.out.println("Calling right, C switch");
					(new DriveSwitch("right", "C")).start();
				}
				else if (gameScaleConfig == 'R') {
//					 (new DriveScale()).start();
					(new DriveAutoLine()).start();
				}
				else
					(new DriveAutoLine()).start();
			}
			else if (autonChoice.equals("Scale")) {
				if (gameScaleConfig == 'R')
					(new DriveScale("right", "C")).start();
				else
					(new DriveScale("left", "C")).start();
			}
			else {
				System.out.println("AUTO LINE SWITCH");
				(new DriveAutoLine()).start();
			}
			break;
		default:
			(new DriveAutoLine()).start(); 
			break;
		}
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
		elevator.zero();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		//		SmartDashboard.putBoolean("Limit Switch Pressed", limitSwitch.get());

		//		SmartDashboard.putNumber("Sensor Reading", a);
		//		SmartDashboard.putNumber("Right Dist", drivetrain.getRightDist());
		//		SmartDashboard.putNumber("Left Dist", drivetrain.getLeftDist());
		//		SmartDashboard.putNumber("Right Vel", drivetrain.getRightVel());
		//		SmartDashboard.putNumber("Left Vel", drivetrain.getLeftVel());
		//		SmartDashboard.putNumber("Current Dist", drivetrain.getCurrentDist());
		//		SmartDashboard.putNumber("Current Vel", drivetrain.getCurrentVel());
		//		SmartDashboard.putNumber("Sensor Reading", hallEffect.getAverageVoltage());
		elevator.log();
		drivetrain.log();
		carriage.log();
		SmartDashboard.putBoolean("Cube Detected?", carriage.cubeDetected());

		//		SmartDashboard.putBoolean("BreakBeam Value", breakBeam.get());
		//		SmartDashboard.putBoolean("BreakBeam Value", breakBeam.get());

		/*	SmartDashboard.putNumber("Right Dist", drivetrain.getRightDist());
		SmartDashboard.putNumber("Left Dist", drivetrain.getLeftDist());
		SmartDashboard.putNumber("Right Vel", drivetrain.getRightVelk                                                                                                   	1`());
		SmartDashboard.putNumber("Left Vel", drivetrain.getLeftVel());
		SmartDashboard.putNumber("Current Dist", drivetrain.getCurrentDist());
		SmartDashboard.putNumber("Current Vel", drivetrain.getCurrentVel());
		 */

		//		SmartDashboard.putNumber("Elevator Encoder value", elevator.left.getSelectedSensorPosition(0));

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
