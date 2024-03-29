
package org.usfirst.frc.team115.robot;

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

import org.usfirst.frc.team115.robot.auton.DriveAutoLine;
import org.usfirst.frc.team115.robot.auton.DriveScale;
import org.usfirst.frc.team115.robot.auton.DriveSwitch;

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
	
	DigitalInput limitSwitch;

	Command autonomousCommand;
	SendableChooser<Command> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		oi = new OI();
		drivetrain = new DriveTrain();
		intake = new Intake();
		carriage = new Carriage();
		elevator = new Elevator();
		
		limitSwitch = new DigitalInput(0);

		chooser.addDefault("Do Nothing", null);
		chooser.addObject("Drive Auto Line", new DriveAutoLine());
		chooser.addObject("Drive Scale", new DriveScale());
		chooser.addObject("Drive Switch", new DriveSwitch());

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
		autonomousCommand = chooser.getSelected();

		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */

		//Read game configuration via FMS and robot position via Driverstation
		//https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/826278-2018-game-data-details
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		String robotStartingPos;
		SmartDashboard.getString("Starting Position: ", robotStartingPos); //A, B, C

		gameRobotStartingConfig = robotStartingPos.charAt(0); //A,B,C from left to right
		gameSwitchConfig = gameData.charAt(0); //L,R from driver view
		gameScaleConfig = gameData.charAt(1); //L,R from driver view


		// schedule the autonomous command (example)
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
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
		SmartDashboard.putBoolean("Limit Switch Pressed", limitSwitch.get());
	}

	
	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
