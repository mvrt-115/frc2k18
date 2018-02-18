package org.usfirst.frc.team115.robot.auton;

import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.commands.MPTrajectoryPointLoader;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;

public class DriveMP extends Command {
	
	String profileName;
	String startingPos;
	MPTrajectoryPointLoader driver;
	Notifier timer;
	
	public DriveMP(String profileName, String startingPos) {
		requires(Robot.drivetrain);
		this.profileName = profileName;
		driver = new MPTrajectoryPointLoader(profileName, startingPos);
		driver.initialize();
		timer = new Notifier(new Runnable() {
			public void run() {
				System.out.println("Calling driver.drive()");
				driver.drive();
			}
		});
		//push points into queue at a faster rate than popping them (running them)
		timer.startPeriodic(0.02);
	}
	
	public boolean isFinished() {
		return driver.isFinished();
	}
}
