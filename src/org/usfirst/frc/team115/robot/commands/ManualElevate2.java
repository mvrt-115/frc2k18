package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ManualElevate2 extends Command{

	public ManualElevate2() {
		requires(Robot.elevator);
	}
	
	public void execute() {
		// if(Robot.oi.manualElevatePressed())
		Robot.elevator.manualElevate(Robot.oi.getManualElevate());
		
	}

	protected boolean isFinished() {
		return false;
		// return !Robot.oi.manualElevatePressed();
	}

	public void end() {
//		Robot.elevator.zero();
	}

}
