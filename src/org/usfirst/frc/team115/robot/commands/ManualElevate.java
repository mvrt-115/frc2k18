package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ManualElevate extends Command{

	public ManualElevate() {
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
