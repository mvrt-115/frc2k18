package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class LowerElevator extends InstantCommand{

	public LowerElevator() {
		requires(Robot.elevator);
	}
	
	public void execute() {
		Robot.elevator.home();
	}
}
