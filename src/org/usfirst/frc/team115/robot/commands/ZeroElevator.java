package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class ZeroElevator extends InstantCommand {

	public ZeroElevator() {
		requires(Robot.elevator);
	}
	
	public void execute() {
		Robot.elevator.zero();
	}
}
