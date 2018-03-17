package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.subsystems.Elevator.ElevatorState;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ZeroElevator extends InstantCommand {

	public ZeroElevator() {
		requires(Robot.elevator);
	}
	
	public void initialize() {
		Robot.elevator.enable(true);
		Robot.elevator.updateState(ElevatorState.ZEROING);
	}
	
//	public void end() {
//		Robot.elevator.zero();
//	}
	
}
