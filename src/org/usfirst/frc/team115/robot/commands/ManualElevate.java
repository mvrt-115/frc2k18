package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.subsystems.Elevator.ElevatorState;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ManualElevate extends InstantCommand{

	public ManualElevate() {
		requires(Robot.elevator);
	}

	protected void initialize() {
		Robot.intake.extendIntake();
		Robot.elevator.enable(true);
		Robot.elevator.updateState(ElevatorState.MANUAL);
	}

	public void execute() {
//		Robot.elevator.manualElevate(Robot.oi.getManualElevate());
	}

//	protected boolean isFinished() {
//		return false;
//	}

	public void end() {}

}
