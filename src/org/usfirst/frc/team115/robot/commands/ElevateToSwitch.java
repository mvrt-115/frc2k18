package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;


public class ElevateToSwitch extends InstantCommand {

	boolean isAuton;
	
	public ElevateToSwitch(boolean isAuton) {
		requires(Robot.elevator);
		this.isAuton = isAuton;
	}

	public void initialize() {
		Robot.intake.extendIntake();
		Robot.elevator.setElevatorSetpoint(Constants.kDefaultSwitchHeight);
		Robot.elevator.enable(true);
	}

	public void execute() {
//		Robot.elevator.manualElevate(-0.3);
//		if(Robot.elevator.getTopLimit()) {
//		if(Robot.elevator.getError() <= UnitConverter.convertInchesToTicks(1.0)) {
//			Robot.elevator.hold();
//			if(isAuton) {
//				Robot.intake.setOuttakeSpeed(-0.5);
//				Robot.intake.outtakeCube();
//			}
//		}
	}

	public void end() {
//		Robot.elevator.updateState(ElevatorState.ZEROING);
//		if(isAuton) {
//			Robot.intake.stop();
//			Robot.intake.setOuttakeSpeed(-1.0);
//		}
	}
}
