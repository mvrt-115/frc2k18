package org.usfirst.frc.team115.robot.commands;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.UnitConverter;

import edu.wpi.first.wpilibj.command.Command;


public class ElevateToSwitch extends Command {

	boolean isAuton;
	
	public ElevateToSwitch(boolean isAuton) {
		requires(Robot.elevator);
		this.isAuton = isAuton;
	}

	public void initialize() {
		Robot.intake.extendIntake();
		Robot.elevator.setElevatorSetpoint(Constants.kDefaultSwitchHeight);
	}

	public void execute() {
		if(Robot.elevator.getError() <= UnitConverter.convertInchesToTicks(1.0)) {
			Robot.elevator.hold();
			if(isAuton) {
				Robot.intake.outtakeCube();
			}
		} 
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	public void end() {
		Robot.elevator.zero();
		if(isAuton) {
			Robot.intake.stop();
		}
	}
}
