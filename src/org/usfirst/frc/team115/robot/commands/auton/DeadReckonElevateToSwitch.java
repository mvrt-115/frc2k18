package org.usfirst.frc.team115.robot.commands.auton;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

public class DeadReckonElevateToSwitch extends TimedCommand {

	public DeadReckonElevateToSwitch(double timeout) {
		super(timeout);
		// TODO Auto-generated constructor stub
	}
	
	public void initialize() {
	}
	
	public void execute() {
		if (this.timeSinceInitialized() < 2.0) {
			Robot.elevator.manualElevate(-0.3);
		}
		else {
			Robot.elevator.hold();
			Robot.intake.outtakeCube();
		}
	}

	public void end() {
	}

}
