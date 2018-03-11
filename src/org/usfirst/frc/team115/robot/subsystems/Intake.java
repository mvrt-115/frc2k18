package org.usfirst.frc.team115.robot.subsystems;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Hardware;
import org.usfirst.frc.team115.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem {

	public double outtakeSpeed = -1.0;
	
	public Intake()  {
		Hardware.intakeLeft = new TalonSRX(Constants.kIntakeLeftTalonID);
		Hardware.intakeRight = new TalonSRX(Constants.kIntakeRightTalonID);
		Hardware.intakeExtendSolenoid = new DoubleSolenoid(1, 6, 1);
		Hardware.intakeStowLeft = new DoubleSolenoid(1, 0, 7);
		Hardware.intakeStowRight = new DoubleSolenoid(1, 2, 5);
		
		Hardware.intakeLeft.configContinuousCurrentLimit(10, 0);
		Hardware.intakeRight.configContinuousCurrentLimit(10, 0);
		Hardware.intakeLeft.configPeakCurrentLimit(20, 0);
		Hardware.intakeRight.configPeakCurrentLimit(20, 0);
		Hardware.intakeLeft.configPeakCurrentDuration(10, 0);
		Hardware.intakeRight.configPeakCurrentDuration(10, 0);
		Hardware.intakeLeft.enableCurrentLimit(true);
		Hardware.intakeRight.enableCurrentLimit(true);
	}


	public void extendIntake() {
		Hardware.intakeExtendSolenoid.set(Value.kForward);
	}

	public void retractIntake() {
		Hardware.intakeExtendSolenoid.set(Value.kReverse);
	}

	public void intakeDown() {
		Hardware.intakeStowLeft.set(Value.kReverse);
		Hardware.intakeStowRight.set(Value.kReverse);
	}

	public void stowIntake() {
		Hardware.intakeStowLeft.set(Value.kForward);
		Hardware.intakeStowRight.set(Value.kForward);
	}

	public void stallIntake() {
		Hardware.intakeLeft.set(ControlMode.PercentOutput, 0);
		Hardware.intakeRight.set(ControlMode.PercentOutput, 0);
		Robot.carriage.intakeCube(1.0/12.0);
	}

	public void intakeCube (boolean wide) {
		//	intakeDown();
		if(wide) {
			extendIntake();
		} else {
			retractIntake();
		}
		Hardware.intakeLeft.set(ControlMode.PercentOutput, 0.75);
		Hardware.intakeRight.set(ControlMode.PercentOutput, -0.75);
		Robot.carriage.intakeCube(0.90);
	}

//	public void setOuttakeSpeed (double motorSpeed) {
//		this.outtakeSpeed = motorSpeed;
//	}
	
	
	public void outtakeCube () {
		// if (intake.get() != Value.kReverse) //check if already extended
		// extendIntake();
		Hardware.intakeLeft.set(ControlMode.PercentOutput, -0.65);
		Hardware.intakeRight.set(ControlMode.PercentOutput, 0.65);
		Robot.carriage.outtakeCube(-1);
//		Robot.carriage.outtakeCube(outtakeSpeed);
	}

	public void stop()  {
		Hardware.intakeLeft.set(ControlMode.PercentOutput, 0);
		Hardware.intakeRight.set(ControlMode.PercentOutput, 0);
		//	stowIntake();
		//	retractIntake();
		Robot.carriage.stop();
	}

	public void initDefaultCommand()  {
		//	setDefaultCommand(new IntakeCommand());
	}

}