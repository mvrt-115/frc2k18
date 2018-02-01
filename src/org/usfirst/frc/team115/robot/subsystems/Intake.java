package org.usfirst.frc.team115.robot.subsystems;


import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.commands.IntakeCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem {
	
	private DoubleSolenoid intake;
	private TalonSRX left;
	private TalonSRX right;
//	public DigitalInput breakbeam;

	public Intake()  {
		left = new TalonSRX(16);	//left cantalon port tbd
		right = new TalonSRX(17);	//right cantalon port tbd
		right.set(ControlMode.Follower, left.getDeviceID());
//		breakbeam = new DigitalInput(2);
		intake = new DoubleSolenoid(1, 3, 4);
	}
	
	public void extendIntake() {
		intake.set(Value.kReverse);
	}

	public void retractIntake() {
		intake.set(Value.kForward);	
	}
	
	public void intakeCube () {
		if (intake.get() != Value.kReverse) //check if already extended
			extendIntake();
		left.set(ControlMode.PercentOutput, 1);
		Robot.carriage.intakeCube(1);
	}
	
	public void outtakeCube () {
		if (intake.get() != Value.kReverse) //check if already extended
			extendIntake();
		left.set(ControlMode.PercentOutput, 1);
		Robot.carriage.outtakeCube(-1);
	}
	
	public void stop()  {
		left.set(ControlMode.PercentOutput, 0);
	}
	
	public void initDefaultCommand()  {
		setDefaultCommand(new IntakeCommand());
	}
	
}