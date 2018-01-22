package org.usfirst.frc.team115.robot.subsystems;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.commands.CarriageCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Carriage extends Subsystem {
	
	private DoubleSolenoid carriage;
	
	public Carriage()  {
		carriage = new DoubleSolenoid(1, Constants.kCarriagePortA, Constants.kCarriagePortB);
	}
	
	public void extend() {
		carriage.set(Value.kForward);
	}
	
	public void retract() {
		carriage.set(Value.kReverse);
	}
	
	public void intakeCube (double leftSpeed, double rightSpeed) {
	
	}
	
	public void outtakeCube (double motorSpeed) {
		
	}
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new CarriageCommand());
	}

}
