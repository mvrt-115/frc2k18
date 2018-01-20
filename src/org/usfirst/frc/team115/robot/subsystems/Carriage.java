package org.usfirst.frc.team115.robot.subsystems;

import org.usfirst.frc.team115.robot.commands.CarriageCommand;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Carriage extends Subsystem {
	
	private TalonSRX leftCarriage;
	private TalonSRX rightCarriage;
	
	public Carriage()  {
		leftCarriage = new TalonSRX(24);
		rightCarriage = new TalonSRX(25);
	}
	
	public void intakeCube (double leftSpeed, double rightSpeed) 
	{
		leftCarriage.set(ControlMode.PercentOutput, leftSpeed);
		rightCarriage.set(ControlMode.PercentOutput, rightSpeed);
	}
	
	public void outtakeCube (double motorSpeed) 
	{
		leftCarriage.set(ControlMode.Follower, rightCarriage.getDeviceID() );
		rightCarriage.set(ControlMode.PercentOutput, motorSpeed);
	}
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new CarriageCommand());
	}

}
