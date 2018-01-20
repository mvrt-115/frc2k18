package org.usfirst.frc.team115.robot.subsystems;

import org.usfirst.frc.team115.robot.commands.IntakeCommand;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem{
	
	private TalonSRX leftIntake;
	private TalonSRX rightIntake;
	
	Carriage carriage;
	
	public Intake()  {
		carriage = new Carriage();
		leftIntake = new TalonSRX(16);	//left cantalon port tbd
		rightIntake = new TalonSRX(17);	//right cantalon port tbd
	}
	
	public void intakeCube (double leftSpeed, double rightSpeed) 
	{
		leftIntake.set(ControlMode.PercentOutput, leftSpeed);
		rightIntake.set(ControlMode.PercentOutput, rightSpeed);
		carriage.intakeCube(1, 1);	//values tbd
	}
	
	public void outtakeCube (double motorSpeed) 
	{
		leftIntake.set(ControlMode.Follower, rightIntake.getDeviceID() );
		rightIntake.set(ControlMode.PercentOutput, motorSpeed);
		carriage.outtakeCube(-1);	//values tbd
	}
	
	public void stop()  {
		leftIntake.set(ControlMode.PercentOutput, 0);
		rightIntake.set(ControlMode.PercentOutput, 0);
	}
	public void initDefaultCommand()  {
		setDefaultCommand(new IntakeCommand());
	}
	
}
