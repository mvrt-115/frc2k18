package org.usfirst.frc.team115.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Hardware {

	public static TalonSRX driveFrontLeft;
	public static TalonSRX driveFrontRight;
	public static TalonSRX driveBackLeft;
	public static TalonSRX driveBackRight;
	
	public static TalonSRX carriageLeft;
	public static TalonSRX carriageRight;
	
	public static TalonSRX elevatorLeft;
	public static TalonSRX elevatorRight;
	
	public static TalonSRX intakeLeft;
	public static TalonSRX intakeRight;
	
	public static DoubleSolenoid intakeRightSolenoid;
	public static DoubleSolenoid intakeExtendSolenoid;
	public static DoubleSolenoid intakeStowLeft;
	public static DoubleSolenoid intakeStowRight;
	
	
	public static DigitalInput carriageBreakbeam;
	
	public static DigitalInput bottomHallEffect;
	public static DigitalInput topHallEffect;
	
	
}
