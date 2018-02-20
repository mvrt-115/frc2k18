package org.usfirst.frc.team115.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Hardware {

	//DRIVE
	public static TalonSRX driveFrontLeft;
	public static TalonSRX driveFrontRight;
	public static TalonSRX driveBackLeft;
	public static TalonSRX driveBackRight;
	public static DoubleSolenoid shifter;
	public static AHRS navX;
	
	//CARRIAGE
	public static TalonSRX carriageLeft;
	public static TalonSRX carriageRight;
	
	
	//ELEVATOR
	public static TalonSRX elevatorLeft;
	public static TalonSRX elevatorRight;
	
	
	//INTAKE
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
