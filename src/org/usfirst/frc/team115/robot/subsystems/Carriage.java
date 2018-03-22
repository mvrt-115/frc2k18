package org.usfirst.frc.team115.robot.subsystems;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Carriage extends Subsystem {
	
	public Carriage()  {
		Hardware.carriageLeft = new TalonSRX(Constants.kCarriageLeftTalonID);	
		Hardware.carriageRight = new TalonSRX(Constants.kCarriageRightTalonID);
		Hardware.carriageRight.set(ControlMode.Follower, Hardware.carriageLeft.getDeviceID());
		Hardware.carriageRight.setInverted(true);
		Hardware.carriageClamp = new DoubleSolenoid(0, 3, 4);
		Hardware.carriageBreakbeam = new DigitalInput(2);
	}
	
	public void intakeCube (double motorSpeed) {
		if (isClamped())
			unclampCube();
		Hardware.carriageLeft.set(ControlMode.PercentOutput, motorSpeed);
	}
	
	public void outtakeCube (double motorSpeed) {
		if (isClamped())
			unclampCube();
		Hardware.carriageLeft.set(ControlMode.PercentOutput, motorSpeed);
	}
	
	public boolean isClamped() {
		return (Hardware.carriageClamp.get() == Value.kForward);
	}
	
	public void clampCube() {
		Hardware.carriageClamp.set(Value.kForward);
	}
	
	public void unclampCube() {
		Hardware.carriageClamp.set(Value.kReverse);
	}
	
	public boolean cubeDetected() {
		return !(Hardware.carriageBreakbeam.get()); 
	}
	
	public void log() {
		SmartDashboard.putBoolean("Carriage Breakbeam", cubeDetected());
	}
	
	public void stop()  {
		if (cubeDetected())
			clampCube();
		Hardware.carriageLeft.set(ControlMode.PercentOutput, 0.0);
		
	}
	protected void initDefaultCommand() {}

}