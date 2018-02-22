package org.usfirst.frc.team115.robot.subsystems;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Hardware;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Carriage extends Subsystem {
	
	public Carriage()  {
		Hardware.carriageLeft = new TalonSRX(Constants.kCarriageLeftTalonID);	
		Hardware.carriageRight = new TalonSRX(Constants.kCarriageRightTalonID);
		Hardware.carriageRight.set(ControlMode.Follower, Hardware.carriageLeft.getDeviceID());
		Hardware.carriageRight.setInverted(true);
		Hardware.carriageBreakbeam = new DigitalInput(2);
	}
	
	public void intakeCube (double motorSpeed) {
		Hardware.carriageLeft.set(ControlMode.PercentOutput, motorSpeed);
	}
	
	public void outtakeCube (double motorSpeed) { //motorSpeed should be negative
		Hardware.carriageLeft.set(ControlMode.PercentOutput, motorSpeed);
	}
	
	public boolean cubeDetected() {
		return !(Hardware.carriageBreakbeam.get()); 
	}
	
	public void log() {
		SmartDashboard.putBoolean("Carriage Breakbeam", cubeDetected());
	}
	
	public void stop()  {
		Hardware.carriageLeft.set(ControlMode.PercentOutput, 0.0);
	}
	protected void initDefaultCommand() {
//		setDefaultCommand(new CarriageCommand());
	}

}