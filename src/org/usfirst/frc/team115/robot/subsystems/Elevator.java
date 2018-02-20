package org.usfirst.frc.team115.robot.subsystems;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.commands.ManualElevate;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Subsystem {

	public TalonSRX left;
	public VictorSPX right;
	boolean limitVoltage = false;
	
	public Elevator() {
		left = new TalonSRX(6); //9 //5
		right = new VictorSPX(7); //10 //8
//		right.set(ControlMode.Follower, left.getDeviceID());
		right.follow(left);
		left.setInverted(false);
		right.setInverted(true);
		
		/* First choose the sensor. */
		left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		left.setSensorPhase(false);
		
		/* Set relevant frame periods to be at least as fast as periodic rate. */
		left.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		left.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
	
		/* Set the peak and nominal outputs */
//		left.configNominalOutputForward(0, Constants.kTimeoutMs);
//		left.configNominalOutputReverse(0, Constants.kTimeoutMs);
		
		if (limitVoltage) {
			left.configPeakOutputForward(0.6, Constants.kTimeoutMs);
			left.configPeakOutputReverse(-0.6, Constants.kTimeoutMs);
			left.configNominalOutputForward(0.5 / 12, 0);
			left.configNominalOutputReverse(-0.5 / 12, 0);
		}
		else {
			left.configPeakOutputForward(0.2, Constants.kTimeoutMs);
			left.configPeakOutputReverse(-0.2, Constants.kTimeoutMs);
		}
		
		/* set closed loop gains in slot0 - see documentation */
		left.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		left.configPeakOutputForward(1, Constants.kTimeoutMs);
		left.configPeakOutputReverse(-0.5, Constants.kTimeoutMs);
		left.configNominalOutputForward(0.6/12.0, 0);
		left.configNominalOutputReverse(-0.6/12.0, 0);
//		left.config_kF(0, Constants.kElevatorF, Constants.kTimeoutMs);
		left.config_kP(0, Constants.kElevatorP, Constants.kTimeoutMs);
//		left.config_kI(0, Constants.kElevatorI, Constants.kTimeoutMs);
		left.config_kD(0, Constants.kElevatorD, Constants.kTimeoutMs);
		
		left.configAllowableClosedloopError(Constants.kSlotIdx, 455, Constants.kTimeoutMs);
		
		/* set acceleration and vcruise velocity - see documentation */
		left.configMotionCruiseVelocity(10000, Constants.kTimeoutMs);
		left.configMotionAcceleration(6000, Constants.kTimeoutMs);
		
		/* zero the sensor */
		left.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		left.configForwardSoftLimitThreshold((int)(convertMetersToTicks(1.5 + 0.0254)), 0);
		left.configForwardSoftLimitEnable(true, 0);
	}
	
	public double handleDeadband(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}
	
	public void manualElevate(double throttle) {
		throttle *= -1.0;
		if(Robot.oi.getHoldPosition()) {
			throttle = 0.5 / 12;
		}
		//throttle = handleDeadband(throttle, 0.1);
		SmartDashboard.putNumber("Applied Voltage", throttle * 12.0);
		SmartDashboard.putNumber("Applied Current",  left.getOutputCurrent());
//		SmartDashboard.putNumber("Left Encoder Reading", left.getSelectedSensorPosition(0));
		left.set(ControlMode.PercentOutput, throttle);
	}
	
	double setpoint = 0.0;
	public void setElevatorSetpoint(double height) { //meters
		setpoint = convertMetersToTicks(height);
		left.set(ControlMode.Position, setpoint);
	}
	
	public void log() {
		SmartDashboard.putNumber("Applied Current",  left.getOutputCurrent());
		SmartDashboard.putNumber("Applied Voltage",  left.getMotorOutputVoltage());
		SmartDashboard.putNumber("Current Elevator Position", convertTicksToInches(left.getSelectedSensorPosition(0)));
		SmartDashboard.putNumber("Elevator Setpoint Ticks", setpoint);
		SmartDashboard.putNumber("Elevator Error", convertTicksToInches(setpoint - left.getSelectedSensorPosition(0)));
	}
	
	public double getError() {
		return Math.abs(left.getSelectedSensorPosition(0) - setpoint);
	}
	
	public double convertTicksToInches(double ticks) {
		return (ticks/4096) * 1.432 * Math.PI;
	}
	
	public double convertMetersToTicks(double goal) {
		return ((goal / 0.0254) * (1 / (1.282 * Math.PI)) * 4096); 
	}
	
	public void zero() {
		left.setSelectedSensorPosition(0, 0, 0);
	}
	
	public void hold() {
		left.set(ControlMode.PercentOutput, 0.6 / 12);
	}
	
	public void stop() {
		left.set(ControlMode.PercentOutput, 0);
	}
	protected void initDefaultCommand() {
		setDefaultCommand(new ManualElevate());
	}

}
