package org.usfirst.frc.team115.robot.subsystems;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.commands.ManualElevate;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Subsystem {

	TalonSRX left, right;
	boolean limitCurrent = false;
	
	public Elevator() {
		left = new TalonSRX(3);
		right = new TalonSRX(9);
		right.set(ControlMode.Follower, left.getDeviceID());
		right.setInverted(true);
		
		/* first choose the sensor */
		left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		left.setSensorPhase(true);
		left.setInverted(false);
		
		/* Set relevant frame periods to be at least as fast as periodic rate*/
		left.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		left.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
	
		/* set the peak and nominal outputs */
		left.configNominalOutputForward(0, Constants.kTimeoutMs);
		left.configNominalOutputReverse(0, Constants.kTimeoutMs);
		
		if (limitCurrent) {
			left.configPeakOutputForward(0.3, Constants.kTimeoutMs);
			left.configPeakOutputReverse(-0.3, Constants.kTimeoutMs);
		}
		else {
			left.configPeakOutputForward(1, Constants.kTimeoutMs);
			left.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		}
		
		/* set closed loop gains in slot0 - see documentation */
		left.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		left.config_kF(0, Constants.kElevatorF, Constants.kTimeoutMs);
		left.config_kP(0, Constants.kElevatorP, Constants.kTimeoutMs);
		left.config_kI(0, Constants.kElevatorI, Constants.kTimeoutMs);
		left.config_kD(0, Constants.kElevatorD, Constants.kTimeoutMs);
	
		/* set acceleration and vcruise velocity - see documentation */
		left.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		left.configMotionAcceleration(6000, Constants.kTimeoutMs);
		
		/* zero the sensor */
		left.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}
	
	public double handleDeadband(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}
	
	public void manualElevate(double throttle) {
		throttle *= -1.0;
		throttle = handleDeadband(throttle, 0.1);
		SmartDashboard.putNumber("Applied Voltage", throttle * 12.0);
		SmartDashboard.putNumber("Applied Current",  left.getOutputCurrent());
		SmartDashboard.putNumber("Left Encoder Reading", left.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("Right Encoder Reading", left.getSensorCollection().getQuadraturePosition());
		left.set(ControlMode.PercentOutput, throttle);
	}
	
	public void setElevatorSetpoint(double height) { //meters
		double targetPos = height * 39.3701 / (2.0 * 2.0 * Math.PI) * 4096; //pulley radius yet to be determined
		left.set(ControlMode.MotionMagic, targetPos); 
	}
	
	public void zero() {
		setElevatorSetpoint(0.0);
	}
	
	public void stop() {
		left.set(ControlMode.PercentOutput, 0);
	}
	protected void initDefaultCommand() {
		setDefaultCommand(new ManualElevate());
	}

}
