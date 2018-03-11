package org.usfirst.frc.team115.robot.subsystems;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Hardware;
import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.UnitConverter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Subsystem {

	boolean limitVoltage = false;

	public Elevator() {
		Hardware.elevatorLeft = new TalonSRX(Constants.kElevatorLeftTalonID);
		Hardware.elevatorRight = new TalonSRX(Constants.kElevatorRightTalonID);

		Hardware.bottomHallEffect = new DigitalInput(0);
		Hardware.topHallEffect = new DigitalInput(1);

//		Hardware.elevatorLeft.set(ControlMode.Follower, Hardware.elevatorRight.getDeviceID());
		Hardware.elevatorLeft.follow(Hardware.elevatorRight);
		Hardware.elevatorRight.setInverted(false);
//		Hardware.elevatorLeft.setInverted(false);
		//Hardware.elevatorRight.setInverted(true);

		/* First choose the sensor. */
		Hardware.elevatorRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		Hardware.elevatorRight.setSensorPhase(false);

		/* Set relevant frame periods to be at least as fast as periodic rate. */
		Hardware.elevatorRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		Hardware.elevatorLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);


		if (limitVoltage) {
			Hardware.elevatorRight.configPeakOutputForward(0.4, Constants.kTimeoutMs);
			Hardware.elevatorRight.configPeakOutputReverse(-0.4, Constants.kTimeoutMs);
			Hardware.elevatorLeft.configPeakOutputForward(0.4, Constants.kTimeoutMs);
			Hardware.elevatorLeft.configPeakOutputReverse(-0.4, Constants.kTimeoutMs);
		}
		else {
			Hardware.elevatorRight.configPeakOutputForward(0.8, Constants.kTimeoutMs);
			Hardware.elevatorRight.configPeakOutputReverse(-0.8, Constants.kTimeoutMs);
		}

		/* set closed loop gains in slot0 - see documentation */
		Hardware.elevatorRight.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		//		Hardware.elevatorLeft.config_kF(0, Constants.kElevatorF, Constants.kTimeoutMs);
		Hardware.elevatorRight.config_kP(0, Constants.kElevatorP, Constants.kTimeoutMs);
		Hardware.elevatorRight.config_kD(0, Constants.kElevatorD, Constants.kTimeoutMs);

		Hardware.elevatorRight.configAllowableClosedloopError(Constants.kSlotIdx, (int)(UnitConverter.convertInchesToTicks(1.1)), Constants.kTimeoutMs);

		/* set acceleration and vcruise velocity - see documentation */
		Hardware.elevatorRight.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		Hardware.elevatorRight.configMotionAcceleration(6000, Constants.kTimeoutMs);

		/* zero the sensor */
		Hardware.elevatorRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		Hardware.elevatorRight.configForwardSoftLimitThreshold((int)(UnitConverter.convertMetersToTicks(1.6)), 0);
		Hardware.elevatorRight.configForwardSoftLimitEnable(true, 0);
	}

	/*** Elevator Movement Methods ***/

	public double handleDeadband(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	public void manualElevate(double throttle) {
		throttle *= -1;

		if((getTopLimit() && throttle >= 0) || Robot.oi.getHoldPosition()) {
			throttle = 1.0/12;
		}
		
		SmartDashboard.putNumber("Applied Voltage", throttle * 12.0);
		SmartDashboard.putNumber("Applied Current",  Hardware.elevatorRight.getOutputCurrent());
		SmartDashboard.putNumber("Hardware.elevatorRight Encoder Reading", Hardware.elevatorRight.getSelectedSensorPosition(0));
		Hardware.elevatorRight.set(ControlMode.PercentOutput, throttle);
	}

	public void hold() {
		Hardware.elevatorRight.set(ControlMode.PercentOutput, 1.0 / 12);
	}

	/*** PID Methods ***/

	double setpoint = 0.0;
	public void setElevatorSetpoint(double height) { //meters
		setpoint = UnitConverter.convertMetersToTicks(height);
		Hardware.elevatorRight.set(ControlMode.Position, setpoint);
//		Hardware.elevatorRight.set(ControlMode.Position, setpoint);
	}


	/*** Get Methods ***/

	public double getError() {
		return Math.abs(Hardware.elevatorRight.getSelectedSensorPosition(0) - setpoint);
	}

	public boolean getBottomLimit() {
		return !(Hardware.bottomHallEffect.get()); //false = magnet detected
	}	

	public boolean getTopLimit() {
		return !(Hardware.topHallEffect.get()); //false = magnet detected
	}

	/*** Zeroing and Logging ***/

	public void stop() {
		Hardware.elevatorRight.set(ControlMode.PercentOutput, 0);
	}
	
	public void log() {
		SmartDashboard.putNumber("Applied Current",  Hardware.elevatorRight.getOutputCurrent());
		SmartDashboard.putNumber("Applied Voltage",  Hardware.elevatorRight.getMotorOutputVoltage());
		SmartDashboard.putNumber("ClosedLoopError", Hardware.elevatorRight.getClosedLoopError(0));
		SmartDashboard.putNumber("Current Elevator Position", UnitConverter.convertTicksToInches(Hardware.elevatorRight.getSelectedSensorPosition(0) * 0.0254));
		SmartDashboard.putNumber("Elevator Setpoint Ticks", setpoint);
		SmartDashboard.putNumber("Elevator Error", UnitConverter.convertTicksToInches(setpoint - Hardware.elevatorRight.getSelectedSensorPosition(0) * 0.0254));
		SmartDashboard.putBoolean("Bottom Hall Effect", getBottomLimit());
		SmartDashboard.putBoolean("Top Hall Effect", getTopLimit());
	}

	public void zero() {
		if (getBottomLimit()) {
			stop();
			Hardware.elevatorRight.setSelectedSensorPosition(0, 0, 0);
		} else {
			Hardware.elevatorRight.set(ControlMode.PercentOutput, 0.03);
		}
	}

	/*** Other ***/
	protected void initDefaultCommand() {
	}
}
