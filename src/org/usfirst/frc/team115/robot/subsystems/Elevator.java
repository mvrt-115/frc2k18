package org.usfirst.frc.team115.robot.subsystems;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Hardware;
import org.usfirst.frc.team115.robot.Robot;

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

		Hardware.elevatorLeft.set(ControlMode.Follower, Hardware.elevatorRight.getDeviceID());
		Hardware.elevatorRight.setInverted(true);
		Hardware.elevatorLeft.setInverted(true);
//		Hardware.elevatorRight.setInverted(true);

		/* First choose the sensor. */
		Hardware.elevatorRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		Hardware.elevatorRight.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate. */
		Hardware.elevatorRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		//		Hardware.elevatorLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
		//		Hardware.elevatorLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);

		if (limitVoltage) {
			Hardware.elevatorRight.configPeakOutputForward(0.3, Constants.kTimeoutMs);
			Hardware.elevatorRight.configPeakOutputReverse(-0.3, Constants.kTimeoutMs);
//			Hardware.elevatorRight.configNominalOutputForward(0.3 / 12, 0);
//			Hardware.elevatorRight.configNominalOutputReverse(-0.3 / 12, 0);
		}
		else {
			Hardware.elevatorRight.configPeakOutputForward(0.8, Constants.kTimeoutMs);
			Hardware.elevatorRight.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		}

		/* set closed loop gains in slot0 - see documentation */
		Hardware.elevatorRight.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
//		Hardware.elevatorRight.configPeakOutputForward(1, Constants.kTimeoutMs);
//		Hardware.elevatorRight.configPeakOutputReverse(-0.5, Constants.kTimeoutMs);
//		Hardware.elevatorRight.configNominalOutputForward(0.6/12.0, 0);
//		Hardware.elevatorRight.configNominalOutputReverse(-0.6/12.0, 0);
		//		Hardware.elevatorLeft.config_kF(0, Constants.kElevatorF, Constants.kTimeoutMs);
		Hardware.elevatorRight.config_kP(0, Constants.kElevatorP, Constants.kTimeoutMs);
		//		Hardware.elevatorLeft.config_kI(0, Constants.kElevatorI, Constants.kTimeoutMs);
		Hardware.elevatorRight.config_kD(0, Constants.kElevatorD, Constants.kTimeoutMs);

		Hardware.elevatorRight.configAllowableClosedloopError(Constants.kSlotIdx, (int)(convertInchesToTicks(1.1)), Constants.kTimeoutMs);

		/* set acceleration and vcruise velocity - see documentation */
		Hardware.elevatorRight.configMotionCruiseVelocity(10000, Constants.kTimeoutMs);
		Hardware.elevatorRight.configMotionAcceleration(6000, Constants.kTimeoutMs);

		/* zero the sensor */
//		Hardware.elevatorRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		Hardware.elevatorRight.configForwardSoftLimitThreshold((int)(convertMetersToTicks(1.6)), 0);
		Hardware.elevatorRight.configForwardSoftLimitEnable(true, 0);
	}

	public double handleDeadband(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	public void manualElevate(double throttle) {
		throttle *= -1;
		
		if(getTopLimit() && throttle >= 0) {
			throttle = 1.0/12;
		}
		
		if(Robot.oi.getHoldPosition()) {
			throttle = 1.0 / 12;
		}
		//throttle = handleDeadband(throttle, 0.1);
		SmartDashboard.putNumber("Applied Voltage", throttle * 12.0);
		SmartDashboard.putNumber("Applied Current",  Hardware.elevatorRight.getOutputCurrent());
		SmartDashboard.putNumber("Hardware.elevatorRight Encoder Reading", Hardware.elevatorRight.getSelectedSensorPosition(0));
		//	SmartDashboard.putNumber("Hardware.elevatorRight Encoder Reading", Hardware.elevatorLeft.getSensorCollection().getQuadraturePosition());
		//		if (Robot.oi.getManualMode())
		Hardware.elevatorRight.set(ControlMode.PercentOutput, throttle);
	}

	double setpoint = 0.0;
	public void setElevatorSetpoint(double height) { //meters
		setpoint = convertMetersToTicks(height);
		Hardware.elevatorRight.set(ControlMode.Position, setpoint);
	}

	public void log() {
		SmartDashboard.putNumber("Applied Current",  Hardware.elevatorRight.getOutputCurrent());
		SmartDashboard.putNumber("Applied Voltage",  Hardware.elevatorRight.getMotorOutputVoltage());
		SmartDashboard.putNumber("ClosedLoopError", Hardware.elevatorRight.getClosedLoopError(0));
		SmartDashboard.putNumber("Current Elevator Position", convertTicksToInches(Hardware.elevatorRight.getSelectedSensorPosition(0) * 0.0254));
		SmartDashboard.putNumber("Elevator Setpoint Ticks", setpoint);
		SmartDashboard.putNumber("Elevator Error", convertTicksToInches(setpoint - Hardware.elevatorRight.getSelectedSensorPosition(0) * 0.0254));
		SmartDashboard.putBoolean("Bottom Hall Effect", getBottomLimit());
		SmartDashboard.putBoolean("Top Hall Effect", getTopLimit());
	}

	public double getError() {
		return Math.abs(Hardware.elevatorRight.getSelectedSensorPosition(0) - setpoint);
	}

	public double convertTicksToInches(double ticks) {
		return (ticks/4096) * 2 * 0.75636 * Math.PI;
	}

	public double convertMetersToTicks(double goal) {
		return ((goal / 0.0254) * (1 / (2 * 0.75636 * Math.PI)) * 4096); 
	}
	
	public double convertInchesToTicks(double inches) {
		return (inches / (2 * 0.75636 * Math.PI) * 4096);
	}

	public void zero() {
		//if zero hallEffect triggered, set encoder 
		if (getBottomLimit()) {
			stop();
			Hardware.elevatorRight.setSelectedSensorPosition(0, 0, 0);
		} else {
			Hardware.elevatorRight.set(ControlMode.PercentOutput, 0.03);
		}
	}

	public boolean getBottomLimit() {
		return !(Hardware.bottomHallEffect.get()); //false = magnet detected
	}	
	
	public boolean getTopLimit() {
		return !(Hardware.topHallEffect.get()); //false = magnet detected
	}

	public void hold() {
		Hardware.elevatorRight.set(ControlMode.PercentOutput, 1.0 / 12);
	}

	public void stop() {
		Hardware.elevatorRight.set(ControlMode.PercentOutput, 0);
	}
	protected void initDefaultCommand() {
		//		setDefaultCommand(new ManualElevate());
	}

}
