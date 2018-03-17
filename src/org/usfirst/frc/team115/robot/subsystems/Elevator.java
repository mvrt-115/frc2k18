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
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends Subsystem {

	public static final double kHoldVoltage = 1.0/12.0;

	boolean limitVoltage = false;

	public enum ElevatorState {
		DISABLED,
		ZEROING,
		SETPOINT, 
		HOLD,
		MANUAL
	};

	public ElevatorState currState = ElevatorState.DISABLED;
	public boolean enabled = false;
	public double offset = 0.0;
	public double goal = 0.0;

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
		new Notifier(new Runnable(){

			@Override
			public void run() {
				loop();				
			}

		}).startPeriodic(0.02);
	}

	/*** Elevator Movement Methods ***/

	public double handleDeadband(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	public void manualElevate(double throttle) {
		throttle *= -1;

		if((getTopLimit() && throttle >= 0) || Robot.oi.getHoldPosition()) {
			throttle = kHoldVoltage;
		}

		SmartDashboard.putNumber("Applied Voltage", throttle * 12.0);
		SmartDashboard.putNumber("Applied Current",  Hardware.elevatorRight.getOutputCurrent());
		SmartDashboard.putNumber("Hardware.elevatorRight Encoder Reading", Hardware.elevatorRight.getSelectedSensorPosition(0));
		Hardware.elevatorRight.set(ControlMode.PercentOutput, throttle);
	}

	public void hold() {
		Hardware.elevatorRight.set(ControlMode.PercentOutput, kHoldVoltage);
	}

	/*** PID Methods ***/

	public void setElevatorSetpoint(double height) { //meters
		this.goal = UnitConverter.convertElevatorFeetToTicks(height);
		updateState(ElevatorState.SETPOINT);
	}


	/*** Get Methods ***/

	public double getError() {
		return Math.abs(Hardware.elevatorRight.getSelectedSensorPosition(0) - goal);
	}

	public boolean getBottomLimit() {
		return !(Hardware.bottomHallEffect.get()); //false = magnet detected
	}	

	public boolean getTopLimit() {
		return !(Hardware.topHallEffect.get()); //false = magnet detected
	}

	public boolean goalReached() {
		return getError() <= UnitConverter.convertInchesToTicks(1.0);
	}

	/*** Zeroing and Logging ***/

	public void stop() {
		Hardware.elevatorRight.set(ControlMode.PercentOutput, 0);
	}


	public void updateState(ElevatorState state) {
		currState = state;
	}

	public void enable(boolean enable) {
		enabled = enable;
	}

	//Updated every 20ms
	public void loop() {
		switch(currState) {
		case DISABLED:
			if(enabled) {
				updateState(ElevatorState.SETPOINT);
			}
			break;
		case ZEROING:
			if(!enabled) {
				updateState(ElevatorState.DISABLED);
			} else if(getBottomLimit()) {
				stop();
				Hardware.elevatorRight.setSelectedSensorPosition(0, 0, 0);
				enable(false);
				//					Robot.oi.rumbleOperator();
			} else {
				Hardware.elevatorRight.set(ControlMode.PercentOutput, 0.5/12.0);
			}
			break;
		case SETPOINT:
			if(goal > Constants.kMaxElevatorHeight) {
				goal = Constants.kMaxElevatorHeight;
			} else if(goal < Constants.kMinElevatorHeight) {
				goal = Constants.kMinElevatorHeight;
			}

			if(!enabled) {
				updateState(ElevatorState.DISABLED);
			} else {
				if(getError() <= UnitConverter.convertInchesToTicks(1.0)) {
					updateState(ElevatorState.HOLD);
				} else {
					if(getTopLimit() && goal >= Hardware.elevatorRight.getSelectedSensorPosition(0)) {
						updateState(ElevatorState.HOLD);
					} else if(getBottomLimit() && goal <= Hardware.elevatorRight.getSelectedSensorPosition(0)) {
						stop();
					} else {
						Hardware.elevatorRight.set(ControlMode.Position, goal);
					}
				}
			}
			break;
		case HOLD:
			if(!enabled) {
				updateState(ElevatorState.DISABLED);
			} else {
				hold();
			}
		case MANUAL:
			manualElevate(Robot.oi.getManualElevate());
			break;
		}
	}


	public void log() {
		SmartDashboard.putNumber("Applied Current",  Hardware.elevatorRight.getOutputCurrent());
		SmartDashboard.putNumber("Applied Voltage",  Hardware.elevatorRight.getMotorOutputVoltage());
		SmartDashboard.putNumber("ClosedLoopError", Hardware.elevatorRight.getClosedLoopError(0));
		SmartDashboard.putNumber("Current Elevator Position Feet", UnitConverter.convertElevatorTicksToFeet(Hardware.elevatorRight.getSelectedSensorPosition(0) * 0.0254));
		SmartDashboard.putNumber("Elevator Setpoint Ticks", goal);
		SmartDashboard.putNumber("Elevator Error Inches", UnitConverter.convertTicksToInches(goal - Hardware.elevatorRight.getSelectedSensorPosition(0) * 0.0254));
		SmartDashboard.putBoolean("Bottom Hall Effect", getBottomLimit());
		SmartDashboard.putBoolean("Top Hall Effect", getTopLimit());
		SmartDashboard.putBoolean("Enabled", enabled);
		SmartDashboard.putString("Elevator State", currState.toString());
	}

	/*** Other ***/
	protected void initDefaultCommand() {
	}
}
