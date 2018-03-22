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
	public boolean zeroed = false;

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
			Hardware.elevatorRight.configPeakOutputForward(0.95, Constants.kTimeoutMs);
			Hardware.elevatorRight.configPeakOutputReverse(-0.8, Constants.kTimeoutMs);
		}

		/* set closed loop gains in slot0 - see documentation */
		Hardware.elevatorRight.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		//		Hardware.elevatorLeft.config_kF(0, Constants.kElevatorF, Constants.kTimeoutMs);
		Hardware.elevatorRight.config_kP(0, Constants.kElevatorP, Constants.kTimeoutMs);
		Hardware.elevatorRight.config_kD(0, Constants.kElevatorD, Constants.kTimeoutMs);

		//		Hardware.elevatorLeft.config_kF(0, Constants.kElevatorF, Constants.kTimeoutMs);
		Hardware.elevatorRight.config_kP(1, Constants.kElevatorP, Constants.kTimeoutMs);
		Hardware.elevatorRight.config_kD(1, Constants.kElevatorD, Constants.kTimeoutMs);

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

		}).startPeriodic(0.005);
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
		//		Hardware.elevatorRight.selectProfileSlot(1, 1);
		//		Hardware.elevatorRight.set(ControlMode.Position, goal);
		//		Hardware.elevatorRight.set(ControlMode.PercentOutput, kHoldVoltage);
	}

	/*** PID Methods ***/

	public void setElevatorSetpoint(double height) { //meters
		this.goal = UnitConverter.convertElevatorFeetToTicks(height);
		System.out.println("Set Goal Value: " + this.goal);
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

	

	//Updated every 5ms
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
			} else if(getBottomLimit() && !zeroed) {
				stop();
				Hardware.elevatorRight.setSelectedSensorPosition(0, 0, 0);
				for(int i = 0; i < 100; i++)
					Robot.oi.rumbleJoystick();
				zeroed = true;
			} else if(zeroed) {
				Hardware.elevatorRight.setSelectedSensorPosition(0, 0, 0);
				Robot.oi.stopRumble();
			} else {
				if (Hardware.elevatorRight.getSelectedSensorPosition(0) > UnitConverter.convertInchesToTicks(3.0)) {
					goal = UnitConverter.convertInchesToTicks(3.0);
					if (getError() <= UnitConverter.convertInchesToTicks(2.0))
						stop();
					else
						Hardware.elevatorRight.set(ControlMode.Position, goal);
				}
				else
					Hardware.elevatorRight.set(ControlMode.PercentOutput, -2.0/12.0);
//				Hardware.elevatorRight.set(ControlMode.PercentOutput, -3.0/12.0);
//				goal = UnitConverter.convertInchesToTicks(1.0);
//				if(getError() <= UnitConverter.convertInchesToTicks(3.0)) {
//					stop();
//				} else {
//					Hardware.elevatorRight.set(ControlMode.Position, goal);
//				}
			}
			break;
		case SETPOINT:
			zeroed = false;
			double maxHeight = UnitConverter.convertElevatorFeetToTicks(Constants.kMaxElevatorHeight);
			double minHeight = UnitConverter.convertElevatorFeetToTicks(Constants.kMinElevatorHeight);
			
			goal = Math.min(goal, maxHeight);
			goal = Math.max(goal, minHeight);

			Hardware.elevatorRight.selectProfileSlot(0, 0);

			if(!enabled) {
				updateState(ElevatorState.DISABLED);
			} else {
				if(getError() <= UnitConverter.convertInchesToTicks(1.0)) {
					System.out.println("Found goal! " + goal + ", " + getError());
					//					updateState(ElevatorState.HOLD);
				} else {
					if(getTopLimit() && goal >= Hardware.elevatorRight.getSelectedSensorPosition(0)) {
						System.out.println("Holding at topLimit");
						Hardware.elevatorRight.set(ControlMode.Position, Hardware.elevatorRight.getSelectedSensorPosition(0));
						//						updateState(ElevatorState.HOLD);
					} else if(getBottomLimit() && goal <= Hardware.elevatorRight.getSelectedSensorPosition(0)) {
						System.out.println("Stopping at bottomLimit");
						stop();
					} else {
						System.out.println("PID ControlMode.Position...");
						Hardware.elevatorRight.set(ControlMode.Position, goal);
					}
				}
			}
			break;
		case HOLD:
			if(!enabled) {
				updateState(ElevatorState.DISABLED);
			} else {
				if(getError() <= UnitConverter.convertInchesToTicks(1.0)) {
					hold();
				} else {
					updateState(ElevatorState.SETPOINT);
				}
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
		SmartDashboard.putNumber("Current Elevator Position Feet", UnitConverter.convertElevatorTicksToFeet(Hardware.elevatorRight.getSelectedSensorPosition(0)));
		SmartDashboard.putNumber("Current Elevator Position Ticks", (Hardware.elevatorRight.getSelectedSensorPosition(0)));
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
