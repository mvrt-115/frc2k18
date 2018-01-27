package org.usfirst.frc.team115.robot.subsystems;

import java.util.function.DoubleFunction;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.commands.CheesyDriveJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem implements PIDOutput {
	TalonSRX frontLeft, backLeft, frontRight, backRight;
	public static PIDController PIDController;
	
	private static final double SENSITIVITY = 0.75;
	private DoubleSolenoid shifter;
	private boolean isHighGear;
	private DoubleFunction<Double> limiter = limiter(-1.0, 1.0);
	
//	private double oldWheel = 0.0;
//	private double negativeInertiaAccumulator = 0.0;
	private double quickStopAccumulator = 0.0;
	private double wheelDeadband = 0.02;
	private double throttleDeadband = 0.02;
	
	
	public DriveTrain() {
		frontLeft = new TalonSRX(0);
		backLeft = new TalonSRX(1);	
		backLeft.set(ControlMode.Follower, frontLeft.getDeviceID());

		frontRight = new TalonSRX(2);
		backRight = new TalonSRX(3);
		backRight.set(ControlMode.Follower, frontRight.getDeviceID());
		
		/* first choose the sensor */
		frontLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		frontLeft.setSensorPhase(true);
		frontLeft.setInverted(false);
		
		frontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		frontRight.setSensorPhase(true);
		frontRight.setInverted(false);
		
		/* Set relevant frame periods to be at least as fast as periodic rate*/
		frontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		frontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
		
		frontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		frontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);


		/* set the peak and nominal outputs */
		frontLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
		frontLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
		frontLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
		frontLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		
		frontRight.configNominalOutputForward(0, Constants.kTimeoutMs);
		frontRight.configNominalOutputReverse(0, Constants.kTimeoutMs);
		frontRight.configPeakOutputForward(1, Constants.kTimeoutMs);
		frontRight.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		
		/* set closed loop gains in slot0 - see documentation */
		frontLeft.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		frontLeft.config_kF(0, Constants.kDriveF, Constants.kTimeoutMs);
		frontLeft.config_kP(0, Constants.kDriveP, Constants.kTimeoutMs);
		frontLeft.config_kI(0, Constants.kDriveI, Constants.kTimeoutMs);
		frontLeft.config_kD(0, Constants.kDriveD, Constants.kTimeoutMs);
		
		frontRight.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		frontRight.config_kF(0, Constants.kDriveF, Constants.kTimeoutMs);
		frontRight.config_kP(0, Constants.kDriveP, Constants.kTimeoutMs);
		frontRight.config_kI(0, Constants.kDriveI, Constants.kTimeoutMs);
		frontRight.config_kD(0, Constants.kDriveD, Constants.kTimeoutMs);
		
		/* set acceleration and vcruise velocity - see documentation */
		frontLeft.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		frontLeft.configMotionAcceleration(6000, Constants.kTimeoutMs);
		
		frontRight.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		frontRight.configMotionAcceleration(6000, Constants.kTimeoutMs);
		
		/* zero the sensor */
		frontLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		frontRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		
	}

	public void drive(double throttle, double wheel, boolean quickturn) {
		
		wheel = handleDeadband(wheel, wheelDeadband);
		throttle = handleDeadband(throttle, throttleDeadband);
		double overPower;
		double angularPower;

		wheel = dampen(wheel, 0.5);
		wheel = dampen(wheel, 0.5);
		wheel = dampen(wheel, 0.5);

		if (quickturn) {
			if (Math.abs(throttle) < 0.2) {
				double alpha = 0.1;
				quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * limiter.apply(wheel) * 2;
			}
			overPower = 1.0;
			angularPower = wheel;
		} else {
			overPower = 0.0;
			angularPower = Math.abs(throttle) * wheel * SENSITIVITY - quickStopAccumulator;
			if (quickStopAccumulator > 1) {
				quickStopAccumulator -= 1;
			} else if (quickStopAccumulator < -1) {
				quickStopAccumulator += 1;
			} else {
				quickStopAccumulator = 0.0;
			}
		}

		double rightPwm = throttle - angularPower;
		double leftPwm = throttle + angularPower;
		if (leftPwm > 1.0) {
			rightPwm -= overPower * (leftPwm - 1.0);
			leftPwm = 1.0;
		} else if (rightPwm > 1.0) {
			leftPwm -= overPower * (rightPwm - 1.0);
			rightPwm = 1.0;
		} else if (leftPwm < -1.0) {
			rightPwm += overPower * (-1.0 - leftPwm);
			leftPwm = -1.0;
		} else if (rightPwm < -1.0) {
			leftPwm += overPower * (-1.0 - rightPwm);
			rightPwm = -1.0;
		}

		setLeftRightMotorOutputs(leftPwm, rightPwm);
	}

	public void stop() {
		setLeftRightMotorOutputs(0.0,  0.0);
	}

	public void setLeftRightMotorOutputs (double left, double right)  {
		frontRight.set(ControlMode.PercentOutput, right);
		frontLeft.set(ControlMode.PercentOutput, left);
	}

	public void shift() {
		// TODO Add code for shifting from low -> high or vice versa
		if(isHighGear) {
			shifter.set(Value.kReverse);
		}
		else {
			shifter.set(Value.kForward); 	
		}	
		isHighGear = !isHighGear;
	}

	private DoubleFunction<Double> limiter(double minimum, double maximum) {
		if (maximum < minimum) {
			throw new IllegalArgumentException("The minimum value cannot exceed the maximum value");
		}

		return (double value) -> {
			if (value > maximum) {
				return maximum;
			}
			if (value < minimum) {
				return minimum;
			}
			return value;
		};
	}

	public double handleDeadband(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	private static double dampen(double wheel, double wheelNonLinearity) {
		double factor = Math.PI * wheelNonLinearity;
		return Math.sin(factor * wheel) / Math.sin(factor);
	} 
	
	public void setDistanceSetpoint(double dist) {
		double targetPos = dist * 12.0 / (4.0 * Math.PI) * 4096;
		frontLeft.set(ControlMode.MotionMagic, targetPos); 
		frontRight.set(ControlMode.MotionMagic, targetPos);
	}

	public void setTurnSetpoint(double angle) {
		PIDController.enable();
		PIDController.setSetpoint(angle);
	}
	
	protected void initDefaultCommand() {
		setDefaultCommand(new CheesyDriveJoystick());
	}

	@Override
	public void pidWrite(double output) {
		setLeftRightMotorOutputs(-output, output);
	}
}
