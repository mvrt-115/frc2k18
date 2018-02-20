package org.usfirst.frc.team115.robot.subsystems;

import java.util.function.DoubleFunction;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Hardware;
import org.usfirst.frc.team115.robot.commands.CheesyDriveJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends Subsystem implements PIDOutput {
	public static PIDController PIDController;

	private static final double SENSITIVITY = 0.90;
	private DoubleSolenoid shifter;
	private boolean isHighGear;
	private DoubleFunction<Double> limiter = limiter(-0.9, 0.9);

	private double quickStopAccumulator = 0.0;
	private double wheelDeadband = 0.03;
	private double throttleDeadband = 0.02;

	private Encoder leftEncoder, rightEncoder;

	private boolean limitCurrent = false;

	public DriveTrain() {
//		leftEncoder = new Encoder(Constants.kLeftAchannel, Constants.kLeftBchannel);
//		leftEncoder.setReversceDirection(true);
//		rightEncoder = new Encoder(Constants.kRightAchannel, Constants.kRightBchannel);

		// rightEncoder.setReverseDirection(true);  
//		leftEncoder.setDistancePerPulse(Constants.kInchesPerTicks);
//		rightEncoder.setDistancePerPulse(Constants.kInchesPerTicks);

		Hardware.driveFrontLeft = new TalonSRX(Constants.kDriveFrontLeftTalonID);
		Hardware.driveBackLeft = new TalonSRX(Constants.kDriveBackLeftTalonID);
		Hardware.driveBackLeft.set(ControlMode.Follower, Hardware.driveFrontLeft.getDeviceID());
		Hardware.driveBackLeft.setInverted(true);

		Hardware.driveFrontRight = new TalonSRX(Constants.kDriveFrontRightTalonID);
		Hardware.driveBackRight = new TalonSRX(Constants.kDriveBackRightTalonID);
		Hardware.driveBackRight.set(ControlMode.Follower, Hardware.driveFrontRight.getDeviceID());

		/* first choose the sensor */
		Hardware.driveFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		Hardware.driveFrontLeft.setSensorPhase(true);
		Hardware.driveFrontLeft.setInverted(true);

		Hardware.driveFrontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);
		Hardware.driveFrontRight.setSensorPhase(true);
		Hardware.driveFrontRight.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		//		Hardware.driveFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		//		Hardware.driveFrontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		//		Hardware.driveFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		//		Hardware.driveFrontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* set the peak and nominal outputs */
		//		Hardware.driveFrontLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
		//		Hardware.driveFrontLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);

		if (limitCurrent) {
			Hardware.driveFrontLeft.configPeakCurrentLimit(30, 0);
			Hardware.driveFrontLeft.configContinuousCurrentLimit(20, 0);
			Hardware.driveFrontLeft.configPeakCurrentDuration(10, 0);
			Hardware.driveFrontLeft.enableCurrentLimit(true);
		}
		else {
			Hardware.driveFrontLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
			Hardware.driveFrontLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		}

		//		frontLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
		//		frontLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		//		Hardware.driveFrontRight.configNominalOutputForward(0, Constants.kTimeoutMs);
		//		Hardware.driveFrontRight.configNominalOutputReverse(0, Constants.kTimeoutMs);

		if (limitCurrent) {
			Hardware.driveFrontRight.configPeakCurrentLimit(30, 0);
			Hardware.driveFrontRight.configContinuousCurrentLimit(20, 0);
			Hardware.driveFrontRight.configPeakCurrentDuration(10, 0);
			Hardware.driveFrontRight.enableCurrentLimit(true);
		}
		else {
			Hardware.driveFrontRight.configPeakOutputForward(1, Constants.kTimeoutMs);
			Hardware.driveFrontRight.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		}

		//		frontRight.configPeakOutputForward(1, Constants.kTimeoutMs);
		//		frontRight.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* set closed loop gains in slot0 - see documentation */
		Hardware.driveFrontLeft.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		Hardware.driveFrontLeft.config_kF(0, Constants.kDriveF, Constants.kTimeoutMs);
		Hardware.driveFrontLeft.config_kP(0, Constants.kDriveP, Constants.kTimeoutMs);
		Hardware.driveFrontLeft.config_kI(0, Constants.kDriveI, Constants.kTimeoutMs);
		Hardware.driveFrontLeft.config_kD(0, Constants.kDriveD, Constants.kTimeoutMs);

		Hardware.driveFrontRight.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		Hardware.driveFrontRight.config_kF(0, Constants.kDriveF, Constants.kTimeoutMs);
		Hardware.driveFrontRight.config_kP(0, Constants.kDriveP, Constants.kTimeoutMs);
		Hardware.driveFrontRight.config_kI(0, Constants.kDriveI, Constants.kTimeoutMs);
		Hardware.driveFrontRight.config_kD(0, Constants.kDriveD, Constants.kTimeoutMs);

		/* set acceleration and vcruise velocity - see documentation */
		Hardware.driveFrontLeft.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		Hardware.driveFrontLeft.configMotionAcceleration(6000, Constants.kTimeoutMs);

		Hardware.driveFrontRight.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		Hardware.driveFrontRight.configMotionAcceleration(6000, Constants.kTimeoutMs);

		/* zero the sensor */
		Hardware.driveFrontLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		Hardware.driveFrontRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		
		shifter = new DoubleSolenoid(1, 3, 4);

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
		setLeftRightMotorOutputs(0.0, 0.0);
	}

	public void setLeftRightMotorOutputs(double left, double right) {
		Hardware.driveFrontRight.set(ControlMode.PercentOutput, right);
		Hardware.driveFrontLeft.set(ControlMode.PercentOutput, left);
	}

	public void shift() {
		// TODO Add code for shifting from low -> high or vice versa
		if (isHighGear) {
			shifter.set(Value.kReverse);
		} else {
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
		Hardware.driveFrontLeft.set(ControlMode.MotionMagic, targetPos);
		Hardware.driveFrontRight.set(ControlMode.MotionMagic, targetPos);
	}

	public void setTurnSetpoint(double angle) {
		PIDController.enable();
		PIDController.setSetpoint(angle);
	}

	protected void initDefaultCommand() {
		setDefaultCommand(new CheesyDriveJoystick());
	}

	public double getCurrentDist() {
		return (rightEncoder.getDistance() + leftEncoder.getDistance()) / 2;
	}

	public double getLeftDist() {
		return leftEncoder.getDistance();
	}

	public double getRightDist() {
		return rightEncoder.getDistance();
	}

	public double getLeftVel() {
		return leftEncoder.getRate();
	}

	public double getRightVel() {
		return rightEncoder.getRate();
	}

	public double getCurrentVel() {
		return (leftEncoder.getRate() + rightEncoder.getRate()) / 2;
	}

	public void zeroPos() {
		leftEncoder.reset();
		rightEncoder.reset();
	}

	@Override
	public void pidWrite(double output) {
		setLeftRightMotorOutputs(-output, output);
	}
	
	
	public void log() {
		SmartDashboard.putNumber("Encoder L", Hardware.driveFrontLeft.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Encoder R", Hardware.driveFrontRight.getSelectedSensorPosition(0));
	}
	
}
