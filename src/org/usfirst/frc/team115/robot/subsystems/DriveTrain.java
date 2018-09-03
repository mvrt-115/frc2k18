package org.usfirst.frc.team115.robot.subsystems;

import java.util.function.DoubleFunction;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Hardware;
import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.UnitConverter;
import org.usfirst.frc.team115.robot.commands.CheesyDriveJoystick;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveTrain extends Subsystem implements PIDOutput, PIDSource {

	private static final double SENSITIVITY = 0.90;

	public PIDController turnController;
	public PIDController driveStraightController;
	public PIDController driveDistanceController;

	public PidState state = PidState.NULL;
	public DriveStraightPIDOutput driveStraightOutput;

	private boolean isHighGear;
	private DoubleFunction<Double> limiter = limiter(-0.9, 0.9);

	private double quickStopAccumulator = 0.0;
	private double wheelDeadband = 0.03;
	private double throttleDeadband = 0.02;

	private boolean limitCurrent = false;
	private boolean autonSpeedLimit = false;
	
	private double defaultSpeed;
	private double turnSpeed;
	private double nominalDriveOutput = 0.3;
	private double nominalTurnOutput = 0.35;

	private enum PidState {
		TURN,
		DRIVESTRAIGHT,
		DISTANCE,
		NULL
	}

	public class DriveStraightPIDOutput implements PIDOutput {

		PidState state;
		PIDOutput drivetrain;
		double output;

		public DriveStraightPIDOutput(PidState state, PIDOutput drivetrain) {
			this.state = state;
			this.drivetrain = drivetrain;
		}

		public void setState(PidState state) {
			this.state = state;
		}

		public double getOutput() { return output; }

		public void pidWrite(double output) {
			if (state == PidState.DISTANCE)
				this.output = output;
			else if (state == PidState.DRIVESTRAIGHT)
				drivetrain.pidWrite(output);
		}
	};

	public DriveTrain() {
//		Hardware.shifter = new DoubleSolenoid(0, 3, 4);
		Hardware.navX = new AHRS(SPI.Port.kMXP);

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

		Hardware.driveFrontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		Hardware.driveFrontRight.setSensorPhase(true);
		Hardware.driveFrontRight.setInverted(false);

		if (limitCurrent) {
			Hardware.driveFrontLeft.configPeakOutputForward(0.3, Constants.kTimeoutMs);
			Hardware.driveFrontLeft.configPeakOutputReverse(-0.3, Constants.kTimeoutMs);
			Hardware.driveFrontRight.configPeakOutputForward(0.3, Constants.kTimeoutMs);
			Hardware.driveFrontRight.configPeakOutputReverse(-0.3, Constants.kTimeoutMs);
		} else if (autonSpeedLimit) {
			Hardware.driveFrontLeft.configPeakOutputForward(0.6, Constants.kTimeoutMs);
			Hardware.driveFrontLeft.configPeakOutputReverse(-0.6, Constants.kTimeoutMs);
			Hardware.driveFrontRight.configPeakOutputForward(0.6, Constants.kTimeoutMs);
			Hardware.driveFrontRight.configPeakOutputReverse(-0.6, Constants.kTimeoutMs);
		} else {
			Hardware.driveFrontLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
			Hardware.driveFrontLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);
			Hardware.driveFrontRight.configPeakOutputForward(1, Constants.kTimeoutMs);
			Hardware.driveFrontRight.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		}

		/* set closed loop gains in slot0 - see documentation */
		Hardware.driveFrontLeft.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		Hardware.driveFrontLeft.config_kF(0, Constants.kDriveFrontLeftF, Constants.kTimeoutMs);
		Hardware.driveFrontLeft.config_kP(0, Constants.kDriveP, Constants.kTimeoutMs);
		Hardware.driveFrontLeft.config_kI(0, Constants.kDriveI, Constants.kTimeoutMs);
		Hardware.driveFrontLeft.config_kD(0, Constants.kDriveD, Constants.kTimeoutMs);

		Hardware.driveFrontRight.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		Hardware.driveFrontRight.config_kF(0, Constants.kDriveFrontRightF, Constants.kTimeoutMs);
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

		turnController = new PIDController(Constants.kTurnP, Constants.kTurnI, Constants.kTurnD, Hardware.navX, this, 0.005);
		turnController.setInputRange(-180, 180);
		turnController.setContinuous(true);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(1);

		driveStraightOutput = new DriveStraightPIDOutput(state, this);
		driveStraightController = new PIDController(Constants.kDriveStraightP, Constants.kDriveStraightI, Constants.kDriveStraightD, Hardware.navX, driveStraightOutput, 0.005);
		driveStraightController.setInputRange(-180, 180);
		driveStraightController.setContinuous(true);
		driveStraightController.setOutputRange(-1.0, 1.0);
		driveStraightController.setAbsoluteTolerance(1);

		driveDistanceController = new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD, this, this, 0.005);
		driveDistanceController.setOutputRange(-1.0, 1.0);
	}

	/*** Drive Methods ***/

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

	public void setLeftRightMotorOutputs(double left, double right) {
		Hardware.driveFrontLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
		Hardware.driveFrontLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
		Hardware.driveFrontRight.configNominalOutputForward(0, Constants.kTimeoutMs);
		Hardware.driveFrontRight.configNominalOutputReverse(0, Constants.kTimeoutMs);

		Hardware.driveFrontRight.set(ControlMode.PercentOutput, right);
		Hardware.driveFrontLeft.set(ControlMode.PercentOutput, left);

		SmartDashboard.putNumber("Calculated Left Motor Output", left);
		SmartDashboard.putNumber("Calculated Right Motor Output", right);
	}
	
	public void setMotionProfile(TrajectoryPoint leftPoint, TrajectoryPoint rightPoint) {
		Hardware.driveFrontLeft.pushMotionProfileTrajectory(leftPoint);
		Hardware.driveFrontRight.pushMotionProfileTrajectory(rightPoint);
	}

	public void shift() {
		if (isHighGear) {
			Hardware.shifter.set(Value.kReverse);
		} else {
			Hardware.shifter.set(Value.kForward);
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

	private static double dampen(double wheel, double wheelNonLinearity) {
		double factor = Math.PI * wheelNonLinearity;
		return Math.sin(factor * wheel) / Math.sin(factor);
	}

	public double handleDeadband(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	public void setPeakSpeed(double speed) {
		Hardware.driveFrontLeft.configPeakOutputForward(speed, Constants.kTimeoutMs);
		Hardware.driveFrontLeft.configPeakOutputReverse(-speed, Constants.kTimeoutMs);
		Hardware.driveFrontRight.configPeakOutputForward(speed, Constants.kTimeoutMs);
		Hardware.driveFrontRight.configPeakOutputReverse(-speed, Constants.kTimeoutMs);
	}

	/*** PID Controller Methods ***/

	public void setTurnSetpoint(double angle) {
		if (state == PidState.DRIVESTRAIGHT) {
			driveStraightController.disable();
		}
		state = PidState.TURN;
		turnController.setSetpoint(angle);
	}

	public void pidDriveStraight(double angle, double speed) {
		if (state == PidState.TURN) {
			turnController.disable();
		}
		else if (state == PidState.DISTANCE) {
			defaultSpeed = 0.0;
			driveStraightController.setSetpoint(angle);
			driveStraightOutput.setState(PidState.DISTANCE);
		}
		else {
			defaultSpeed = speed;
			state = PidState.DRIVESTRAIGHT;
			driveStraightController.setSetpoint(angle);
			driveStraightOutput.setState(PidState.DRIVESTRAIGHT);
		}
	}

	double setpoint;
	public void pidDriveDistance(double distanceInFeet) {
		if (state == PidState.TURN) {
			turnController.disable();
		}
		else if (state == PidState.DRIVESTRAIGHT) {
			driveStraightController.disable();
		}
		this.setpoint = distanceInFeet;
		state = PidState.DISTANCE;
		driveDistanceController.setSetpoint(setpoint);
	}

	@Override
	public void pidWrite(double output) {
		output = -output;
		if (state == PidState.TURN) {
			if (output >= 0.0)
				output = Math.max(nominalTurnOutput, output);
			else
				output = Math.min(-nominalTurnOutput, output);
			setLeftRightMotorOutputs(output, -output);
		}
		else if (state == PidState.DRIVESTRAIGHT) {
			setLeftRightMotorOutputs(-defaultSpeed + output, -defaultSpeed - output);
		}
		else if (state == PidState.DISTANCE) {
			if (output >= 0.0)
				output = Math.max(nominalDriveOutput, output);
			else
				output = Math.min(-nominalDriveOutput, output);
			turnSpeed = -driveStraightOutput.getOutput();
			SmartDashboard.putNumber("TurnSpeed", turnSpeed);
			SmartDashboard.putNumber("Motor output", output);
			setLeftRightMotorOutputs(output + turnSpeed, output - turnSpeed);
		}
	}

	public double pidGet() {
		if (state == PidState.TURN || state == PidState.DRIVESTRAIGHT)
			return Hardware.navX.pidGet();
		else if (state == PidState.DISTANCE) {
			return getCurrentDist();
		}
		return 0.0;
	}

	/*** Get Methods ***/

	public double getDriveStraightOutput() {
		return driveStraightController.get();
	}

	public double getTotalAngle(double delta) {
		return (getYaw() + delta + 540)%360 - 180;
	}

	public double getCurrentAngle() {
		return Hardware.navX.pidGet();
	}

	public double getTurnOutput() {
		return turnController.get();
	}

	public double getYaw() {
		return Hardware.navX.getYaw();
	}

	public double getError() {
		SmartDashboard.putNumber("Setpoint", this.setpoint);
		double error = this.setpoint - getCurrentDist();
		SmartDashboard.putNumber("Error", error);
		return error;
	}
	
	public double getCurrentDist() {
		double dist = UnitConverter.convertDriveTicksToFeet((Hardware.driveFrontRight.getSelectedSensorPosition(0) + Hardware.driveFrontLeft.getSelectedSensorPosition(0))/2.0);
		SmartDashboard.putNumber("Current Dist (in feet)", dist);
		return dist;
	}

	/*** Zeroing and Logging ***/

	public void log() {
		SmartDashboard.putNumber("navX output", Robot.drivetrain.getYaw());
		SmartDashboard.putNumber("Encoder L", Hardware.driveFrontLeft.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Encoder R", Hardware.driveFrontRight.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("Distance", getCurrentDist());
		SmartDashboard.putNumber("Distance Error", getError());
		SmartDashboard.putNumber("DriveDistanceController value", driveDistanceController.get());
	}

	public void resetPidState() {
		state = PidState.NULL;
	}

	public void stop() {
		setLeftRightMotorOutputs(0.0, 0.0);
	}

	public void zeroPosition() {
		Hardware.driveFrontLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		Hardware.driveFrontRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}

	public void zeroYaw() {
		Hardware.navX.zeroYaw();
	}
	
	public void zeroDrive() {
		zeroYaw();
		zeroPosition();
	}

	/*** Misc. ***/

	protected void initDefaultCommand() {
		setDefaultCommand(new CheesyDriveJoystick());
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}
}
