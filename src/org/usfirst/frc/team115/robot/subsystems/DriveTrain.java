package org.usfirst.frc.team115.robot.subsystems;

import java.util.function.DoubleFunction;

import org.usfirst.frc.team115.robot.Constants;
import org.usfirst.frc.team115.robot.Robot;
import org.usfirst.frc.team115.robot.commands.CheesyDriveJoystick;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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

	public TalonSRX frontLeft, frontRight;
	public VictorSPX backLeft, backRight;
	public PIDController turnController;
	public PIDController driveStraightController;
	public PIDController driveDistanceController;
	public PIDController PIDController;
	public static AHRS navX;

	private static final double SENSITIVITY = 0.75;
	private DoubleSolenoid shifter;
	private boolean isHighGear;
	private DoubleFunction<Double> limiter = limiter(-1.0, 1.0);

	private double quickStopAccumulator = 0.0;
	private double wheelDeadband = 0.02;
	private double throttleDeadband = 0.02;

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

	public PidState state = PidState.NULL;
	public DriveStraightPIDOutput driveStraightOutput;

	private boolean limitCurrent = false;
	private boolean autonSpeedLimit = false;

	public DriveTrain() {
		//		isHighGear = true;
		frontLeft = new TalonSRX(9); //9
		backLeft = new VictorSPX(8); //11
		backLeft.follow(frontLeft);
		// backLeft.set(ControlMode.Follower, frontLeft.getDeviceID());
		// frontLeft.setInverted(true);
		// backLeft.setInverted(true);
		frontLeft.setInverted(false);
		backLeft.setInverted(false);

		frontRight = new TalonSRX(11); //2
		backRight = new VictorSPX(10); //0 //10
		backRight.follow(frontRight);
		// backRight.set(ControlMode.Follower, frontRight.getDeviceID());
		frontRight.setInverted(true);
		backRight.setInverted(true);


		frontRight.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 0);
		frontLeft.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 0);

		/* first choose the sensor */
		frontLeft.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		frontLeft.setSensorPhase(false);

		frontRight.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kPIDLoopIdx,Constants.kTimeoutMs);
		frontRight.setSensorPhase(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		frontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		frontLeft.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		frontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		frontRight.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* set the peak and nominal outputs */
		frontLeft.configNominalOutputForward(0.0, Constants.kTimeoutMs);
		frontLeft.configNominalOutputReverse(0.0, Constants.kTimeoutMs);
		frontRight.configNominalOutputForward(0.0, Constants.kTimeoutMs);
		frontRight.configNominalOutputReverse(0.0, Constants.kTimeoutMs);

		frontLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
		frontLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		frontRight.configPeakOutputForward(1, Constants.kTimeoutMs);
		frontRight.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		if (limitCurrent) {
			frontLeft.configPeakOutputForward(0.3, Constants.kTimeoutMs);
			frontLeft.configPeakOutputReverse(-0.3, Constants.kTimeoutMs);
			frontRight.configPeakOutputForward(0.3, Constants.kTimeoutMs);
			frontRight.configPeakOutputReverse(-0.3, Constants.kTimeoutMs);
		}
		else if (autonSpeedLimit) {
			frontLeft.configPeakOutputForward(0.6, Constants.kTimeoutMs);
			frontLeft.configPeakOutputReverse(-0.6, Constants.kTimeoutMs);
			frontRight.configPeakOutputForward(0.6, Constants.kTimeoutMs);
			frontRight.configPeakOutputReverse(-0.6, Constants.kTimeoutMs);
		}
		else {
			frontLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
			frontLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);
			frontRight.configPeakOutputForward(1, Constants.kTimeoutMs);
			frontRight.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		}

		/* set closed loop gains in slot0 - see documentation */
		frontLeft.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		frontLeft.config_kF(0, Constants.kDriveFrontLeftF, Constants.kTimeoutMs);
		frontLeft.config_kP(0, Constants.kDriveP, Constants.kTimeoutMs);
		frontLeft.config_kI(0, Constants.kDriveI, Constants.kTimeoutMs);
		frontLeft.config_kD(0, Constants.kDriveD, Constants.kTimeoutMs);

		frontRight.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		frontRight.config_kF(0, Constants.kDriveFrontRightF, Constants.kTimeoutMs);
		frontRight.config_kP(0, Constants.kDriveP, Constants.kTimeoutMs);
		frontRight.config_kI(0, Constants.kDriveI, Constants.kTimeoutMs);
		frontRight.config_kD(0, Constants.kDriveD, Constants.kTimeoutMs);

		/* set acceleration and vcruise velocity - see documentation */
		frontLeft.configMotionCruiseVelocity((int)(15.0 * (12 / (2.0 * 2.0 * Math.PI)) * 4096.0), Constants.kTimeoutMs);
		frontLeft.configMotionAcceleration((int)(2.0 * (12 / (2.0 * 2.0 * Math.PI)) * 4096.0), Constants.kTimeoutMs);

		frontRight.configMotionCruiseVelocity((int)(15.0 * (12 / (2.0 * 2.0 * Math.PI)) * 4096.0), Constants.kTimeoutMs);
		frontRight.configMotionAcceleration((int)(2.0 * (12 / (2.0 * 2.0 * Math.PI)) * 4096.0), Constants.kTimeoutMs);

		/* zero the sensor */
		frontLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		frontRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		//		frontLeft.clearStickyFaults(Constants.kTimeoutMs);
		//		frontRight.clearStickyFaults(Constants.kTimeoutMs);
		//		backLeft.clearStickyFaults(Constants.kTimeoutMs);
		//		backRight.clearStickyFaults(Constants.kTimeoutMs);

		//		PIDController = new PIDController(0, 0, 0, this, this);

		navX = new AHRS(SPI.Port.kMXP);

		turnController = new PIDController(Constants.kTurnP, Constants.kTurnI, Constants.kTurnD, navX, this, 0.005);
		turnController.setInputRange(-180, 180);
		turnController.setContinuous(true);
		turnController.setOutputRange(-1.0, 1.0);
		turnController.setAbsoluteTolerance(1);

		driveStraightOutput = new DriveStraightPIDOutput(state, this);
		driveStraightController = new PIDController(Constants.kDriveStraightP, Constants.kDriveStraightI, Constants.kDriveStraightD, navX, driveStraightOutput, 0.005);
		driveStraightController.setInputRange(-180, 180);
		driveStraightController.setContinuous(true);
		driveStraightController.setOutputRange(-1.0, 1.0);
		driveStraightController.setAbsoluteTolerance(1);

		driveDistanceController = new PIDController(Constants.kDriveP, Constants.kDriveI, Constants.kDriveD, this, this, 0.005);
//		driveStraightController.setInputRange(-20.0, 20);
//		driveStraightController.setContinuous(true);
		driveDistanceController.setOutputRange(-1.0, 1.0);
//		driveDistanceController.setAbsoluteTolerance(1);
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
		SmartDashboard.putNumber("Calculated Left Motor Output", left);
		SmartDashboard.putNumber("Calculated Right Motor Output", right);
		frontLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
		frontLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
		frontRight.configNominalOutputForward(0, Constants.kTimeoutMs);
		frontRight.configNominalOutputReverse(0, Constants.kTimeoutMs);
		frontRight.set(ControlMode.PercentOutput, right);
		frontLeft.set(ControlMode.PercentOutput, left);
	}

	public void setMotionProfile(TrajectoryPoint leftPoint, TrajectoryPoint rightPoint) {
		frontLeft.pushMotionProfileTrajectory(leftPoint);
		frontRight.pushMotionProfileTrajectory(rightPoint);
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

	public double convertFeetToTicks(double feet) {
		return feet * (12 / (2.0 * 2.0 * Math.PI)) * 4096; //2048.0;
	}

	public double convertTicksToFeet(double ticks) {
		return (ticks / 4096.0) * (2.0 * 2.0 * Math.PI / 12);
	}

	double setpoint;
	public void setDistanceSetpoint(double dist) {
		double targetPos = convertFeetToTicks(dist); 
		this.setpoint = targetPos;
		frontLeft.set(ControlMode.Position, targetPos);
		frontRight.set(ControlMode.Position, targetPos);
	}

	public void setTurnSetpoint(double angle) {
		if (state == PidState.DRIVESTRAIGHT) {
			driveStraightController.disable();
		}
		state = PidState.TURN;
		turnController.setSetpoint(angle);
	}

	public void setPeakSpeed(double speed) {
		frontLeft.configPeakOutputForward(speed, Constants.kTimeoutMs);
		frontLeft.configPeakOutputReverse(-speed, Constants.kTimeoutMs);
		frontRight.configPeakOutputForward(speed, Constants.kTimeoutMs);
		frontRight.configPeakOutputReverse(-speed, Constants.kTimeoutMs);
	}

	protected void initDefaultCommand() {
		setDefaultCommand(new CheesyDriveJoystick());
	}

	public double getCurrentDist() {
		double dist = convertTicksToFeet((frontRight.getSelectedSensorPosition(0) + frontLeft.getSelectedSensorPosition(0))/2.0);
		SmartDashboard.putNumber("Current Dist (in feet)", dist);
		return dist;
	}

	public double getError() {
		SmartDashboard.putNumber("Setpoint", this.setpoint);
		double error = this.setpoint - getCurrentDist();
		SmartDashboard.putNumber("Error", error);
		return error;
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

	public double getDriveStraightOutput() {
		return driveStraightController.get();
	}

	public double getYaw() {
		return navX.getYaw();
	}

	public double getTotalAngle(double delta) {
		return (getYaw() + delta + 540)%360 - 180;
	}

	public double getCurrentAngle() {
		return navX.pidGet();
	}

	public double getTurnOutput() {
		return turnController.get();
	}
	
	public void zeroYaw() {
		navX.zeroYaw();
	}

	public void zeroPosition() {
		frontLeft.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		frontRight.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}

	public double pidGet() {
		if (state == PidState.TURN || state == PidState.DRIVESTRAIGHT)
			return navX.pidGet();
		else if (state == PidState.DISTANCE) {
			return getCurrentDist();//-getError();
		}
		return 0.0;
	}
	
	public void resetPidState() {
		state = PidState.NULL;
	}

	double defaultSpeed;
	double turnSpeed;
	double nominalDriveOutput = 0.3;
	double nominalTurnOutput = 0.35;

	@Override
	public void pidWrite(double output) {
		if (state == PidState.TURN) {
			if (output >= 0.0)
				output = Math.max(nominalTurnOutput, output);
			else
				output = Math.min(-nominalTurnOutput, output);
			setLeftRightMotorOutputs(output, -output);
		}
		else if (state == PidState.DRIVESTRAIGHT) {
			setLeftRightMotorOutputs(defaultSpeed + output, defaultSpeed - output);
		}
		else if (state == PidState.DISTANCE) {
			if (output >= 0.0)
				output = Math.max(nominalDriveOutput, output);
			else
				output = Math.min(-nominalDriveOutput, output);
			turnSpeed = driveStraightOutput.getOutput();
//			turnSpeed = 0.0;
			SmartDashboard.putNumber("TurnSpeed", turnSpeed);
			SmartDashboard.putNumber("Motor output", output);
			setLeftRightMotorOutputs(output + turnSpeed, output - turnSpeed);
		}
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub

	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return PIDSourceType.kDisplacement;
	}
}
