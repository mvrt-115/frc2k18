package org.usfirst.frc.team115.robot;


/**
 * 
 * This is essentially RobotMap,
 * but it also includes any tuning constants as well.
 * @author Ishan
 *
 */
public class Constants {

	public static double kDriveP = 0.0;
	public static double kDriveI = 0.0;
	public static double kDriveD = 0.0;
	public static double kDriveF = 0.0;
	
	public static double kElevatorP = 0.0;
	public static double kElevatorI = 0.0;
	public static double kElevatorD = 0.0;
	public static double kElevatorF = 0.0;

	public static double kTurnP = 0.0;
	public static double kTurnI = 0.0;
	public static double kTurnD = 0.0;
	
	public static int kLeftAchannel = 4;
	public static int kLeftBchannel = 5;
	public static int kRightAchannel = 6;
	public static int kRightBchannel = 7;

	public static final int kTopLeftPin = 0;
	public static final int kTopRightPin = 1;
	public static final int kBottomLeftPin = 2;
	public static final int kBottomRightPin = 3;

	public static final int shifterPin1 = 0;
	public static final int shifterPin2 = 1;

	public static int kThrottle = 0;
	public static int kWheel = 1;
	public static int kQuickTurn = 5;

	public static double kWinchSpeed = 1;
	public static int kWinchMotor = 13;
	public static int kOperatorJoystick = 0;
	public static int kWinchButton = 1;
	
	public static int kCarriagePortA;
	public static int kCarriagePortB;
	
	public static int kPIDLoopIdx;
	public static int kTimeoutMs;
	public static int kSlotIdx;
}
