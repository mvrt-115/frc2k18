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
	
	public static double kElevatorP = 61.756;
	public static double kElevatorI = 0.000;
	public static double kElevatorD = 28.199;
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
	public static int kManualElevate = 1;

	public static double kWinchSpeed = 1;
	public static int kWinchMotor = 13;
	public static int kOperatorJoystick = 0;
	public static int kWinchButton = 1;
	
	public static int kIntakePortA = 5;
	public static int kIntakePortB = 6;
	
	public static int kPIDLoopIdx;
	public static int kTimeoutMs = 0;
	public static int kSlotIdx;


	// need to add extra height for carriage, current heights are equal to the switch/scale heights
	// in feet
	public static double kLowScaleHeight = 4;
	public static double kDefaultScaleHeight = 5;
	public static double kHighScaleHeight = 6;

	public static double kDefaultSwitchHeight = 1.5625; //1 ft. 6-3/4 in
	public static double kHighSwitchHeight = 1.5625;
}
