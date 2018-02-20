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
	
	public static double kElevatorP = 2.4;
	public static double kElevatorI = 0.000;
	public static double kElevatorD = 1;
	public static double kElevatorF = 0.0;
	
	public static int kIntakeLeftTalonID = 41;
	public static int kIntakeRightTalonID = 13;
	public static int kElevatorLeftTalonID = 9;  //9
	public static int kElevatorRightTalonID = 3; //3
	public static int kCarriageLeftTalonID = 42;
	public static int kCarriageRightTalonID = 6;
	public static int kDriveFrontLeftTalonID = 2;
	public static int kDriveFrontRightTalonID = 17;
	public static int kDriveBackLeftTalonID = 4;
	public static int kDriveBackRightTalonID = 40;

	public static double kTurnP = 0.0;
	public static double kTurnI = 0.0;
	public static double kTurnD = 0.0;

	public static final int kTopLeftPin = 0;
	public static final int kTopRightPin = 1;
	public static final int kBottomLeftPin = 2;
	public static final int kBottomRightPin = 3;

	public static final int shifterPin1 = 0;
	public static final int shifterPin2 = 1;

	public static int kThrottle = 5;
	public static int kWheel = 0;
	public static int kQuickTurn = 5;
	
	public static int kManualElevate = 1;
	public static int kManualMode = 3;
	public static int kZero = 4;
	
	public static int kIntake = 6;
	public static int kOuttake = 1;

	public static int kSwitch = 12;
	
	public static int kPIDLoopIdx = 0;
	public static int kTimeoutMs = 0;
	public static int kSlotIdx = 0;

	public static double offset = 0.127;
	public static double kLowScaleHeight = 0.81;
	public static double kDefaultScaleHeight = 0.97;
	public static double kHighScaleHeight = 1.15;

	public static double kDefaultSwitchHeight = 0.35; //1 ft. 6-3/4 in
	
	//encoder testing values
	public static double kInchesPerTicks;
//	public static int kLeftAchannel = 0;
//	public static int kLeftBchannel = 1;
//	public static int kRightAchannel = 2;
//	public static int kRightBchannel = 3;
}
