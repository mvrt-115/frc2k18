package org.usfirst.frc.team115.robot;


/**
 * 
 * This is essentially RobotMap,
 * but it also includes any tuning constants as well.
 * @author Ishan
 *
 */
public class Constants {

	public static double kDriveP = 1.1 * 1.0/22.0; 
	public static double kDriveI = 0.00;
	public static double kDriveD = 1 / (8.0 * 4.5);
	public static double kDriveFrontLeftF = 1023 * 0.7576 / 3488.0;
	public static double kDriveFrontRightF = 1023 * 0.8591 / 5098.0;
	
	
	public static double kElevatorP = 2.4;
	public static double kElevatorI = 0.000;
	public static double kElevatorD = 1;
	public static double kElevatorF = 0.0;
	
	public static int kIntakeLeftTalonID = 41;
	public static int kIntakeRightTalonID = 13;
	public static int kElevatorLeftTalonID = 9;
	public static int kElevatorRightTalonID = 3; 
	public static int kCarriageLeftTalonID = 42;
	public static int kCarriageRightTalonID = 6;
	public static int kDriveFrontLeftTalonID = 2;
	public static int kDriveFrontRightTalonID = 17;
	public static int kDriveBackLeftTalonID = 4;
	public static int kDriveBackRightTalonID = 40;

	public static double kDriveStraightP = 1.0 / 15.0; //1/30.0;
	public static double kDriveStraightI = 0.0;
	public static double kDriveStraightD = 1 / (8.0 * 2.0); //1 / (8.0 * 1.5);
	
	public static double kTurnP = 1.0 / 96.50; //1.0 / 95.0; //old: 0.5 * 1.0 / 90.0;
	public static double kTurnI = 0.0;
	public static double kTurnD = 1 / (8.0 * 2.05); //1 / (8.0 * 3.4); //old: 1 / (8.0 * 3.2);


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

	public static double kDefaultSwitchHeight = 0.35;
}
