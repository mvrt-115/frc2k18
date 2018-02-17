package org.usfirst.frc.team115.robot;


/**
 * 
 * This is essentially RobotMap,
 * but it also includes any tuning constants as well.
 * @author Ishan
 *
 */
public class Constants {

	public static double kDriveP = 0.5;
	public static double kDriveI = 0.6;
	public static double kDriveD = 0.8;
	public static double kDriveF = 0.9;
	
	public static double kElevatorP = 2.4;
	public static double kElevatorI = 0.000;
	public static double kElevatorD = 1;
	public static double kElevatorF = 0.0;

//	public static double kTurnP = 0.00575;
//	public static double kTurnI = 0.0;
//	public static double kTurnD = 0.02560;
	
	public static double kDriveStraightP = 0.0255;
	public static double kDriveStraightI = 0.0;
	public static double kDriveStraightD = 0.0;
	
	public static double kTurnP = 1.0 /64.0;
	public static double kTurnI = 0.0;
	public static double kTurnD = 0.00538;


	public static final int kTopLeftPin = 0;
	public static final int kTopRightPin = 1;
	public static final int kBottomLeftPin = 2;
	public static final int kBottomRightPin = 3;

	public static final int shifterPin1 = 0;
	public static final int shifterPin2 = 1;

	public static int kThrottle = 5; //left analog up/down
	public static int kWheel = 0; //right analog left/right
	public static int kQuickTurn = 5;
	public static int kManualElevate = 1;
	public static int kIntake = 6; //x button
	public static int kOuttake = 3; //y button

//	public static double kWinchSpeed = 1;
//	public static int kWinchMotor = 13;
//	public static int kOperatorJoystick = 0;
//	public static int kWinchButton = 1;
	
//	public static int kIntakePortA = 5;
//	public static int kIntakePortB = 6;
	
	public static int kPIDLoopIdx = 0;
	public static int kTimeoutMs = 0;
	public static int kSlotIdx = 0;


	// need to add extra height for carriage, current heights are equal to the switch/scale heights
	// in meters
	public static double offset = 0.127;
	public static double kLowScaleHeight = 1.2912 - offset;
	public static double kDefaultScaleHeight = 1.524 - offset;
	public static double kHighScaleHeight = 1.5;

	public static double kDefaultSwitchHeight = 0.47625 + 0.33 - 0.127; //1 ft. 6-3/4 in
	
	//encoder testing values
	public static double kInchesPerTicks;
	public static int kLeftAchannel = 0;
	public static int kLeftBchannel = 1;
	public static int kRightAchannel = 2;
	public static int kRightBchannel = 3;
}
