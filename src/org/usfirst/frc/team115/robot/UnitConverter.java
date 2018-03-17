package org.usfirst.frc.team115.robot;

public class UnitConverter {
	
	public static final double kPulleyRadius = 1.282 / 2.0; //0.75636;
	public static final double kWheelRadius = 2.0;
	
	
	public static double convertFeetToTicks(double feet) {
		return feet * (12 / (2.0 * kWheelRadius * Math.PI)) * 4096;
	}
	
	public static double convertElevatorFeetToTicks(double feet) {
		return feet * (12 / (2.0 * kPulleyRadius * Math.PI)) * 4096;
	}

	public static double convertTicksToFeet(double ticks) {
		return (ticks / 4096.0) * (2.0 * kWheelRadius * Math.PI / 12);
	}
	
	public static double convertElevatorTicksToFeet(double ticks) {
		return (ticks / 4096.0) * (2.0 * kPulleyRadius * Math.PI / 12);
	}

	public static double convertTicksToInches(double ticks) {
		return (ticks/4096.0) * (2 * kPulleyRadius * Math.PI);
	}

	public static double convertInchesToTicks(double inches) {
		return (inches / (2 * kPulleyRadius * Math.PI) * 4096);
	}

	public static double convertMetersToTicks(double meters) {
		return ((meters / (0.0254 * (2 * kPulleyRadius * Math.PI))) * 4096.0); 
	}

	public static double convertTicksToMeters(double ticks) {
		return ((ticks / 4096.0) * (2 * kPulleyRadius * Math.PI) * 0.0254);
	}
}