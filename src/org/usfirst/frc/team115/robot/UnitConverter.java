package org.usfirst.frc.team115.robot;

public class UnitConverter {
	
	public static double convertFeetToTicks(double feet) {
		return feet * (12 / (2.0 * 2.0 * Math.PI)) * 4096;
	}

	public static double convertTicksToFeet(double ticks) {
		return (ticks / 4096.0) * (2.0 * 2.0 * Math.PI / 12);
	}

	public static double convertTicksToInches(double ticks) {
		return (ticks/4096) * 2 * 0.75636 * Math.PI;
	}

	public static double convertInchesToTicks(double inches) {
		return (inches / (2 * 0.75636 * Math.PI) * 4096);
	}

	public static double convertMetersToTicks(double meters) {
		return ((meters / 0.0254) * (1 / (2 * 0.75636 * Math.PI)) * 4096); 
	}

	public static double convertTicksToMeters(double ticks) {
		return (ticks * 0.0254 / (1 / (2 * 0.75636 * Math.PI)) * 4096);
	}
}