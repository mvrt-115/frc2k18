package org.usfirst.frc.team115.robot;

public class UnitConverter {
	
	public double convertFeetToTicks(double feet) {
		return feet * (12 / (2.0 * 2.0 * Math.PI)) * 4096;
	}

	public double convertTicksToFeet(double ticks) {
		return (ticks / 4096.0) * (2.0 * 2.0 * Math.PI / 12);
	}

	public double convertTicksToInches(double ticks) {
		return (ticks/4096) * 2 * 0.75636 * Math.PI;
	}

	public double convertInchesToTicks(double inches) {
		return (inches / (2 * 0.75636 * Math.PI) * 4096);
	}

	public double convertMetersToTicks(double meters) {
		return ((goal / 0.0254) * (1 / (2 * 0.75636 * Math.PI)) * 4096); 
	}

	public double convertTicksToMeters(double ticks) {
		return ticks * 0.0254 / (1 / (2 * 0.75636 * Math.PI)) * 4096);
	}
}