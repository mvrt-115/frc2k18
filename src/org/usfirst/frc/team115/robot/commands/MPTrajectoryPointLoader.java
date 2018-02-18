package org.usfirst.frc.team115.robot.commands;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;

import org.usfirst.frc.team115.robot.Robot;

import com.ctre.phoenix.motion.TrajectoryPoint;

public class MPTrajectoryPointLoader {
	
	String profileName;
	String startingPos;
	int timeIndex;
	ArrayList<Tuple> leftVels = new ArrayList<Tuple>();
	ArrayList<Tuple> rightVels = new ArrayList<Tuple>();
	
	class Tuple {
		double pos;
		double vel;
		public Tuple(double pos, double vel) {
			this.pos = pos;
			this.vel = vel;
		}
	}
	
	public MPTrajectoryPointLoader(String profileName, String startingPos) {
		this.profileName = profileName;
		this.startingPos = startingPos;
		timeIndex = 0;
	}
	
	public void drive() {
		Tuple left = leftVels.get(timeIndex);
		Tuple right = rightVels.get(timeIndex);		
		
		System.out.println("Pushing leftPoint(pos = " + left.pos + ", vel = " + left.vel + ")");
		System.out.println("Pushing rightPoint(pos = " + right.pos + ", vel = " + right.vel + ")");
		
		TrajectoryPoint leftPoint = new TrajectoryPoint();
		TrajectoryPoint rightPoint = new TrajectoryPoint();
		
		leftPoint.position = convertFeetToTicks(left.pos); //Convert feet to ticks
		leftPoint.velocity = convertFeetToTicks(left.vel) / 10.0; //Convert feet/s to ticks/100ms
		leftPoint.headingDeg = 0.0;
		leftPoint.profileSlotSelect0 = 0;
		leftPoint.profileSlotSelect1 = 0;
		leftPoint.timeDur = TrajectoryPoint.TrajectoryDuration.Trajectory_Duration_50ms;
		leftPoint.zeroPos = false;
		
		rightPoint.position = convertFeetToTicks(right.pos); //Convert feet to ticks
		rightPoint.velocity = convertFeetToTicks(right.vel) / 10.0; //Convert feet/s to ticks/100ms
		rightPoint.headingDeg = 0.0;
		rightPoint.profileSlotSelect0 = 0;
		rightPoint.profileSlotSelect1 = 0;
		rightPoint.timeDur = TrajectoryPoint.TrajectoryDuration.Trajectory_Duration_50ms;
		rightPoint.zeroPos = false;
		
		if (timeIndex == 0) {
			leftPoint.zeroPos = true;
			rightPoint.zeroPos = true;
		}
		
		Robot.drivetrain.setMotionProfile(leftPoint, rightPoint);
		timeIndex += 1;
	}
	
	public boolean isFinished() {
		return (timeIndex == leftVels.size());
	}
	
	public double convertFeetToTicks(double pos) {
		return (pos) * (12 / (2.0 * 2.0 * Math.PI)) * 4096.0;
	}
	
	public void initialize() {
		String leftDataFile = "motionProfiles/" + profileName + "/" + startingPos + "/" + profileName + startingPos + "_left_detailed.csv";
		String rightDataFile = "motionProfiles/" + profileName + "/" + startingPos + "/" + profileName + startingPos + "_right_detailed.csv";
		try {
			BufferedReader leftData = new BufferedReader(new FileReader(leftDataFile));
			BufferedReader rightData = new BufferedReader(new FileReader(rightDataFile));
			String line = "";
			leftData.readLine();
			rightData.readLine();
			while ((line = leftData.readLine()) != null) {
				String[] dataEntry = line.split(",");
				double pos = Double.parseDouble(dataEntry[3]);
				double velocity = Double.parseDouble(dataEntry[4]);
				leftVels.add(new Tuple(pos, velocity));
			}
			while ((line = rightData.readLine()) != null) {
				String[] dataEntry = line.split(",");
				double pos = Double.parseDouble(dataEntry[3]);
				double velocity = Double.parseDouble(dataEntry[4]);
				rightVels.add(new Tuple(pos, velocity));
			}
			leftData.close();
			rightData.close();
		} catch (FileNotFoundException e) {
		} catch (IOException e) {
		}
		
	}
}
