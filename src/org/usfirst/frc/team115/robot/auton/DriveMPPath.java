package org.usfirst.frc.team115.robot.auton;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import org.usfirst.frc.team115.robot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

public class DriveMPPath extends TimedCommand {
	
	String profileName;
	public DriveMPPath(double timeout, String profileName) {
		super(timeout);
		requires(Robot.drivetrain);
		this.profileName = profileName;
	}
	
	public void initialize() {
		String leftDataFile = "motionProfiles/" + profileName + "/" + profileName + "_left_detailed.csv";
		String rightDataFile = "motionProfiles/" + profileName + "/" + profileName + "_left_detailed.csv";
		try {
			BufferedReader leftData = new BufferedReader(new FileReader(leftDataFile));
			BufferedReader rightData = new BufferedReader(new FileReader(rightDataFile));
			String line = "";
			leftData.readLine(); //get rid of data column
			while ((line = leftData.readLine()) != null) {
				String[] dataEntry = line.split(",");
				double pos = Double.parseDouble(dataEntry[3]);
				double velocity = Double.parseDouble(dataEntry[4]);
				
			}
			leftData.close();
			rightData.close();
		} catch (FileNotFoundException e) {
			//call bail-out drivestraight
		} catch (IOException e) {
			//call bail-out drivestraight
		}
		
	}
}
