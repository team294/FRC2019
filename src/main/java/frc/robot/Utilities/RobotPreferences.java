package frc.robot.Utilities;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotPreferences {

	private final Preferences prefs;
	
	/*
	 * all of the robot preferences
	 */
	public boolean inBCRLab;			// Set true if in the BCR lab (with a big pole in the middle of the field)
	public boolean prototypeRobot;		// Set true if using code for prototype bots, false for practice and competition bots
	public boolean driveDirection;		// true for reversed
	public double wheelCircumference;	// wheel circumference, in inches
	public double driveTrainDistanceFudgeFactor;  // Scaling factor for driving distance (default = 1)
	
	/**
	 * Creates a RobotPreferences object and reads the robot preferences.
	 */
	public RobotPreferences() {
		prefs = Preferences.getInstance();
		refresh();
	}
	
	/**
	 * Re-reads the robot preferences.
	 */
	public void refresh() {
		inBCRLab = prefs.getBoolean("inBCRLab", false);
		prototypeRobot = prefs.getBoolean("prototypeRobot", false); // true if testing code on a prototype, default to false (competition bot)
		driveDirection = prefs.getBoolean("driveDirection", true);
		wheelCircumference = prefs.getDouble("wheelDiameter", 6) * Math.PI;		
		driveTrainDistanceFudgeFactor = prefs.getDouble("driveTrainDistanceFudgeFactor", 1);
		/* if (driveTrainDistanceFudgeFactor == -9999) {
			// If fudge factor for driving can't be read, then assume value of 1
			driveTrainDistanceFudgeFactor = 1;  //0.96824;
		} */
	}

	public String getString(String k) {
		return getString(k, null);
	}
	public String getString(String k, String d) {
		return prefs.getString(k, d);
	}
	public int getInt(String k) {
		return getInt(k, 0);
	}
	public int getInt(String k, int d) {
		return prefs.getInt(k, d);
	}
	public double getDouble(String k, double d) {
		return prefs.getDouble(k, d);
	}
	public double getDouble(String k) {
		return getDouble(k, 0);
	}
}