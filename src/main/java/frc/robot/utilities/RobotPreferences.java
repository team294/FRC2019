package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class RobotPreferences {

	private final Preferences prefs;
	
	/*
	 * all of the robot preferences
	 */
	public String problemSubsystem;     // Records which subsystem(s) problem(s) exist in
	public boolean problemExists;       // Set true if there is an issue
	public boolean inBCRLab;			// Set true if in the BCR lab (with a big pole in the middle of the field)
	public boolean prototypeRobot;		// Set true if using code for prototype bots, false for practice and competition bots
	public boolean driveDirection;		// True for reversed
	public boolean climbCalibrated = false; // Default to arm being uncalibrated
	public double climbCalZero; // Climb encoder position at 0 degrees in encoder ticks
	public double wheelCircumference;	// Wheel circumference, in inches
	public double driveTrainDistanceFudgeFactor;  // Scaling factor for driving distance (default = 1)
	public double elevatorGearCircumference; //circumference of the gear driving the elevator in inches
	public double elevatorBottomToFloor; //distance of elevator 0 value from the ground
	public double cameraDistanceFromFrontOfBumper;  // (Default = 12 inches)


	/*
	Measurement variables
	*/

	//Elevator level heights
	public final double hatchLow = 19.0;
  	public final double hatchMid = 47.0;
  	public final double hatchHigh = 75.0;
  	public final double cargoShipCargo = 34.75;
  	public final double ballOffset = 8.5;

	public enum ElevatorPosition {hatchLow, hatchMid, hatchHigh, cargoShipCargo}

	//Climb Target Angles (in degrees)
	public final double climbStartingAngle = -50.0; //TODO Test when climb is built
	public final double vacuumTargetAngle = 180.0; //TODO Test when climb is built
	public final double robotLiftAngle = 0.0; //TODO Test when climb is built


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
		problemSubsystem = prefs.getString("problemSubsystem", "");
		problemExists = prefs.getBoolean("problemExists", false);
		inBCRLab = prefs.getBoolean("inBCRLab", false);
		prototypeRobot = prefs.getBoolean("prototypeRobot", false); // true if testing code on a prototype, default to false (competition bot w/ Victors)
		driveDirection = prefs.getBoolean("driveDirection", true);
		wheelCircumference = prefs.getDouble("wheelDiameter", 6) * Math.PI;	
		elevatorGearCircumference = prefs.getDouble("elevatorGearDiameter", 1.7 * Math.PI); // TODO Change value when actual elevator is built, Conversion factor for makeshift elevator 18/32.3568952084);
		driveTrainDistanceFudgeFactor = prefs.getDouble("driveTrainDistanceFudgeFactor", 1);
		elevatorBottomToFloor = prefs.getDouble("elevatorBottomToFloor", 15.0); //TODO Change value when actual elevator is built
		/* if (driveTrainDistanceFudgeFactor == -9999) {
			// If fudge factor for driving can't be read, then assume value of 1
			driveTrainDistanceFudgeFactor = 1;  //0.96824;
		} */
		wheelCircumference = prefs.getDouble("wheelDiameter", 6) * Math.PI;		
		cameraDistanceFromFrontOfBumper = prefs.getDouble("cameraDistanceFromFrontOfBumper", 12);	
		climbCalZero = prefs.getDouble("calibrationZeroDegrees", -9999);
		climbCalibrated = (climbCalZero != -9999);
		if(!climbCalibrated) {
			DriverStation.reportError("Error: Preferences missing from RoboRio for Climb calibration.", true);
			climbCalZero = 0;
		}	
	}

	/**
	 * Sets arm angle calibration factor and enables angle control modes for arm.
	 * 
	 * @param armCalZero
	 *            Calibration factor for arm
	 * @param writeCalToPreferences
	 *            true = store calibration in Robot Preferences, false = don't
	 *            change Robot Preferences
	 */
	public void setArmCalibration(double climbCalZero, boolean writeCalToPreferences) {
		this.climbCalZero = climbCalZero;
		climbCalibrated = true;
		if (writeCalToPreferences) {
			prefs.putDouble("calibrationZeroDegrees", climbCalZero);
		}
}

	/* Sets up Preferences if they haven't been set as when changing RoboRios or first start-up.
		The values are set to defaults, so if using the prototype robots set inBCRLab to true
	*/	
	public void doExist(){				 
		if (!prefs.containsKey("problemSubsystem")){
			prefs.putString("problemSubsystem", "");
		}
		if (!prefs.containsKey("problemExists")) {
			prefs.putBoolean("problemExists", false);
		}
		if (!prefs.containsKey("inBCRLab")){
			 prefs.putBoolean("inBCRLab", false);
		}	 
		if (!prefs.containsKey("prototypeRobot")){
			prefs.putBoolean("prototypeRobot", false);
		}
		if (!prefs.containsKey("driveDirection")){
			prefs.putBoolean("driveDirection", true);
		}
		if (!prefs.containsKey("wheelDiameter")){
			prefs.putDouble("wheelDiameter", 6);
		}
		if (!prefs.containsKey("cameraDistanceFromFrontOfBumper")){
			prefs.putDouble("cameraDistanceFromFrontOfBumper", 12);
		}
		if (!prefs.containsKey("elevatorGearDiameter")) {
			prefs.putDouble("elevatorGearDiameter", 1.7 * Math.PI);
		}
		if (!prefs.containsKey("elevatorBottomToFloor")) {
			prefs.putDouble("elevatorBottomToFloor", 15.0);
		}
		if (!prefs.containsKey("calibrationZeroDegrees")) {
			prefs.putDouble("calibrationZeroDegrees", -9999.0);		}
	}

	/**
	 * Records in robotPreferences, fileLog, and Shuffleboard that a problem was found in a subsystem
	 * (only records if the subsystem wasn't already flagged)
	 * @param subsystem String name of subsystem in which a problem exists
	 */
	public void recordStickyFaults(String subsystem) {
		if(problemSubsystem.indexOf(subsystem) == -1) {
			if(problemSubsystem.length() != 0) {
				problemSubsystem = problemSubsystem + ", ";
			}
			problemSubsystem = problemSubsystem + subsystem;
			putString("problemSubsystem", problemSubsystem);
			Robot.log.writeLog(subsystem, "Sticky Fault Logged", "");
		}
		if(!problemExists) {
			problemExists = true;
			putBoolean("problemExists", problemExists);
		}
		showStickyFaults();
	}

	/**
	 * Clears any sticky faults in the RobotPreferences and Shuffleboard
	 */
	public void clearStickyFaults() {
		problemSubsystem = "";
		problemExists = false;
		putString("problemSubsystem", problemSubsystem);
		putBoolean("problemExists", problemExists);
		showStickyFaults();
		Robot.log.writeLog("RobotPrefs", "Sticky Faults Cleared", "");
	}

	/**
	 * Show any sticky faults on Shuffleboard
	 */
	public void showStickyFaults() {
		SmartDashboard.putString("problemSubsystem", problemSubsystem);
		SmartDashboard.putBoolean("problemExists", problemExists);
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
	public boolean getBoolean(String k, boolean d) {
		return prefs.getBoolean(k, d);
	}
	public boolean getBoolean(String k) {
		return getBoolean(k, false);
	}
	
	public void putString(String k, String d) {
		prefs.putString(k, d);
	}
	public void putDouble(String k, double d) {
		prefs.putDouble(k, d);
	}	
	public void putInt(String k, int d) {
		prefs.putInt(k, d);
	}
	public void putBoolean(String k, boolean d) {
		prefs.putBoolean(k, d);
	}

}