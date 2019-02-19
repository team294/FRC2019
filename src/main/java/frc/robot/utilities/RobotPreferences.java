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
	public double wheelCircumference;	// Wheel circumference, in inches
	public double elevatorGearCircumference; //circumference of the gear driving the elevator in inches
	public double elevatorBottomToFloor; //distance of elevator 0 value from the ground
	public double elevatorWristSafe; 	 //lowest elevator position (from ground) where wrist can't hit the floor at its lower limit switch.  Wrist can be stowed in this position.
	public double cameraDistanceFromFrontOfBumper;  // (default = 12 inches)
	public double wristGearRatio; 		// wrist gear ratio, gear with encoder / gear driving wrist
	public double wristCalZero;   		// Wrist encoder position at O degrees, in encoder ticks (i.e. the calibration factor)
	public boolean wristCalibrated = false;     // Default to wrist being uncalibrated.  Calibrate from robot preferences or "Calibrate Wrist Zero" button on dashboard
	public double climbCalZero; // Climb encoder position at 0 degrees in encoder ticks
	public boolean climbCalibrated = false; // Default to climb being uncalibrated
	public double vacuumCurrentThreshold; // assume left climb vacuum is achieved if this threshold is passed TODO needs to be tested

	/*
	* Measurements
	*/
	// Wrist Angles (in degrees)
	public static final double wristMax = 110.0;
	public static final double wristStowed = 107.36;
	public static final double wristKeepOut = 60.0;  // Max angle to avoid interference with elevator or climber
	public static final double wristUp = 15.0;
	public static final double wristStraight = 0.0;
	public static final double wristDown = -45.0;
	public static final double wristMin = -50.0;
	public enum WristAngle {stowed, up, straight, down}

	// TODO Update with 2019 base
  	// Robot Pathfinder data
  	public final double encoderTicksPerRevolution = 4096.0;
  	public final double wheelbase_in = 25.0;       // wheelbase, in inches
  	// public static final double wheel_diameter_in = 6.0;   // wheel diamater, in inches  -- DO NOT USE -- Use wheelCircumference preference instead
  	// public static final double wheel_distance_in_per_tick = wheel_diameter_in*Math.PI/encoderTicksPerRevolution;  // wheel distance traveled per encoder tick, in inches
  	public final double max_velocity_ips = 200.0;   // max robot velocity, in inches per second
  	public final double max_acceleration_ipsps = 130.0;  // max robot acceleration, in inches per second per second
  	public final double max_jerk_ipspsps = 2400.0;  // max robot jerk, in inches per second per second per second

	// Hatch piston positions
	//public enum HatchPistonPositions { grab, release, moving, unknown }

	/*
	Measurement variables
	*/

	// Field level heights (for elevator targeting), in inches
	public final double hatchLow = 19.0;
  	public final double hatchMid = 47.0;
  	public final double hatchHigh = 75.0;
  	public final double cargoShipCargo = 34.75;
	public final double rocketBallOffset = 8.5;
	public final double loadCargo = 44.125;
	public final double groundCargo = 10.0; 

	public enum ElevatorPosition {bottom, wristSafe, hatchLow, hatchMid, hatchHigh, cargoShipCargo, loadCargo, groundCargo}

	//Climb Target Angles (in degrees)
	//TODO Test and adjust angles when climb is built
	public final double climbStartingAngle = 120.0;
	public final double climbLiftAngle = 125.0;
	public final double climbVacuumAngle = -5.0;
	public final double climbMinAngle = -20.0;
	
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
		elevatorGearCircumference = prefs.getDouble("elevatorGearDiameter", 1.7) * Math.PI; // TODO Change value when actual elevator is built, Conversion factor for makeshift elevator 18/32.3568952084);
		elevatorBottomToFloor = prefs.getDouble("elevatorBottomToFloor", 15.0); //TODO Change value when actual elevator is built
		elevatorWristSafe = prefs.getDouble("elevatorWristSafe", 20.0); //TODO Change value when actual elevator is built (elevator position from floor where wrist can't hit the floor at its lower limit switch.  Wrist can be stowed in this position.)
		cameraDistanceFromFrontOfBumper = prefs.getDouble("cameraDistanceFromFrontOfBumper", 12);
		wristGearRatio = prefs.getDouble("wristGearRatio", 1.0);
		wristCalZero = prefs.getDouble("wristCalZero", -9999);
		wristCalibrated = (wristCalZero != -9999);
		if(!wristCalibrated) {
			DriverStation.reportError("Error: Preferences missing from RoboRio for Wrist calibration.", false);
			recordStickyFaults("Preferences-wristCalZero");
			wristCalZero = 0;
		}	
		climbCalZero = prefs.getDouble("climbCalZero", -9999);
		climbCalibrated = (climbCalZero != -9999);
		if(!climbCalibrated) {
			DriverStation.reportError("Error: Preferences missing from RoboRio for Climb calibration.", false);
			recordStickyFaults("Preferences-climbCalZero");
			climbCalZero = 0;
		}	
		vacuumCurrentThreshold = prefs.getDouble("vacuumCurrentThreshold", 3.0);
	}

	/**
	 * Sets climb angle calibration factor and enables angle control modes for climb.
	 * 
	 * @param climbCalZero
	 *            Calibration factor for climb
	 * @param writeCalToPreferences
	 *            true = store calibration in RobotPrefs, false = don't change RobotPrefs
	 */
	public void setClimbCalibration(double climbCalZero, boolean writeCalToPreferences) {
		this.climbCalZero = climbCalZero;
		climbCalibrated = true;
		Robot.climb.stopClimbMotor();  // Stop motor, so it doesn't jump to new value
		Robot.log.writeLog("Preferences", "Calibrate climber", "zero value," + climbCalZero);
		if (writeCalToPreferences) {
			prefs.putDouble("climbCalZero", climbCalZero);
		}
	}

	/**
	 * Stops climb motor and sets climbCalibrated to false
	 */
	public void setClimbUncalibrated() {
		Robot.climb.stopClimbMotor();
		climbCalibrated = false;
		Robot.log.writeLog("Preferences", "Uncalibrate climber", "");
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
		if (!prefs.containsKey("elevatorGearDiameter")) {
			prefs.putDouble("elevatorGearDiameter", 1.7);
		}
		if (!prefs.containsKey("elevatorBottomToFloor")) {
			prefs.putDouble("elevatorBottomToFloor", 15.0);
		}
		if (!prefs.containsKey("elevatorWristSafe")) {
			prefs.putDouble("elevatorWristSafe", 20.0);
		}
		if (!prefs.containsKey("cameraDistanceFromFrontOfBumper")){
			prefs.putDouble("cameraDistanceFromFrontOfBumper", 12);
		}
		if (!prefs.containsKey("wristGearRatio")) {
			prefs.putDouble("wristGearRatio", 1.0);
		}
		if (!prefs.containsKey("wristCalZero")) {
			prefs.putDouble("wristCalZero", -9999);
		}
		if (!prefs.containsKey("climbCalZero")) {
			prefs.putDouble("climbCalZero", -9999);
		}
		if (!prefs.containsKey("vacuumCurrentThreshold")) {
			prefs.putDouble("vacuumCurrentThreshold", 3.0);
		}
	}

	/**
	 * Sets wrist angle calibration factor and enables angle control modes for wrist
	 * 
	 * @param wristCalZero  Calibration factor for wrist
	 * @param writeCalToPreferences  true = store calibration in Robot Preferences, false = don't change Robot Preferences
	 */
	public void setWristCalibration(double wristCalZero, boolean writeCalToPreferences) {
		this.wristCalZero = wristCalZero;
		wristCalibrated = true;
		Robot.wrist.stopWrist();	// Stop motor, so it doesn't jump to new value
		Robot.log.writeLog("Preferences", "Calibrate wrist", "zero value," + wristCalZero);
		if (writeCalToPreferences) {
			prefs.putDouble("wristCalZero", wristCalZero);
		}
	}

	/**
	 * Stops wrist motor and sets wristCalibrated to false
	 */
	public void setWristUncalibrated() {
		Robot.wrist.stopWrist();;
		wristCalibrated = false;
		Robot.log.writeLog("Preferences", "Uncalibrate wrist", "");
	}

	// Much of this is not going to be useful in competition. The drivers are not going to look at the laptop screen to see if a subsystem has thrown an error.
	// TODO: delete the method(s) below by competition time

	/**
	 * Records in robotPreferences, fileLog, and Shuffleboard that a problem was found in a subsystem
	 * (only records if the subsystem wasn't already flagged)
	 * @param subsystem String name of subsystem in which a problem exists
	 */
	public void recordStickyFaults(String subsystem) {
		if (problemSubsystem.indexOf(subsystem) == -1) {
			if (problemSubsystem.length() != 0) {
				problemSubsystem = problemSubsystem + ", ";
			}
			problemSubsystem = problemSubsystem + subsystem;
			putString("problemSubsystem", problemSubsystem);
			Robot.log.writeLogEcho(subsystem, "Sticky Fault Logged", "");
		}
		if (!problemExists) {
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
	public void putString(String key, String val) {
		prefs.putString(key, val);
	}
	public void putDouble(String key, double val) {
		prefs.putDouble(key, val);
	}	
	public void putInt(String key, int val) {
		prefs.putInt(key, val);
	}
	public void putBoolean(String key, boolean val) {
		prefs.putBoolean(key, val);
	}

}