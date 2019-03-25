package frc.robot.utilities;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FileLog {
	
	private FileWriter fileWriter;
	private static final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
	private static final SimpleDateFormat fileDateFormat = new SimpleDateFormat("yyyy-MM-dd.HH-mm-ss");
	private String fileNameBase, fileNameFull;
	private long startTime;
	private int logLevel = 3;			// File logging level of detail.  Value between 1-3, where 1 is the most detailed and 3 is the least detailed.
	
	// File logging rotation cycles, to spread out logging times between subsystems	
	public int rotation = 0;
	public final static int DRIVE_CYCLE = 1;
	public final static int ELEVATOR_CYCLE = 3;
	public final static int WRIST_CYCLE = 5;
	public final static int CLIMB_CYCLE = 7;
	public final static int REARHATCH_CYCLE = 8;
	public final static int CARGO_CYCLE = 9;

	/**
	 * Creates a new log file called "/home/lvuser/logfile.ver.date.time.txt"
	 * @param version Version of robot code
	 */
	public FileLog(String version) {
		this("/home/lvuser/logfile", version);
	}
	
	/**
	 * Creates a new log file.  ".ver.date.time.txt" will automatically be added to the end of the bae file name.
	 * @param filenameBase Path and base name of log file
	 * @param version Version of robot code
	 */
	public FileLog(String filenameBase, String version) {
		this.fileNameBase = filenameBase + "." + version + ".";
		startTime = System.currentTimeMillis();
		fileNameFull = fileNameBase + fileDateFormat.format(startTime) + ".csv";

		try {
			fileWriter = new FileWriter(fileNameFull, true);
			fileWriter.write("----------------------------\n");
			fileWriter.write(dateFormat.format(System.currentTimeMillis()) + ",FileLog,Open," + fileNameFull + "\n");
			fileWriter.flush();
		} catch (IOException exception) {
			System.out.println("Could not open log file: " + exception);
		}
	}

	/**
	 * Renames them log file name to the current date and time
	 */
	public void updateFilenameDateTime() {
		String fileNameNew;
		File oldFile, newFile;

		// Close the current log file
		try {
			fileWriter.close();
		} catch (IOException exception) {
		}

		// Update startTime and generate the new file name
		startTime = System.currentTimeMillis();
		fileNameNew = fileNameBase + fileDateFormat.format(startTime) + ".csv";

		// Rename the file
		oldFile = new File(fileNameFull);
		newFile = new File(fileNameNew);
		oldFile.renameTo(newFile);

		// Update member variables and open the new file
		fileNameFull = fileNameNew;
		try {
			fileWriter = new FileWriter(fileNameFull, true);
			fileWriter.write("----------------------------\n");
			fileWriter.write(dateFormat.format(System.currentTimeMillis()) + ",FileLog,Rename," + fileNameFull + "\n");
			fileWriter.flush();
		} catch (IOException exception) {
			System.out.println("Could not open log file: " + exception);
		}
	}
	
	/**
	 * Writes a message to the log file.  The message will be timestamped.  Does not echo the message to the screen.
	 * @param subsystemOrCommand The name of the subsytem or command generating the message
	 * @param event A description of the event (ex. start, data, event)
	 * @param msg The message
	 */
	public void writeLog(String subsytemOrCommand, String event, String msg) {
		// If system clock has reset by more than 24 hours (like when the clock is set
		// at the start of a match), then fix the filename
		if (System.currentTimeMillis() - startTime > 1000*3600*24) {
			updateFilenameDateTime();
		}

		// Write the message to the file
		try {
			fileWriter.write(dateFormat.format(System.currentTimeMillis()) + "," + subsytemOrCommand + "," + event + "," + msg + "\n");
			fileWriter.flush();
		} catch (IOException exception) {
		}
	}

	/**
	 * Writes a message to the log file.  The message will be timestamped.  Does not echo the message to the screen.
	 * @param logWhenDisabled true will log when disabled, false will discard the message
	 * @param subsystemOrCommand The name of the subsytem or command generating the message
	 * @param event A description of the event (ex. start, data, event)
	 * @param msg The message
	 */
	public void writeLog(boolean logWhenDisabled, String subsytemOrCommand, String event, String msg) {
		// If system clock has reset by more than 24 hours (like when the clock is set
		// at the start of a match), then fix the filename
		if (System.currentTimeMillis() - startTime > 1000*3600*24) {
			updateFilenameDateTime();
		}

		// Write the message to the file
		if(logWhenDisabled || DriverStation.getInstance().isEnabled()) {
			try {
			fileWriter.write(dateFormat.format(System.currentTimeMillis()) + "," + subsytemOrCommand + "," + event + "," + msg + "\n");
			fileWriter.flush();
			} catch (IOException exception) {
			}
		}
	}
	
	/**
	 * Writes a message to the log file.  The message will be timestamped.  Also echos the message to the screen.
	 * @param subsystemOrCommand The name of the subsytem or command generating the message
	 * @param event A description of the event (ex. start, data, event)
	 * @param msg The message
	 */
	public void writeLogEcho(String subsytemOrCommand, String event, String msg) {
		writeLog(subsytemOrCommand, event, msg);
		System.out.println("Log: " + subsytemOrCommand + "," + event + "," + msg);
	}

	/**
	 * Changes level of detail for fileLog
	 * <ul><li>1 = full debugging logs.  Huge file log, so use sparingly.</li>
	 * <li>2 = normal lab mode.  Moderate logging details.</li>
	 * <li>3 = competition mode.  Minimal logging.</li></ul>
	 * @param level between 1-3, where 1 is the most detailed and 3 is the least detailed.
	 */
	public void setLogLevel(int level) {
		logLevel = level;
		SmartDashboard.putNumber("Log Level", level);
		writeLogEcho("FileLog", "setLogLevel", "Level" + level);
	}

	/**
	 * Returns what level of detail the fileLog should be at (to be called in each subsystem)
	 * <ul><li>1 = full debugging logs.  Huge file log, so use sparingly.</li>
	 * <li>2 = normal lab mode.  Moderate logging details.</li>
	 * <li>3 = competition mode.  Minimal logging.</li></ul>
	 */
	public int getLogLevel() {
		return logLevel;
	}

	/** 
	 * Advances the log rotation counter by one place, and resets if above threshhold for the current log level
	 */
	public void advanceLogRotation() {
		rotation++;
		if (logLevel == 3) {
			if (rotation >= 25) rotation = 0;
		} else {
			if (rotation >= 10) rotation = 0;
		}
	}

	/**
	 * Gets the index of the file log rotation
	 * @return int between 0 and 9 (log levels 1 or 2) or between 0 and 24 (log level 3)
	 */
	public int getLogRotation() {
		return rotation;
	}
	
	/**
	 * Closes the log file.  All writes after closing the log file will be ignored.
	 */
	public void close() {
		try {
			fileWriter.close();
			fileWriter = null;
		} catch (IOException exception) {
		}
	}

}
