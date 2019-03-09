package frc.robot.utilities;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorProfileGenerator {
	
	public double currentMPDistance;  // Current distance travelled in positive motion profile (always positive)
	public double targetMPDistance;	  // Target total distance in positive motion profile
	
	public double initialPosition;   // Initial position in user coords
	public double finalPosition;	// Final position in user coords

	public double currentVelocity;
	public double maxVelocity;

	public double currentAcceleration;
	public double maxAcceleration;
	public double stoppingAcceleration;

	public double dt;
	public double totalTime;
		
	private double directionSign;  // +1 if finalPosition>InitialPosition, -1 if not
	
	private long startTime, currentTime;

	/**
	 * Creates a new profile generator, starting now
	 * @param initialPosition in inches
	 * @param finalPosition in inches
	 * @param initialVelocity in inches per second
	 * @param maxVelocity in inches per second
	 * @param maxAcceleration in inches per second^2
	 */
	public ElevatorProfileGenerator(double initialPosition, double finalPosition, double initialVelocity, 
			double maxVelocity, double maxAcceleration) {
		newProfile( initialPosition, finalPosition, initialVelocity, maxVelocity, maxAcceleration);
	}

	
	/**
	 * Resets profile generator with new parameters, starting now.
	 * This method continues seamlessly with the prior profile.
	 * @param finalPosition in inches
	 * @param maxVelocity in inches per second
	 * @param maxAcceleration in inches per second^2
	 */
	public void newProfile(double finalPosition, double maxVelocity, double maxAcceleration) {
		newProfile(getCurrentPosition(), finalPosition, getCurrentVelocity(), maxVelocity, maxAcceleration);
	}
	
	
	/**
	 * Resets profile generator with new parameters, starting now
	 * @param initialPosition in inches
	 * @param finalPosition in inches
	 * @param initialVelocity in inches per second
	 * @param maxVelocity in inches per second
	 * @param maxAcceleration in inches per second^2
	 */
	public void newProfile(double initialPosition, double finalPosition, double initialVelocity, 
			double maxVelocity, double maxAcceleration) {
		this.initialPosition = initialPosition;
		this.finalPosition = finalPosition;
		
		currentMPDistance = 0;
		targetMPDistance = Math.abs(finalPosition-initialPosition);
		
		directionSign = Math.signum(finalPosition - initialPosition);

		this.currentVelocity = initialVelocity * directionSign;
		this.maxVelocity = Math.abs(maxVelocity);
		this.maxAcceleration = Math.abs(maxAcceleration);
		stoppingAcceleration = .75*maxAcceleration;
		
		// Save starting time
		startTime = System.currentTimeMillis();
		currentTime = startTime;
		
		Robot.log.writeLog("ElevatorProfile", "New Profile", "init pos," + initialPosition + ",final pos," + finalPosition );
	}

	/**
	 * Call this method once per scheduler cycle.  This method calculates the distance that
	 * the robot should have traveled at this point in time, per the motion profile.
	 * Also calculates velocity in in/s
	 */
	public void updateProfileCalcs(){
		long tempTime = System.currentTimeMillis();
		dt = ((double)(tempTime - currentTime))/1000.0;
		currentTime = tempTime;		
		
		double stoppingDistance = 0.5 * currentVelocity * currentVelocity / stoppingAcceleration;
		if((targetMPDistance - currentMPDistance < stoppingDistance) && (currentVelocity > 0)) currentAcceleration = -stoppingAcceleration;
		else if(currentVelocity < maxVelocity) currentAcceleration = maxAcceleration;
		else currentAcceleration = 0;
		
		currentVelocity = currentVelocity + currentAcceleration * dt;
		
		if(currentVelocity > maxVelocity) currentVelocity = maxVelocity;
		currentMPDistance = currentMPDistance + currentVelocity * dt;
		if(currentMPDistance > targetMPDistance) currentMPDistance = targetMPDistance;
		SmartDashboard.putNumber("Profile Position", currentMPDistance);
		SmartDashboard.putNumber("Profile Velocity", currentVelocity);
		Robot.log.writeLog("ElevatorProfile", "updateCalc", "current pos," + (currentMPDistance * directionSign) + ",actualPos," + (Robot.elevator.getElevatorPos()-Robot.robotPrefs.elevatorBottomToFloor) + ",targetPos," + targetMPDistance + ",time since start," + getTimeSinceProfileStart() + ",dt," + dt +
				 ",current vel," + (currentVelocity * directionSign) + ",current acceleration," + currentAcceleration);
		
	}

	/**
	 * Current target position for the robot, in inches
	 * @return Current target position for the robot, in inches
	 */
	public double getCurrentPosition(){
		return currentMPDistance * directionSign + initialPosition;
		
	}

	/**
	 * Returns the time since starting this profile generator
	 * @return
	 */
	public double getTimeSinceProfileStart() {
		return ((double)(currentTime - startTime))/1000.0;
	}
	
	/**
	 * Returns current target velocity in in/s
	 * @return Current target velocity from profile calculation in in/s
	 */
	public double getCurrentVelocity() {
		return currentVelocity * directionSign;
	}
}
