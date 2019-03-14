package frc.robot.utilities;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorProfileGenerator {

	public double currentMPDistance; // Distance that should have been travelled in current motion profile (always positive)
	public double targetMPDistance; // Total distance to be travelled in current motion profile (always positive)

	public double initialPosition; // Initial position in inches from the floor
	public double finalPosition; // Final position in inches from the floor

	public double currentVelocity;
	public double maxVelocity;

	public double currentAcceleration;
	public double maxAcceleration;
	public double stoppingAcceleration;

	public double dt; // delta T (time)
	public double totalTime;

	private double directionSign; // +1 if finalPosition > InitialPosition, -1 if not

	private long startTime, currentTime;

	/**
	 * Creates a new profile generator, starting now
	 * @param initialPosition in inches from the floor
	 * @param finalPosition   in inches from the floor
	 * @param initialVelocity in inches per second
	 * @param maxVelocity     in inches per second
	 * @param maxAcceleration in inches per second^2
	 */
	public ElevatorProfileGenerator(double initialPosition, double finalPosition, double initialVelocity,
			double maxVelocity, double maxAcceleration) {
		newProfile(initialPosition, finalPosition, initialVelocity, maxVelocity, maxAcceleration);
	}

	/**
	 * Resets profile generator with new parameters, starting now. This method 
	 * continues seamlessly with the previous profile
	 * @param initialPosition in inches from the floor
	 * @param finalPosition   in inches from the floor
	 * @param initialVelocity in inches per second
	 * @param maxVelocity     in inches per second
	 * @param maxAcceleration in inches per second^2
	 */
	public void newProfile(double initialPosition, double finalPosition, double initialVelocity, double maxVelocity,
			double maxAcceleration) {
		this.initialPosition = initialPosition;
		this.finalPosition = finalPosition;

		currentMPDistance = 0;
		targetMPDistance = Math.abs(finalPosition - initialPosition);

		directionSign = Math.signum(finalPosition - initialPosition);

		this.currentVelocity = initialVelocity * directionSign;
		this.maxVelocity = Math.abs(maxVelocity);
		this.maxAcceleration = Math.abs(maxAcceleration);
		stoppingAcceleration = .5 * maxAcceleration;

		// Save starting time
		startTime = System.currentTimeMillis();
		currentTime = startTime;

		Robot.log.writeLog("ElevatorProfile", "New Profile", "Init pos," + initialPosition + ",Final pos," + finalPosition);
	}

	/**
	 * Call this method once per scheduler cycle. This method calculates the
	 * distance that the robot should have traveled at this point in time, per the
	 * motion profile. Also calculates velocity in in/s
	 */
	public void updateProfileCalcs() {
		if (Math.abs((currentMPDistance * directionSign) - targetMPDistance) >= 0.25) { 
			// does not continue calculating after we should have reached our target (within a quarter inch)
			long tempTime = System.currentTimeMillis();
			dt = ((double) (tempTime - currentTime)) / 1000.0;
			currentTime = tempTime;

			double stoppingDistance = 0.5 * currentVelocity * currentVelocity / stoppingAcceleration;

			// calculating target acceleration
			if (((targetMPDistance - currentMPDistance) < stoppingDistance) && (currentVelocity > 0)) {
				currentAcceleration = -stoppingAcceleration;
			}
			else if (currentVelocity < maxVelocity) {
				currentAcceleration = maxAcceleration;
			}
			else {
				currentAcceleration = 0;
			}

			// calculating target velocity
			currentVelocity = currentVelocity + currentAcceleration * dt;
			if (currentVelocity > maxVelocity) {
				currentVelocity = maxVelocity;
			}

			// calculating the distance the elevator should have travelled
			currentMPDistance = currentMPDistance + currentVelocity * dt;
			if (currentMPDistance > targetMPDistance) {
				currentMPDistance = targetMPDistance;
			}

			// SmartDashboard.putNumber("Profile Position", currentMPDistance);
			// SmartDashboard.putNumber("Profile Velocity", currentVelocity);
			Robot.log.writeLog("ElevatorProfile", "updateCalc",
					"MP Pos," + getCurrentPosition() + ",ActualPos,"
							+ Robot.elevator.getElevatorPos() + ",TargetPos,"
							+ finalPosition + ",Time since start," + getTimeSinceProfileStart() + ",dt," + dt
							+ ",MP Vel," + (currentVelocity * directionSign) + ",MP Accel,"
							+ currentAcceleration);
		} else { // do not change the theoretical distance once it has reached the target range (+/- 0.25 inches)
			currentMPDistance = targetMPDistance;
		}
	}

	/**
	 * Current target position for the robot, in inches
	 * @return Current target position for the robot, in inches
	 */
	public double getCurrentPosition() {
		return currentMPDistance * directionSign + initialPosition;

	}

	/**
	 * @return time in seconds since starting the current profile
	 */
	public double getTimeSinceProfileStart() {
		return ((double) (currentTime - startTime)) / 1000.0;
	}

	/**
	 * @return Current target velocity from profile calculation in in/s
	 */
	public double getCurrentVelocity() {
		return currentVelocity * directionSign;
	}
}
