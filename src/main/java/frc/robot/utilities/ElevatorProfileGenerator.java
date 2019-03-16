package frc.robot.utilities;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorProfileGenerator {

	private boolean profileEnabled = false;

	private double currentMPDistance; // Distance that should have been travelled in current motion profile (always positive)
	private double targetMPDistance; // Total distance to be travelled in current motion profile (always positive)

	private double initialPosition; // Initial position in inches from the floor
	private double finalPosition; // Final position in inches from the floor

	private double maxVelocity = 50;
	private double currentVelocity;

	private double maxAcceleration = 100;
	private double stoppingAcceleration = .5 * maxAcceleration;
	private double currentAcceleration;

	private double dt; // delta T (time)

	private double directionSign; // +1 if finalPosition > InitialPosition, -1 if not

	private long startTime, lastTime;

	private double prevError, error, intError;

	private double kVu = 0.0;  // Should be around 0.015
	private double kPu = 0.15;  // was 0.25, 0.05
	private double kIu = 0;
	private double kDu = 0;
	private double kVd = 0.0;   // Should be around 0.012
	private double kPd = 0.05;
	private double kId = 0;
	private double kDd = 0;
	private double kFF = 0.05;  // was 0.1, 0.0
	
	/**
	 * Creates a new profile generator but keeps it in disabled mode
	 */
	public ElevatorProfileGenerator() {
		disableProfileControl();
	}

	/**
	 * disables motion profile's control of motors
	 */
	public void disableProfileControl() {
		profileEnabled = false;
	}

	/**
	 * Sets target position for elevator, using motion profile movement.
	 * This enables the profiler to take control of the elevator, so only call it if the
	 * encoder is working, the elevator is calibrated, and there are no physical obstructions
	 * (check interlocks before calling).
	 * Enables motion profile's control of motors
	 * @param pos in inches from the floor.
	*/
	public void setProfileTarget(double pos) {
		profileEnabled = true;
		finalPosition = pos;
		initialPosition = Robot.elevator.getElevatorPos();
		intError = 0;			// Clear integrated error
		prevError = 0;			// Clear previous error

		// Save starting time
		startTime = System.currentTimeMillis();
		lastTime = startTime;

		SmartDashboard.putNumber("ElevatorInitPos", initialPosition);
		SmartDashboard.putNumber("ElevatorTarget", finalPosition);

		currentMPDistance = 0;
		targetMPDistance = Math.abs(finalPosition - initialPosition);

		directionSign = Math.signum(finalPosition - initialPosition);

		/* TODO uncomment if we decide to have different velocities/accelerations for up vs down
		if(directionSign == 1) {

		} else {

		} */

		// Seed the profile with the current velocity, in case the elevator is already moving
		currentVelocity = Robot.elevator.getElevatorVelocity();

		Robot.log.writeLog("ElevatorProfile", "New Profile", "Init pos," + initialPosition + ",Final pos," + finalPosition);
	}

	/**
	 * Call this method once per scheduler cycle. This method calculates the
	 * distance that the robot should have traveled at this point in time, per the
	 * motion profile. Also calculates velocity in in/s
	 */
	public void updateProfileCalcs() {
		if (Math.abs(currentMPDistance - targetMPDistance) >= 0.25) { 
			// does not continue calculating after we should have reached our target (within a quarter inch)
			long currentTime = System.currentTimeMillis();
			dt = ((double) (currentTime - lastTime)) / 1000.0;
			lastTime = currentTime;

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
							+ ",ActualVel," + Robot.elevator.getElevatorVelocity()
							+ ",MP Vel," + (currentVelocity * directionSign) + ",MP Accel,"
							+ (currentAcceleration * directionSign) );
		} else { // do not change the theoretical distance once it has reached the target range (+/- 0.25 inches)
			currentMPDistance = targetMPDistance;
			currentVelocity = 0;
			currentAcceleration = 0;

			// Robot.log.writeLog("ElevatorProfile", "updateCalcDone",
			// 		"MP Pos," + getCurrentPosition() + ",ActualPos,"
			// 				+ Robot.elevator.getElevatorPos() + ",TargetPos,"
			// 				+ finalPosition + ",Time since start," + getTimeSinceProfileStart() + ",dt," + dt
			// 				+ ",MP Vel," + (currentVelocity * directionSign) + ",MP Accel,"
			// 				+ currentAcceleration);
		}
	}


	/**
	 * Code to make the elevator follow the MotionProfile, should be called exactly once per scheduler cycle.
	 * ONLY follow the MotionProfile if elevPosControl is true, else we should be in manual mode so do nothing.
	 * @return percent power to set the elevator motors to based on calculations
	 */
	public double trackProfilePeriodic() {
		if(profileEnabled) {
			updateProfileCalcs();
			error = getCurrentPosition() - Robot.elevator.getElevatorPos();
			intError = intError + error * dt;

			double percentPower = kFF;
			if (directionSign == 1) {
				percentPower += kVu * currentVelocity + kPu * error + ((error - prevError) * kDu) + (kIu * intError);
			} else if(directionSign == -1) {
				percentPower += -kVd * currentVelocity + kPd * error + ((error - prevError) * kDd) + (kId * intError);
			} 
			prevError = error;

			// If we are using our motion profile control loop, then set the power directly using elevatorMotor1.set().
			// Do not call setElevatorMotorPercentOutput(), since that will change the elevPosControl to false (manual control).
			return percentPower;
		} else {
			return 0.0;
		}
	}


	/**
	 * @return Current target position for the robot, in inches
	 */
	public double getCurrentPosition() {
		return currentMPDistance * directionSign + initialPosition;

	}

	/**
	 * @return Current final position of motion profile
	 */
	public double getFinalPosition() {
		return finalPosition;
	}

	/**
	 * @return time in seconds since starting the current profile
	 */
	public double getTimeSinceProfileStart() {
		return ((double) (lastTime - startTime)) / 1000.0;
	}

	/**
	 * @return Current target velocity from profile calculation in in/s
	 */
	public double getCurrentVelocity() {
		return currentVelocity * directionSign;
	}
}
