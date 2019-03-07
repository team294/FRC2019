/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveProfileGenerator {
	
	public double currentMPDistance;  // Current distance travelled in positive motion profile (always positive)
	public double targetMPDistance;	  // Target total distance in positive motion profile
	
	public double initialPosition;   // Initial position in user coords
	public double finalPosition;	// Final position in user coords

	public double currentVelocity;
	public double maxVelocity;

	public double currentAcceleration;
	public double maxAcceleration;

	public double dt;
	public double totalTime;
	
	public double stoppingDistance;
	public boolean doneFlag;
		
	private double directionSign;  // +1 if finalPoistion>InitialPosition, -1 if not
	
	private long startTime, currentTime;

	/**
	 * Creates a new profile generator, starting now
	 * @param initialPosition in inches
	 * @param finalPosition in inches
	 * @param initialVelocity in inches per second
	 * @param maxVelocity in inches per second
	 * @param maxAcceleration in inches per second^2
	 */
	public DriveProfileGenerator(double initialPosition, double finalPosition, double initialVelocity, 
			double maxVelocity, double maxAcceleration) {
		this.initialPosition = initialPosition;
		this.finalPosition = finalPosition;
		
		currentMPDistance = 0;
		targetMPDistance = Math.abs(finalPosition-initialPosition);
		
		directionSign = Math.signum(finalPosition - initialPosition);

		this.currentVelocity = initialVelocity;
		this.maxVelocity = Math.abs(maxVelocity);
		this.maxAcceleration = maxAcceleration;
		
		// Save starting time
		startTime = System.currentTimeMillis();
		currentTime = startTime;
		
		stoppingDistance = 0;
		doneFlag = false;
		
		Robot.log.writeLog(false, "Profile generator", "Update", "Init Pos," + initialPosition + ",Final Pos," + finalPosition);
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
		double stoppingVelocity = currentVelocity + dt*maxAcceleration; 
		stoppingDistance = 0.5*stoppingVelocity*stoppingVelocity/maxAcceleration;
		if(targetMPDistance - currentMPDistance < stoppingDistance) currentAcceleration = -maxAcceleration;
		else if(currentVelocity < maxVelocity) currentAcceleration = maxAcceleration;
		else currentAcceleration = 0;
		
		/*if(currentMPDistance <= 0.5*targetMPDistance)
		{
			currentMPDistance = currentMPDistance + currentVelocity*dt;
			currentVelocity = currentVelocity + maxAcceleration*dt;
			if(currentVelocity >= maxVelocity)
			{
				if(stoppingDistance == 0)
					stoppingDistance = currentMPDistance;
				currentVelocity = maxVelocity;
			}else
			{
				currentMPDistance = currentMPDistance + 0.5*maxAcceleration*dt*dt;
			}
		} else
		{
			if(targetMPDistance - currentMPDistance <= stoppingDistance || stoppingDistance == 0) {
				currentMPDistance = currentMPDistance + currentVelocity*dt - 0.5*maxAcceleration*dt*dt;
				currentVelocity = currentVelocity - maxAcceleration*dt;
			}else
			{
				currentMPDistance = currentMPDistance + currentVelocity*dt;
			}
		}*/
		
		currentVelocity = currentVelocity + currentAcceleration*dt;
		
		if(currentVelocity > maxVelocity) currentVelocity = maxVelocity;
		currentMPDistance = currentMPDistance + currentVelocity*dt;
		if(currentMPDistance >= targetMPDistance-0.05 || doneFlag) 
		{
			doneFlag = true;
			currentMPDistance = targetMPDistance;
			currentVelocity = 0;
		}
		SmartDashboard.putNumber("Profile Position", currentMPDistance);
		SmartDashboard.putNumber("Profile Velocity", currentVelocity);
	}

	/**
	 * Current target position for the robot, in inches
	 * @return Current target position for the robot, in inches
	 */
	public double getCurrentPosition(){
		return currentMPDistance*directionSign + initialPosition;
		
	}

	/**
	 * Returns the time since starting this profile generator
	 * @return
	 */
	public double getTimeSinceProfileStart() {
		return ((double)(currentTime - startTime))/1000.0;
	}
	
	/**
	 * Returns current velocity in in/s
	 * @return Current velocity for profile calculation in in/s
	 */
	public double getCurrentVelocity(){
		return currentVelocity * directionSign;
	}
}
