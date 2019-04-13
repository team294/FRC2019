/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.LinkedList;
import java.util.Iterator;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.utilities.FileLog;

/**
 * This is the parent class for a drive train. All the logic for line following, vision, etc. is here.
 * Cim- and neo-specific code for encoders and motors goes in the respective drive train classes, which
 * must implement all the abstract methods below.
 */
public abstract class DriveTrain extends Subsystem {

  private AHRS ahrs;
  private double yawZero = 0;
  
  private double leftMotorFaultCount; // increments every cycle the left side detects an issue
  private double rightMotorFaultCount; // increments every cycle the right side detects an issue
  boolean driveDirection = true; // true = forward, false = reverse
  private double fieldX;
  private double fieldY;
  
  private double priorTurnPercentOutput = 0.0;
  
  // Encoders
  private double leftEncoderZero = 0, rightEncoderZero = 0;

  // Encoders
  private LinkedList<Double> lEncoderStack = new LinkedList<Double>();
  private LinkedList<Double> rEncoderStack = new LinkedList<Double>();
  private boolean lEncStopped = false, rEncStopped = false;

  public DriveTrain() {
    // Configure navX
		try {
			/* Communicate w/navX MXP via the MXP SPI Bus.
			 * Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB
			 * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
			 * details.
			 */

			ahrs = new AHRS(I2C.Port.kMXP);

		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
		ahrs.zeroYaw();
    // zeroGyroRotation();
  }

  abstract public void tankDrive(double leftPercent, double rightPercent);

  /**
   * Set the left motors only
   * @param percent [-1.0, 1.0]
   */
  abstract public void setLeftMotors(double percent);

  /**
   * Set the right motors only
   * @param percent [-1.0, 1.0 ]
   */
  abstract public void setRightMotors(double percent);

  /**
	 * Sets the robot to drive at a curve.
	 * 
	 * @param speedPct
	 *            Percent output of motor -1.0 to 1.0
	 * @param curve
	 *            the rate at which the robot will curve -1.0 to 1.0. Clockwise is
	 *            positive.
	 */
  abstract public void driveAtCurve(double speedPct, double curve);

  /**
	 * Turns voltage compensation on or off for drive motors.
	 * Voltage compensation increases accuracy for autonomous code,
	 * but it decreases maximum velocity/power when driving by joystick.
	 * @param turnOn true=turn on, false= turn off
	 */
  abstract public void setVoltageCompensation(boolean turnOn);

  abstract public double getLeftEncoderRaw();

  abstract public double getRightEncoderRaw();

  abstract public double getLeftEncoderVelocityRaw();

  abstract public double getRightEncoderVelocityRaw();

  /**
	 * Zeros the left encoder position in software
	 */
  public void zeroLeftEncoder() {
    leftEncoderZero = getLeftEncoderRaw();
  }

  /**
	 * Zeros the right encoder position in software
	 */
  public void zeroRightEncoder() {
    rightEncoderZero = getRightEncoderRaw();
  }

  /**
	 * Get the position of the left encoder, in encoder ticks since last zeroLeftEncoder()
	 * 
	 * @return encoder position, in ticks
	 */
  public double getLeftEncoderTicks() {
    return getLeftEncoderRaw() - leftEncoderZero;
  }

  /**
	 * Get the position of the right encoder, in encoder ticks since last zeroRightEncoder()
	 * 
	 * @return encoder position, in ticks
	 */
  public double getRightEncoderTicks() {
    return -(getRightEncoderRaw() - rightEncoderZero);
  }

  public double encoderTicksToInches(double ticks) {
    return ticks * Robot.robotPrefs.wheelCircumference / Robot.robotPrefs.encoderTicksPerRevolution;
  }

  public double getLeftEncoderInches() {
    return encoderTicksToInches(getLeftEncoderTicks());
  }

  public double getRightEncoderInches() {
    return encoderTicksToInches(getRightEncoderTicks());
  }

  /**
   * @return left encoder velocity in inches / sec
   */
  public double getLeftEncoderVelocity() {
    return encoderTicksToInches(getLeftEncoderVelocityRaw()) * 10;
  }

    /**
   * @return right encoder velocity in inches / sec
   */
  public double getRightEncoderVelocity() {
    return encoderTicksToInches(getRightEncoderVelocityRaw()) * 10;
  }

  public double inchesToEncoderTicks(double inches) {
    return (inches / Robot.robotPrefs.wheelCircumference) * Robot.robotPrefs.encoderTicksPerRevolution;
  }

  /**
   * 
   * @param setCoast true if want to put driveTrain in coast mode false to put in brake mode.
   */
  abstract public void setDriveMode(boolean setCoast);

  /**
   * Gets the raw value of the gyro
   * @return
   */
  public double getGyroRaw() {
    return ahrs.getAngle();
  }

  /**
	 * Zeros the gyro position in software
	 */
	public void zeroGyroRotation() {
		// set yawZero to gryo angle
		yawZero = getGyroRaw();
		// System.err.println("PLZ Never Zero the Gyro Rotation it is not good");
	}

  /**
	 * Resets the gyro position in software to a specified angle
	 * 
	 * @param currentHeading Gyro heading to reset to, in degrees
	 */
	public void setGyroRotation(double currentHeading) {
		// set yawZero to gryo angle, offset to currentHeading
		yawZero = getGyroRaw() - currentHeading;
		// System.err.println("PLZ Never Zero the Gyro Rotation it is not good");
  }

  /**
	 * Gets the rotation of the gyro
	 * 
	 * @return Current angle from -180 to 180 degrees
	 */
	public double getGyroRotation() {
		double angle = getGyroRaw() - yawZero;
		// Angle will be in terms of raw gyro units (-inf,inf), so you need to convert
		// to (-180, 180]
		angle = normalizeAngle(angle);
		return angle;
  }

  /**
	 * Converts the input angle to a number between -179.999 and +180.0
	 * 
	 * @return Normalized angle
	 */
	public double normalizeAngle(double angle) {
		angle = angle % 360;
		angle = (angle <= -180) ? (angle + 360) : angle;
    angle = (angle > 180) ? (angle - 360) : angle;
		return angle;
  }

  /**
   * Stops the motors by calling tankDrive(0, 0)
   */
  public void stop() {
    tankDrive(0, 0);
  }

  /**
   * Empties the encoder tracking stack and zeroes the left and right encoders
   */
  public void clearEncoderList() {
    Robot.log.writeLogEcho("DriveTrain", "Encoders Cleared", "");
    lEncoderStack.clear();
    rEncoderStack.clear();
    zeroLeftEncoder();  // Theoretically these don't need to be zeroed; the stack just adds their values
    zeroRightEncoder();
  }

  /**
   * Averages the ticks of the left and right encoder and adds them to the encoder stacks.
   * Also removes the earliest element if above 50 elements.
   */
  public void updateEncoderList() {
    lEncoderStack.add(getLeftEncoderTicks());
    rEncoderStack.add(getRightEncoderTicks());
    if (lEncoderStack.size() > 50) {
      lEncoderStack.remove();
      rEncoderStack.remove();
    }
  }

  public double getAverageDistance() {
		return (getRightEncoderInches() + getLeftEncoderInches()) / 2.0;
  }

  /**
   * Checks if both encoders are turning. Make sure you have been calling updateEncoderList enough times before.
   * @param precision Precision, in ticks (i.e. number of ticks by which the average can differ from the last reading)
   * @return true if the difference between the average and the last element is less than the precision specified (this means both encoders are stopped)
   */
  public boolean areEncodersStopped(double precision) {
    //TODO Should the next line have comparison operators "==" instead of assignment operators "="?
    if (lEncoderStack.size()<50) return (lEncStopped = false) || (rEncStopped = false); // Sets both to false while returning.
    double lSum = 0.0, rSum = 0.0;
    Iterator<Double> lIterator = lEncoderStack.descendingIterator();
    Iterator<Double> rIterator = rEncoderStack.descendingIterator();
    while(lIterator.hasNext()) {
      lSum += lIterator.next();
      rSum += rIterator.next();
    }
    return ((lEncStopped = Math.abs(lSum/lEncoderStack.size()-lEncoderStack.peekLast()) <= precision) & (rEncStopped = Math.abs(rSum/rEncoderStack.size()-rEncoderStack.peekLast()) <= precision));
  }
  
  /**
   * Checks drive motor currents, records sticky faults if a motor is faulty for more than 5 cycles
   * @param motor1PDP RobotMap PDP address for motor1
   * @param motor2PDP RobotMap PDP address for motor2
   * @param motor3PDP RobotMap PDP address for motor3
   * @param side true is left, false is right
   */
	public void verifyMotors(int motor1PDP, int motor2PDP, int motor3PDP, boolean side) {
      double amps1 = Robot.pdp.getCurrent(motor1PDP);
      double amps2 = Robot.pdp.getCurrent(motor2PDP);
      double amps3 = Robot.pdp.getCurrent(motor3PDP);
      double averageAmps = (amps1 + amps2 + amps3) / 3;

		if(leftMotorFaultCount >= 5) {
      Robot.robotPrefs.recordStickyFaults("Left" + " Drive Train");
      leftMotorFaultCount = 0;
    } else if (rightMotorFaultCount >= 5) {
      Robot.robotPrefs.recordStickyFaults("Right" + " Drive Train");
      rightMotorFaultCount = 0;
    }
		if(averageAmps > 7) {
			if(amps1 < 4 || amps2 < 4 || amps3 < 4) {
        if(side) leftMotorFaultCount++;
        else  rightMotorFaultCount++;
      }
      else {
        if (side) leftMotorFaultCount = 0;
        else rightMotorFaultCount = 0;
      }
    }
  }

  /**
    * Writes information about the subsystem to the filelog
    * @param logWhenDisabled true will log when disabled, false will discard the string
    */
  abstract void updateDriveLog(boolean logWhenDisabled);

  /**
   * Gets the predicted scoring quadrant of the robot based on what the gyro currently reads
   * @return a quadrant (corresponding to the unit circle) with axes in between quadrants numbered as x.5 values.
   */
  public double checkScoringQuadrant() {
    // assuming the same quadrants as a unit circle, with 0 being straight up (+y axis) and -180 or 180 being straight down (-y axis)
    double quadrant = 0.0;
    double gyroAngle = getGyroRotation();

    if (Math.abs(gyroAngle) <= 5) { // Should mean straight up, +y axis, cardianl durection North
      quadrant = 1.5; // in between quadrants 1 and 2
    } else if (Math.abs(gyroAngle) >= 175) { // Within 5 degrees of -y axis
      quadrant = 3.5; // in between quadrants 3 and 4
    } else if (gyroAngle >= 85 && gyroAngle <= 95) { // Within 5 degrees of +x axis
      quadrant = 0.5; // in between quadrants 4 and 1
    } else if (gyroAngle <= -85 && gyroAngle >= -95) { // Wihin 5 degrees of -x axis
      quadrant = 2.5; // in between quadrants 2 and 3
    } else if (gyroAngle > 90) {
      quadrant = 4;
    } else if (gyroAngle > 0) {
      quadrant = 1; // since all Q4 have already returned, only positive angles left are Q1
    } else if (gyroAngle < -90) {
      quadrant = 3;
    } else if (gyroAngle < 0) {
      quadrant = 2; // only negative angles left are Q2
    }
    
    return quadrant;
  }

  /**
   * Returns the target angle we expect to try to score at
   * @param quadrant scoring quadrant from 0.5 to 4 (inclusive) from checkScoringQuadrant()
   * @return double in the range (-180, 180] (in actual practice, a small array of standard values)
   * </br> The default value is 0.
   */
  public double getTargetAngle(double quadrant) {
    double out = 0.0;
    if (quadrant == 1) out = 28.75;
    else if (quadrant == 1.5) out = 0;
    else if (quadrant == 2) out = -28.75;
    else if (quadrant == 2.5) out = -90;
    else if (quadrant == 3) out = -151.75;
    else if (quadrant == 3.5) out = 180;
    else if (quadrant == 4) out = 151.75;
    else if (quadrant == 4.5 || quadrant == 0.5) out = 90;
    return out;
  }

  public double getTargetAngle() {
    return getTargetAngle(checkScoringQuadrant());
  }

  /**
   * Prepares turnWithGyro to run.
   */
  public void turnwithGyroReset() {
    priorTurnPercentOutput = 0.0;
  }

  /**
   * Turns in place to absolute gyro target angle.  Turns in either direction, choosing
   * whichever is the shorter path.  Call this repeatedly in a periodic method to continue
   * or complete the turn.
   * @param targetAngle Angle to turn to, in degrees
  */
  public void turnWithGyro(double targetAngle) {
    double gainConstant, fixSpeed;

    gainConstant = Robot.shifter.isShifterInHighGear() ? 0.002 : 0.005;  
    fixSpeed = Robot.shifter.isShifterInHighGear() ? 0.05 : 0.1; 

    double xVal = normalizeAngle(targetAngle - getGyroRotation());
    double percentOutput = fixSpeed + Math.abs(gainConstant * xVal);

    if (percentOutput - priorTurnPercentOutput > 0.005) {
      // Prevent the motors from accelerating too quickly and causing the wheels to slip
      percentOutput += 0.005;
    }
    priorTurnPercentOutput = percentOutput;

    if(xVal > 0.5){
      setLeftMotors(-percentOutput);
      setRightMotors(percentOutput);
    } else if (xVal < -0.5) {
      setLeftMotors(percentOutput);
      setRightMotors(-percentOutput);
    } else {
      stop();
    }

    updateDriveLog(false);
  }

  public void driveOnLine() {
    
    SmartDashboard.putString("Line Following Routine", "center");

    double lPercentPower = 0;
    double rPercentPower = 0;
    double baseSpeed = 0.7; // Lowered from 1 for new line sensors further out, untested

    int lineNum = Robot.lineFollowing.getLineNumber();
    //System.out.println("Line Number:" + lineNum);
    if (lineNum == 0) {
      // Straight
      //lPercentPower = .55*baseSpeed;
      //rPercentPower = .55*baseSpeed;
      lPercentPower = 0.65*baseSpeed;
      rPercentPower = 0.65*baseSpeed;
    } else if (lineNum == 1) {
      // Turn left slight?
      lPercentPower = .6*baseSpeed;
      rPercentPower = 0*baseSpeed;
    } else if (lineNum == -1) {
      // Turn right slight?
      lPercentPower = 0*baseSpeed;
      rPercentPower = .6*baseSpeed;
    } else if (lineNum == -2) {
      // Turn left
      lPercentPower = .8*baseSpeed;
      rPercentPower = -.8*baseSpeed;
    } else if (lineNum == 2) {
      // Turn right
      lPercentPower = -.8*baseSpeed;
      rPercentPower = .8*baseSpeed;
    } else {
      // Drive forwards in hopes of recovering the line?
      lPercentPower = 0.3*baseSpeed;
      rPercentPower = 0.3*baseSpeed;
    }

    Robot.log.writeLogEcho("DriveTrain", "Line Tracking", "Line Number," + lineNum + ",Left Percent," + lPercentPower + ",Right Percent," + rPercentPower);

    /* Untested auto-turn stuff */
    if (lEncStopped && lPercentPower != 0) rPercentPower = 1.0; // The goal here is to slam the right side so that we still line up to the wall
    if (rEncStopped && rPercentPower != 0) lPercentPower = 1.0;
    if (lPercentPower == 1 || rPercentPower == 1) System.out.println("STOP DETECTED, INITIATING EVASIVE MANEUVERS"); 

    tankDrive(lPercentPower, rPercentPower);
    updateEncoderList();
  }

  public void quadrantLineFollowing(double quadrant) {
    if (quadrant == 0.0) {
      driveOnLine();
      return;
    }

    //quadrant = 1.5; // TEMPORARY FOR TESTING LEFT/RIGHT

    // add a special case for straight down (quadrant 3.5) because of roll over on 180
    double angleTolerance = 1.5; // degrees
    boolean left = getGyroRotation() > getTargetAngle(quadrant) + angleTolerance; // approaching from left, facing too far right
    boolean right = getGyroRotation() < getTargetAngle(quadrant) - angleTolerance; // approaching from right
    if (!left && !right) {
      driveOnLine(); // if we're close to center just do the simple line following
      return;
    }

    double baseSpeed = 0.8; // Lowered from 1 for new line sensors further out, untested
    int lineNum = Robot.lineFollowing.getLineNumber();
    double lPercentPower = 0.0, rPercentPower = 0.0;

    SmartDashboard.putString("Line Following Routine", (left) ? "left" : "right");

    if (left) {
      if (lineNum == 0) {
        lPercentPower = 0.65*baseSpeed;
        rPercentPower = 0.65*baseSpeed;
      } else if (lineNum == 1) {
        lPercentPower = 0.4*baseSpeed;
        rPercentPower = 0.6*baseSpeed;
      } else if (lineNum == -1) {
        lPercentPower = 0.6*baseSpeed;
        rPercentPower = 0.65*baseSpeed;
      } else if (lineNum == -2) {
        lPercentPower = 0.6*baseSpeed; // originally less
        rPercentPower = 0.65*baseSpeed;
      } else if (lineNum == 2) {
        lPercentPower = -0.4*baseSpeed;
        rPercentPower = 0.8*baseSpeed;
      } else {
        lPercentPower = -0.3*baseSpeed;
        rPercentPower = 0.5*baseSpeed;
      }
    } else { // right
      if (lineNum == 0) {
        lPercentPower = 0.65*baseSpeed;
        rPercentPower = 0.65*baseSpeed;
      } else if (lineNum == 1) {
        lPercentPower = 0.65*baseSpeed;
        rPercentPower = 0.6*baseSpeed;
      } else if (lineNum == -1) {
        lPercentPower = 0.6*baseSpeed;
        rPercentPower = 0.4*baseSpeed;
      } else if (lineNum == -2) {
        lPercentPower = 0.8*baseSpeed;
        rPercentPower = -0.4*baseSpeed;
      } else if (lineNum == 2) {
        lPercentPower = 0.65*baseSpeed;
        rPercentPower = 0.6*baseSpeed; // originally less
      } else {
        lPercentPower = 0.5*baseSpeed;
        rPercentPower = -0.3*baseSpeed;
      }
    }

    Robot.log.writeLog(false, "DriveTrain", "Line Tracking", "Quadrant," + quadrant + ",Left," + left + ",Right," + right + ",Line Number," + lineNum + ",Left Percent," + lPercentPower + ",Right Percent," + rPercentPower);

    /* Untested auto-turn stuff */
    if (lEncStopped && lPercentPower != 0) rPercentPower = 1.0; // The goal here is to slam the right side so that we still line up to the wall
    if (rEncStopped && rPercentPower != 0) lPercentPower = 1.0;
    if (lPercentPower == 1 || rPercentPower == 1) System.out.println("STOP DETECTED, INITIATING EVASIVE MANEUVERS"); 

    tankDrive(lPercentPower, rPercentPower);
    updateEncoderList();
  }

  /**
   * 
   * @param direction true = forward; false = reverse
   */
  public void setDriveDirection(boolean direction) {
    this.driveDirection = direction;
  }

  /**
   * 
   * @return drive direction (true = forward, false = reverse)
   */
  public boolean getDriveDirection() {
    return driveDirection;
  }

  public void setFieldPositionX(double x) {
		this.fieldX = x;
		SmartDashboard.putNumber("FieldX", fieldX);
	}

	public void setFieldPositionY(double y) {
		this.fieldY = y;
		SmartDashboard.putNumber("FieldY", fieldY);
	}

	public void addFieldPositionX(double x) {
		setFieldPositionX(fieldX + x);
	}

	public void addFieldPositionY(double y) {
		setFieldPositionY(fieldY + y);
	}

	public double getFieldPositionX() {
		return this.fieldX;
	}

	public double getFieldPositionY() {
		return this.fieldY;
  } 
  
  /**
	 * Get the average position of the two encoders, in inches
	 * 
	 * @return encoder position, in inches
	 */
	public double getAverageEncoderInches() {
		return (getRightEncoderInches() + getLeftEncoderInches()) / 2.0;
	}

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoysticks());
  }

  @Override
  public void periodic() {

    // updateDriveLog(false);

    if (Robot.log.getLogRotation() == FileLog.DRIVE_CYCLE) {
      SmartDashboard.putNumber("Drive Left Inches", getLeftEncoderInches());
      SmartDashboard.putNumber("Drive Right Inches", getRightEncoderInches());
      SmartDashboard.putNumber("Gyro Angle", getGyroRotation());
      SmartDashboard.putNumber("Drive Left Ticks", getLeftEncoderTicks());
      SmartDashboard.putNumber("Drive Right Ticks", getRightEncoderTicks());

      
      updateDriveLog(false);

      if (DriverStation.getInstance().isEnabled()) {
        Robot.lineFollowing.logLineFollowers();

        // TODO move verifyMotors to a pit command, instead of a live command during a match
        verifyMotors(RobotMap.leftMotor1PDP, RobotMap.leftMotor2PDP, RobotMap.leftMotor3PDP, true);
        verifyMotors(RobotMap.rightMotor1PDP, RobotMap.rightMotor2PDP, RobotMap.rightMotor3PDP, false);
      }
    }
  }
}
