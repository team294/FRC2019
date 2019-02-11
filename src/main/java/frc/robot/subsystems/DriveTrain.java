/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.subsystems;   // had to change from just frc.robot.subsystems ?


import java.util.Iterator;
import java.util.LinkedList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;



import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoysticks;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.I2C;

/**
 * Drive Train subsystem.  
 */
public class DriveTrain extends Subsystem {
  private final BaseMotorController leftMotor1;
  private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(RobotMap.leftMotor2);
  private final BaseMotorController leftMotor3;
  private final BaseMotorController rightMotor1;
  private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RobotMap.rightMotor2);
  private final BaseMotorController rightMotor3;

  public final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor2, rightMotor2);

  // Gyro variables
  private AHRS ahrs;
  private double yawZero = 0;
  
  private int periodicCount = 0;
  
  private double leftMotorFaultCount; // increments every cycle the left side detects an issue
  private double rightMotorFaultCount; // increments every cycle the right side detects an issue
  
  // Encoders
  private double leftEncoderZero = 0, rightEncoderZero = 0;
  private LinkedList<Double> lEncoderStack = new LinkedList<Double>();
  private LinkedList<Double> rEncoderStack = new LinkedList<Double>();
  private boolean lEncStopped = false, rEncStopped = false;

  public DriveTrain() {

    // Reads the robot prefs to check if using Victors or Talons for alternate motor controllers
    if (Robot.robotPrefs.prototypeRobot) { // true means Talons
      leftMotor1 = new WPI_TalonSRX(RobotMap.leftMotor1);
      leftMotor3 = new WPI_TalonSRX(RobotMap.leftMotor3);
      rightMotor1 = new WPI_TalonSRX(RobotMap.rightMotor1);
      rightMotor3 = new WPI_TalonSRX(RobotMap.rightMotor3);

      leftMotor1.set(ControlMode.Follower, RobotMap.leftMotor2);
      leftMotor3.set(ControlMode.Follower, RobotMap.leftMotor2);
      rightMotor1.set(ControlMode.Follower, RobotMap.rightMotor2);
      rightMotor3.set(ControlMode.Follower, RobotMap.rightMotor2);
    }
    else {
      leftMotor1 = new WPI_VictorSPX(RobotMap.leftMotor1);
      leftMotor3 = new WPI_VictorSPX(RobotMap.leftMotor3);
      rightMotor1 = new WPI_VictorSPX(RobotMap.rightMotor1);
      rightMotor3 = new WPI_VictorSPX(RobotMap.rightMotor3);

      leftMotor1.follow(leftMotor2);
      leftMotor3.follow(leftMotor2);
      rightMotor1.follow(rightMotor2);
      rightMotor3.follow(rightMotor2);
    }
    leftMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		rightMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		zeroLeftEncoder();
		zeroRightEncoder();

    leftMotor1.setInverted(true);
    leftMotor2.setInverted(true);
    leftMotor3.setInverted(true);
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);
    rightMotor3.setInverted(true);

    leftMotor1.clearStickyFaults(0);
    leftMotor2.clearStickyFaults(0);
    leftMotor3.clearStickyFaults(0);
    rightMotor1.clearStickyFaults(0);
    rightMotor2.clearStickyFaults(0);
    rightMotor3.clearStickyFaults(0);

    leftMotor1.setNeutralMode(NeutralMode.Brake);
    leftMotor2.setNeutralMode(NeutralMode.Brake);
    leftMotor3.setNeutralMode(NeutralMode.Brake);
    rightMotor1.setNeutralMode(NeutralMode.Brake);
    rightMotor2.setNeutralMode(NeutralMode.Brake);
    rightMotor3.setNeutralMode(NeutralMode.Brake);

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

  public void tankDrive (double powerLeft, double powerRight) {
    robotDrive.tankDrive(powerLeft, powerRight);
  }

  /**
	 * Sets the robot to drive at a curve.
	 * 
	 * @param speedPct
	 *            Percent output of motor -1.0 to 1.0
	 * @param curve
	 *            the rate at which the robot will curve -1.0 to 1.0. Clockwise is
	 *            positive.
	 */
	public void driveAtCurve(double speedPct, double curve) {
		robotDrive.curvatureDrive(speedPct, curve, false);
  }
  
  /**
   * Drives the robot along a curve
   * @param speedPct Percent output of motor [-1.0, 1.0]
   * @param rotation Rotation rate (rate of heading change) from [-1.0, 1.0]
   */
  public void driveItLikeYouStoleIt(double speedPct, double rotation) {
    robotDrive.arcadeDrive(speedPct, rotation, false);
  }

  /**
   * Stops the motors by calling tankDrive(0, 0)
   */
  public void stop() {
    tankDrive(0, 0);
  }

  /**
	 * Set the percent output of the left motor.
	 * 
	 * @param powerPct Percent of power -1.0 (reverse) to 1.0 (forward)
	 */
	public void setLeftMotors(double powerPct) {
		//TODO check if direction forward/backward is correct
		leftMotor2.set(ControlMode.PercentOutput, -powerPct);
	}

	/**
	 * Set the percent output of the right motor.
	 * 
	 * @param powerPct Percent of power -1.0 (reverse) to 1.0 (forward)
	 */
	public void setRightMotors(double powerPct) {
		//TODO check if direction forward/backward is correct
		rightMotor2.set(ControlMode.PercentOutput, powerPct);
  }

  /**
	 * Stops all motors.
	 */
	public void stopAllMotors() {
		setLeftMotors(0);
		setRightMotors(0);
	}
  
  /**
	 * Turns voltage compensation on or off for drive motors.
	 * Voltage compensation increases accuracy for autonomous code,
	 * but it decreases maximum velocity/power when driving by joystick.
	 * @param turnOn true=turn on, false= turn off
	 */
	public void setVoltageCompensation(boolean turnOn) {
		leftMotor1.enableVoltageCompensation(turnOn);
		leftMotor2.enableVoltageCompensation(turnOn);
		leftMotor3.enableVoltageCompensation(turnOn);
		rightMotor1.enableVoltageCompensation(turnOn);
		rightMotor2.enableVoltageCompensation(turnOn);
		rightMotor3.enableVoltageCompensation(turnOn);
	}
  
  /**
	 * Zeros the left encoder position in software
	 */
	public void zeroLeftEncoder() {
		leftEncoderZero = leftMotor2.getSelectedSensorPosition(0);
	}

	/**
	 * Zeros the right encoder position in software
	 */
	public void zeroRightEncoder() {
    rightEncoderZero = rightMotor2.getSelectedSensorPosition(0);
	}

	/**
	 * Get the position of the left encoder, in encoder ticks since last zeroLeftEncoder()
	 * 
	 * @return encoder position, in ticks
	 */
	public double getLeftEncoderTicks() {
    return leftMotor2.getSelectedSensorPosition(0) - leftEncoderZero;
	}

	/**
	 * Get the position of the right encoder, in encoder ticks since last zeroRightEncoder()
	 * 
	 * @return encoder position, in ticks
	 */
	public double getRightEncoderTicks() {
		return -(rightMotor2.getSelectedSensorPosition(0) - rightEncoderZero);
	}

  public double encoderTicksToInches(double encoderTicks) {
    return (encoderTicks / Robot.robotPrefs.encoderTicksPerRevolution) * Robot.robotPrefs.wheelCircumference ;
  }
  public double inchesToEncoderTicks(double inches) {
    return (inches / Robot.robotPrefs.wheelCircumference) * Robot.robotPrefs.encoderTicksPerRevolution;
  }
  public double getLeftEncoderInches() {
    SmartDashboard.putNumber("Left Inches", encoderTicksToInches(getLeftEncoderTicks()));
    return encoderTicksToInches(getLeftEncoderTicks());
  }

  public double getRightEncoderInches() {
    SmartDashboard.putNumber("Right Inches", encoderTicksToInches(getRightEncoderTicks()));
    return encoderTicksToInches(getRightEncoderTicks());
  }
  
  /**
   * Empties the ecoder tracking stack and zeroes the left and right encoders
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
	 * Zeros the gyro position in software
	 */
	public void zeroGyroRotation() {
		// set yawZero to gryo angle
		yawZero = ahrs.getAngle();
		// System.err.println("PLZ Never Zero the Gyro Rotation it is not good");
	}

	/**
	 * Resets the gyro position in software to a specified angle
	 * 
	 * @param currentHeading Gyro heading to reset to, in degrees
	 */
	public void setGyroRotation(double currentHeading) {
		// set yawZero to gryo angle, offset to currentHeading
		yawZero = ahrs.getAngle() - currentHeading;
		// System.err.println("PLZ Never Zero the Gyro Rotation it is not good");
	}

	/**
	 * Gets the rotation of the gyro
	 * 
	 * @return Current angle from -180 to 180 degrees
	 */
	public double getGyroRotation() {
		double angle = ahrs.getAngle() - yawZero;
		// Angle will be in terms of raw gyro units (-inf,inf), so you need to convert
		// to (-180, 180]
		angle = angle % 360;
		angle = (angle <= -180) ? (angle + 360) : angle;
    angle = (angle > 180) ? (angle - 360) : angle;
    SmartDashboard.putNumber("Gyro Angle", angle);
		return angle;
  }

  /**
   * 
   * @param setCoast true if want to put driveTrain in coast mode false to put in brake mode.
   */
  public void setDriveMode(boolean setCoast){
   if(setCoast){
    leftMotor1.setNeutralMode(NeutralMode.Coast);
    leftMotor2.setNeutralMode(NeutralMode.Coast);
    leftMotor3.setNeutralMode(NeutralMode.Coast);
    rightMotor1.setNeutralMode(NeutralMode.Coast);
    rightMotor2.setNeutralMode(NeutralMode.Coast);
    rightMotor3.setNeutralMode(NeutralMode.Coast);

   }else{
    leftMotor1.setNeutralMode(NeutralMode.Brake);
    leftMotor2.setNeutralMode(NeutralMode.Brake);
    leftMotor3.setNeutralMode(NeutralMode.Brake);
    rightMotor1.setNeutralMode(NeutralMode.Brake);
    rightMotor2.setNeutralMode(NeutralMode.Brake);
    rightMotor3.setNeutralMode(NeutralMode.Brake);

   }
   
  }

  /**
   * Checks if both encoders are turning. Make sure you have been calling updateEncoderList enough times before.
   * @param precision Precision, in ticks (i.e. number of ticks by which the average can differ from the last reading)
   * @return true if the difference between the average and the last element is less than the precision specified (this means both encoders are stopped)
   */
  public boolean areEncodersStopped(double precision) {
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
  
  public void updateDriveLog () {
    Robot.log.writeLog("DriveTrain", "Update Variables",
      "Drive L1 Volts," + leftMotor1.getMotorOutputVoltage() + ",Drive L2 Volts," + leftMotor2.getMotorOutputVoltage() + ",Drive L3 Volts," + leftMotor3.getMotorOutputVoltage() +
      ",Drive L1 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor1PDP) + ",Drive L2 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor2PDP) + ",Drive L3 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor3PDP) + 
      ",Drive R1 Volts," + rightMotor1.getMotorOutputVoltage() + ",Drive R2 Volts," + rightMotor2.getMotorOutputVoltage() + ",Drive R3 Volts," + rightMotor3.getMotorOutputVoltage() + 
      ",Drive R1 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor1PDP) + ",Drive R2 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor2PDP) + ",Drive R3 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor3PDP) + 
      ",L Enc Ticks," + getLeftEncoderTicks() + ",L Drive Inches," + getLeftEncoderInches() + 
      ",R Enc Ticks," + getRightEncoderTicks() + ",R Drive Inches," + getRightEncoderInches() + 
      ",High Gear," + Robot.shifter.isShifterInHighGear());
  }

  /**
   * Gets the predicted scoring quadrant of the robot based on what the gyro currently reads
   * @return a quadrant (corresponding to the unit circle) with axes in between quadrants numbered as x.5 values.
   */
  public double checkScoringQuadrant() {
    // TODO: Add some console prints or SD to check
    // assuming the same quadrants as a unit circle, with 0 being straight up (+y axis) and -180 or 180 being straight down (-y axis)
    double quadrant = 0.0;
    double gyroAngle = getGyroRotation();

    if (Math.abs(gyroAngle) <= 5) { // Should mean straight up, +y axis, cardianl durection North
      quadrant = 1.5; // in between quadrants 1 and 2
    } else if (Math.abs(gyroAngle) - 180 <= 5) { // Within 5 degrees of -y axis
      quadrant = 3.5; // in between quadrants 3 and 4
    } else if (Math.abs(gyroAngle - 90) <= 5) { // Within 5 degrees of +x axis
      quadrant = 0.5; // in between quadrants 4 and 1
    } else if (Math.abs(gyroAngle + 90) <= 5) { // Wihin 5 degrees of -x axis
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
    
    return quadrant; // Something must be wrong here, this result should never happen
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
   * Drives to the crosshair without gyro adjustment
   */
  public void driveToCrosshair() {
    driveToCrosshair(0);
  }

  /**
   * Drives towards target and stops in front of it using speed from left joystick
   * This may change depending on driver preferences
   */
  public void driveToCrosshair(double quadrant) {

    double xOffsetAdjustmentFactor = 1.7; // Should be tested to be perfect; 2 seems to go out of frame too quickly. Must be greater than 1.

    //double minDistanceToTarget = 13; // Not used right now because changing distance forumla to use height
    double distance = Robot.vision.distanceFromTarget();
    System.out.println("Measured Distance: " + distance);
    double area = Robot.vision.areaFromCamera;
    double xVal = Robot.vision.horizOffset; // Alpha offset
    double finalAngle;

    if (quadrant != 0) {
      System.out.println("Measured Angle: " + xVal);
      double alphaT = xVal + getGyroRotation() - getTargetAngle(quadrant); // true angle for measuring x displacement
      System.out.println("Adjusted (true) angle: " + alphaT);
      double alphaA = Math.toDegrees(Math.atan(xOffsetAdjustmentFactor * Math.tan(Math.toRadians(alphaT)))); // Adjusted angle for x displacement
      System.out.println("False displacement angle:" + alphaA);
      finalAngle = alphaA + getTargetAngle(quadrant) - getGyroRotation();
    } else {
      finalAngle = xVal;
    }

    double gainConstant = 1.0/30.0;

    //double lJoystickAdjust = Math.abs(Robot.oi.leftJoystick.getY());
    double lJoystickAdjust = 0.7 * Math.sqrt(Math.abs(Robot.oi.leftJoystick.getY()));
    double lPercentOutput = lJoystickAdjust + (gainConstant * finalAngle); //xVal
    double rPercentOutput = lJoystickAdjust - (gainConstant * finalAngle); //xVal

    /* Untested auto-turn stuff */
    if (lEncStopped && lPercentOutput != 0) rPercentOutput = 1.0; // The goal here is to slam the right side so that we still line up to the wall
    if (rEncStopped && rPercentOutput != 0) lPercentOutput = 1.0; 
    if (lPercentOutput == 1.0 || rPercentOutput == 1.0) System.out.println("STOP DETECTED, INITIATING EVASIVE MANEUVERS"); // TODO: test this because it doesn't look like it ever works

    if (/* distance > minDistanceToTarget && */ area != 0) tankDrive(lPercentOutput, rPercentOutput); // Just ignore the distance check for now...
    else tankDrive(0, 0);

    Robot.log.writeLogEcho("DriveTrain", "Vision Tracking", "Crosshair Horiz Offset," + xVal + ",Inches from Target," + Robot.vision.distanceFromTarget()
     + ",Target Area," + area + ",Joystick Ouput," + lJoystickAdjust + ",Left Percent," + lPercentOutput + ",Right Percent," + rPercentOutput);
  }

   /**
   * Turns in place to target
   */
  public void turnToCrosshair() {
    double gainConstant = 1.0/30.0;
    double xVal = Robot.vision.xValue.getDouble(0);
    double fixSpeed = 0.4;
    double lPercentOutput = fixSpeed + (gainConstant * xVal);
    double rPercentOutput = fixSpeed - (gainConstant * xVal);
    
    if (xVal > 0.5) {
      this.robotDrive.tankDrive(lPercentOutput, -lPercentOutput);
    } else if (xVal < -0.5) {
      this.robotDrive.tankDrive(-rPercentOutput, rPercentOutput);
    } else {
      this.robotDrive.tankDrive(0, 0);
    }
    updateEncoderList();
    //Robot.log.writeLog("DriveTrain", "Vision Turning", "Degrees from Target," + xVal + ",Inches from Target," + Robot.vision.distanceFromTarget() + ",Target Area," + Robot.vision.areaFromCamera);
  }

  public void driveOnLine() {
    // TODO: Integrate quadrants and gyro to correct based on which side of the line we're on
    // TODO: Explain to Rob why that is needed
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
      lPercentPower = 0.3;
      rPercentPower = 0.3;
    }

    Robot.log.writeLogEcho("DriveTrain", "Line Tracking", "Line Number," + lineNum + ",Left Percent," + lPercentPower + ",Right Percent," + rPercentPower);

    /* Untested auto-turn stuff */
    if (lEncStopped && lPercentPower != 0) rPercentPower = 1.0; // The goal here is to slam the right side so that we still line up to the wall
    if (rEncStopped && rPercentPower != 0) lPercentPower = 1.0;
    if (lPercentPower == 1 || rPercentPower == 1) System.out.println("STOP DETECTED, INITIATING EVASIVE MANEUVERS"); 

    this.robotDrive.tankDrive(lPercentPower, rPercentPower);
    updateEncoderList();
  }

  public void quadrantLineFollowing(double quadrant) {
    // add a special case for straight down (quadrant 3.5) because of roll over on 180
    double angleTolerance = 3.0; // degrees
    boolean left = Math.abs(getGyroRotation()) > Math.abs(getTargetAngle(quadrant) + angleTolerance);
    boolean right = Math.abs(getGyroRotation()) < Math.abs(getTargetAngle(quadrant) - angleTolerance);
    if (!left && !right) {
      driveOnLine(); // if we're close to center just do the simple line following
      return;
    }

    double baseSpeed = 0.7; // Lowered from 1 for new line sensors further out, untested
    int lineNum = Robot.lineFollowing.getLineNumber();
    double lPercentPower = 0.0, rPercentPower = 0.0;

    if (left) {
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
        lPercentPower = 0.3;
        rPercentPower = 0.3;
      }
    }
    
  }
  
  public void quadrantLineFollowing() {
    quadrantLineFollowing(checkScoringQuadrant());
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoysticks());
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("RightEnc", getRightEncoderTicks());
    SmartDashboard.putNumber("LeftEnc", getLeftEncoderTicks());

    if (DriverStation.getInstance().isEnabled()) {
      if ((++periodicCount) >= 10) {
        updateDriveLog();
        verifyMotors(RobotMap.leftMotor1PDP, RobotMap.leftMotor2PDP, RobotMap.leftMotor3PDP, true);
        verifyMotors(RobotMap.rightMotor1PDP, RobotMap.rightMotor2PDP, RobotMap.rightMotor3PDP, false);
        Robot.lineFollowing.logLineFollowers(); // This is the best place for this I guess -- only updates about every 0.5 second
        periodicCount=0;  
      }
    }
  }
}
