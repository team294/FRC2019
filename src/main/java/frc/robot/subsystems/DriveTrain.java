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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoysticks;
import edu.wpi.first.wpilibj.command.Subsystem;

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

  // Iterators for logging
  private int periodicCount = 0;
  
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
  }

  public void tankDrive (double powerLeft, double powerRight) {
    robotDrive.tankDrive(powerLeft, powerRight);
  }

  /**
   * Stops the motors by calling tankDrive(0, 0)
   */
  public void stop() {
    tankDrive(0, 0);
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
    return leftMotor2.getSelectedSensorPosition(0) + leftEncoderZero;
  }

	/**
	 * Get the position of the right encoder, in encoder ticks since last zeroRightEncoder()
	 * 
	 * @return encoder position, in ticks
	 */
	public double getRightEncoderTicks() {
		return rightMotor2.getSelectedSensorPosition(0) + rightEncoderZero;
	}

  public double encoderTicksToInches(double encoderTicks) {
    return (encoderTicks / RobotMap.encoderTicksPerRevolution) * Robot.robotPrefs.wheelCircumference ;
  }
  public double inchesToEncoderTicks(double inches) {
    return (inches / Robot.robotPrefs.wheelCircumference) * RobotMap.encoderTicksPerRevolution;
  }
  public double getLeftEncoderInches() {
    return encoderTicksToInches(getLeftEncoderTicks());
  }

  public double getRightEncoderInches() {
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
    if (lEncoderStack.size() > 50){
      lEncoderStack.remove();
      rEncoderStack.remove();
    }
  }

  /**
   * Checks if both encoders are turning. Make sure you have been calling updateEncoderList enough times before.
   * @param precision Precision, in ticks (i.e. number of ticks by which the average can differ from the last reading)
   * @return true if the difference between the average and the last element is less than the precision specified
   */
  public boolean areEncodersTurning(double precision) {
    if (lEncoderStack.size()<50) return (lEncStopped = false) || (rEncStopped = false); // Sets both to false while returning.
    double lSum = 0.0, rSum = 0.0;
    Iterator<Double> lIterator = lEncoderStack.descendingIterator();
    Iterator<Double> rIterator = rEncoderStack.descendingIterator();
    while(lIterator.hasNext()) {
      lSum += lIterator.next();
      rSum += rIterator.next();
    }
    return (lEncStopped = (Math.abs(lSum/lEncoderStack.size()-lEncoderStack.peekLast()) <= precision) & (rEncStopped = Math.abs(rSum/rEncoderStack.size()-rEncoderStack.peekLast()) <= precision));
  }

  public void updateDriveLog () {
    Robot.log.writeLog("DriveTrain", "Update Variables",
      "Drive L1 Volts," + leftMotor1.getMotorOutputVoltage() + ",Drive L2 Volts," + leftMotor2.getMotorOutputVoltage() + ",Drive L3 Volts," + leftMotor3.getMotorOutputVoltage() +
      ",Drive L1 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor1PDP) + ",Drive L2 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor2PDP) + ",Drive L3 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor3PDP) + 
      ",Drive R1 Volts," + rightMotor1.getMotorOutputVoltage() + ",Drive R2 Volts," + rightMotor2.getMotorOutputVoltage() + ",Drive R3 Volts," + rightMotor3.getMotorOutputVoltage() + 
      ",Drive R1 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor1PDP) + ",Drive R2 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor2PDP) + ",Drive R3 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor3PDP) + 
      ",L Enc Zero," + leftEncoderZero + ",L Enc Ticks," + getLeftEncoderTicks() + ",L Drive Inches," + getLeftEncoderInches() + 
      ",R Enc Zero," + rightEncoderZero + ",R Enc Ticks," + getRightEncoderTicks() + ",R Drive Inches," + getRightEncoderInches() + 
      ",High Gear," + Robot.shifter.isShifterInHighGear());
  }

  /**
   * Drives towards target and stops in front of it using speed from left joystick
   * This may change depending on driver preferences
   */
  public void driveToCrosshair() {

    double minDistanceToTarget = 13;
    double distance = Robot.vision.distanceFromTarget(); // Convert to a pure area measurement since dist is inaccurate
    double area = Robot.vision.areaFromCamera;
    double xVal = Robot.vision.horizOffset;

    double gainConstant = 1.0/30.0;
    //double startSpeed = -0.5;

    /*
    double lJoystickPercent = Robot.oi.leftJoystick.getY()-0.25; // Test value for now, wanted to speed it up
    // double lJoystickPercent = Math.abs(Robot.oi.leftJoystick.getY()) + 0.25
    double lPercentOutput = startSpeed + (gainConstant * xVal);
    double rPercentOutput = startSpeed - (gainConstant * xVal);

    lPercentOutput -= lJoystickPercent;
    rPercentOutput -= lJoystickPercent;
    */

    double lJoystickAdjust = Math.abs(Robot.oi.leftJoystick.getY());
    double lPercentOutput = lJoystickAdjust + (gainConstant * xVal);
    double rPercentOutput = lJoystickAdjust - (gainConstant * xVal);

    /* Untested auto-turn stuff */
    if (lEncStopped && lPercentOutput != 0) rPercentOutput = 1.0; // The goal here is to slam the right side so that we still line up to the wall
    if (rEncStopped && rPercentOutput != 0) lPercentOutput = 1.0; 
    if (lPercentOutput == 1 || rPercentOutput == 1) System.out.println("STOP DETECTED, INITIATING EVASIVE MANEUVERS");

    if (distance > minDistanceToTarget && area != 0) tankDrive(lPercentOutput, rPercentOutput);
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
    double lPercentPower = 0;
    double rPercentPower = 0;
    double baseSpeed = 1; // If this remains 1, we can remove it from the code. Otherwise, we cdan make it responsive to the joystick as well.

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

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoysticks());
  }

  @Override
  public void periodic() {

    if (DriverStation.getInstance().isEnabled()) {
      if ((++periodicCount) >= 25) {
        updateDriveLog();
        Robot.lineFollowing.logLineFollowers(); // This is the best place for this I guess
        periodicCount=0;  
      }
    }
  }
}


