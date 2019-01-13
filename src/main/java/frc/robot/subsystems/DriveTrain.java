/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Drive Train subsystem.  
 */
public class DriveTrain extends Subsystem {
  // This is to test with the 2018 drive base.  The 2019 drive base will use 4 Talon SBX controllers for follower motors 1 and 3
  private final WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(RobotMap.leftMotor1);
  private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(RobotMap.leftMotor2);
  private final WPI_TalonSRX leftMotor3 = new WPI_TalonSRX(RobotMap.leftMotor3);
  private final WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(RobotMap.rightMotor1);
  private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RobotMap.rightMotor2);
  private final WPI_TalonSRX rightMotor3 = new WPI_TalonSRX(RobotMap.rightMotor3);
  public final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor2, rightMotor2);

  private int periodicCount = 0;
  
  private double leftEncoderZero = 0, rightEncoderZero = 0;

  public DriveTrain() {
    
    leftMotor1.set(ControlMode.Follower, RobotMap.leftMotor2);
    leftMotor3.set(ControlMode.Follower, RobotMap.leftMotor2);
    rightMotor1.set(ControlMode.Follower, RobotMap.rightMotor2);
    rightMotor3.set(ControlMode.Follower, RobotMap.rightMotor2);

    leftMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		rightMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		zeroLeftEncoder();
		zeroRightEncoder();

    leftMotor2.clearStickyFaults(0);
    rightMotor2.clearStickyFaults(0);

    leftMotor2.setNeutralMode(NeutralMode.Brake);
    rightMotor2.setNeutralMode(NeutralMode.Brake);

    
  }

  public void tankDrive (double powerLeft, double powerRight) {
    this.robotDrive.tankDrive(powerLeft, powerRight);
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
		return rightMotor2.getSelectedSensorPosition(0) - rightEncoderZero;
	}

  /* Method to convert encoder Ticks to Inches and vice versa, to be uncommented when RobotMap and robotPrefs are updated
  public double encoderTicksToInches(double encoderTicks) {
    return (encoderTicks / RobotMap.encoderTicksPerRevolution) * Robot.robotPrefs.wheelCircumference * Robot.robotPrefs.driveTrainDistanceFudgeFactor;
  } 
  public double inchesToEncoderTicks(double inches) {
    return (inches / Robot.robotPrefs.wheelCircumference / Robot.robotPrefs.driveTrainDistanceFudgeFactor) * RobotMap.encoderTicksPerRevolution;
  }
  */

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new DriveWithJoysticks());
  }
  @Override
  public void periodic() {

    if (DriverStation.getInstance().isEnabled()) {
      if ((++periodicCount) >= 50) {
        updateDriveLog();
        periodicCount=0;
        System.out.println("ENCODER TEST: " + getLeftEncoderTicks() );
      }
    }
  }
  
  public void updateDriveLog () {
    Robot.log.writeLog("DriveTrain", "Update Variables",
      "Left Motor 1 Output Voltage," + leftMotor1.getMotorOutputVoltage() + ",Left Motor 1 Output Current," + leftMotor1.getOutputCurrent() + ",Left Motor 1 Output Percent," + leftMotor1.getMotorOutputPercent() +
      ",Left Motor 2 Output Voltage," + leftMotor2.getMotorOutputVoltage() + ",Left Motor 2 Output Current," + leftMotor2.getOutputCurrent() + ",Left Motor 2 Output Percent," + leftMotor2.getMotorOutputPercent() +
      ",Left Motor 3 Output Voltage," + leftMotor3.getMotorOutputVoltage() + ",Left Motor 3 Output Current," + leftMotor3.getOutputCurrent() + ",Left Motor 3 Output Percent," + leftMotor3.getMotorOutputPercent() +
      ",Right Motor 1 Output Voltage," + rightMotor1.getMotorOutputVoltage() + ",Right Motor 1 Output Current," + rightMotor1.getOutputCurrent() + ",Right Motor 1 Output Percent," + rightMotor1.getMotorOutputPercent() +
      ",Right Motor 1 Output Voltage," + rightMotor2.getMotorOutputVoltage() + ",Right Motor 2 Output Current," + rightMotor2.getOutputCurrent() + ",Right Motor 2 Output Percent," + rightMotor2.getMotorOutputPercent() +
      ",Right Motor 1 Output Voltage," + rightMotor3.getMotorOutputVoltage() + ",Right Motor 3 Output Current," + rightMotor3.getOutputCurrent() + ",Right Motor 3 Output Percent," + rightMotor3.getMotorOutputPercent() +
      ",Left Encoder Ticks," + getLeftEncoderTicks() + ",Right Encoder Ticks," + getRightEncoderTicks() +
      ",High Gear," + Robot.shifter.isShifterInHighGear());
  }
}
