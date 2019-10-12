/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;   // had to change from just frc.robot.subsystems ?

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Drive Train subsystem using CIM motors. 
 */
public class CimDriveTrain extends DriveTrain {

  private final BaseMotorController leftMotor1;
  private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(RobotMap.leftMotor2);
  private final BaseMotorController leftMotor3;
  private final BaseMotorController rightMotor1;
  private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RobotMap.rightMotor2);
  private final BaseMotorController rightMotor3;

  public final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor2, rightMotor2);


  public CimDriveTrain() {
    super();

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
    } else {
      leftMotor1 = new WPI_VictorSPX(RobotMap.leftMotor1);
      leftMotor3 = new WPI_VictorSPX(RobotMap.leftMotor3);
      rightMotor1 = new WPI_VictorSPX(RobotMap.rightMotor1);
      rightMotor3 = new WPI_VictorSPX(RobotMap.rightMotor3);

      leftMotor1.follow(leftMotor2);
      leftMotor3.follow(leftMotor2);
      rightMotor1.follow(rightMotor2);
      rightMotor3.follow(rightMotor2);
    }

    leftMotor1.setInverted(true);
    leftMotor2.setInverted(true);
    leftMotor3.setInverted(true);
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);
    rightMotor3.setInverted(true);

    leftMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    rightMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    leftMotor1.follow(leftMotor2);
    leftMotor3.follow(leftMotor2);
    rightMotor1.follow(rightMotor2);
    rightMotor3.follow(rightMotor2);

    leftMotor1.clearStickyFaults(0);
    leftMotor2.clearStickyFaults(0);
    leftMotor3.clearStickyFaults(0);
    rightMotor1.clearStickyFaults(0);
    rightMotor2.clearStickyFaults(0);
    rightMotor3.clearStickyFaults(0);

    setDriveModeCoast(false);      // Set to brake drive mode

    robotDrive.setDeadband(0.12);  // Was 0.05

    leftMotor1.configVoltageCompSaturation(12.0);
    leftMotor2.configVoltageCompSaturation(12.0);
    leftMotor3.configVoltageCompSaturation(12.0);
    rightMotor1.configVoltageCompSaturation(12.0);
    rightMotor2.configVoltageCompSaturation(12.0);
    rightMotor3.configVoltageCompSaturation(12.0);
  }
  
  @Override
  public void tankDrive(double powerLeft, double powerRight) {
    robotDrive.tankDrive(powerLeft, powerRight);
  }

  @Override
  public void tankDrive(double powerLeft, double powerRight, boolean squaredInputs) {
    robotDrive.tankDrive(powerLeft, powerRight, squaredInputs);
  }

  @Override
	public void driveAtCurve(double speedPct, double curve) {
		robotDrive.curvatureDrive(speedPct, curve, false);
  }

  @Override
  public void setLeftMotors(double percent) {
    leftMotor2.set(ControlMode.PercentOutput, -percent);
  }

  @Override
  public void setRightMotors(double percent) {
    rightMotor2.set(ControlMode.PercentOutput, percent);
  }
  
  /**
   * Drives the robot along a curve
   * @param speedPct Percent output of motor [-1.0, 1.0]
   * @param rotation Rotation rate (rate of heading change) from [-1.0, 1.0]
   */
  public void driveItLikeYouStoleIt(double speedPct, double rotation) {
    robotDrive.arcadeDrive(speedPct, rotation, false);
  }

  @Override
	public void setVoltageCompensation(boolean turnOn) {
    leftMotor1.enableVoltageCompensation(turnOn);
    leftMotor2.enableVoltageCompensation(turnOn);
    leftMotor3.enableVoltageCompensation(turnOn);
    rightMotor1.enableVoltageCompensation(turnOn);
    rightMotor2.enableVoltageCompensation(turnOn);
    rightMotor3.enableVoltageCompensation(turnOn);
	}

  @Override
  public double getRightEncoderRaw() {
    return rightMotor2.getSelectedSensorPosition(0);
  }

  @Override
  public double getLeftEncoderRaw() {
    return leftMotor2.getSelectedSensorPosition(0);
  }

  @Override
  public double getLeftEncoderVelocityRaw() {
    return leftMotor2.getSelectedSensorVelocity(0);
  }

  @Override
  public double getRightEncoderVelocityRaw() {
    return -rightMotor2.getSelectedSensorVelocity(0);
  }

  @Override
  public void setDriveModeCoast(boolean setCoast) {
    if (setCoast) {
      leftMotor1.setNeutralMode(NeutralMode.Coast);
      leftMotor2.setNeutralMode(NeutralMode.Coast);
      leftMotor3.setNeutralMode(NeutralMode.Coast);
      rightMotor1.setNeutralMode(NeutralMode.Coast);
      rightMotor2.setNeutralMode(NeutralMode.Coast);
      rightMotor3.setNeutralMode(NeutralMode.Coast);
    } else {
      leftMotor1.setNeutralMode(NeutralMode.Brake);
      leftMotor2.setNeutralMode(NeutralMode.Brake);
      leftMotor3.setNeutralMode(NeutralMode.Brake);
      rightMotor1.setNeutralMode(NeutralMode.Brake);
      rightMotor2.setNeutralMode(NeutralMode.Brake);
      rightMotor3.setNeutralMode(NeutralMode.Brake);
    }
  }

  @Override
  public void updateDriveLog (boolean logWhenDisabled) {
    Robot.log.writeLog(logWhenDisabled, "DriveTrain", "Update Variables",
      "Drive L1 Volts," + leftMotor1.getBusVoltage() + ",Drive L2 Volts," + leftMotor2.getBusVoltage() + ",Drive L3 Volts," + leftMotor3.getBusVoltage() +
      ",Drive L1 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor1PDP) + ",Drive L2 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor2PDP) + ",Drive L3 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor3PDP) + 
      ",Drive R1 Volts," + rightMotor1.getBusVoltage() + ",Drive R2 Volts," + rightMotor2.getBusVoltage() + ",Drive R3 Volts," + rightMotor3.getBusVoltage() + 
      ",Drive R1 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor1PDP) + ",Drive R2 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor2PDP) + ",Drive R3 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor3PDP) + 
      ",L Enc Ticks," + getLeftEncoderTicks() + ",L Drive Inches," + getLeftEncoderInches() + ",L Vel," + getLeftEncoderVelocity() +
      ",R Enc Ticks," + getRightEncoderTicks() + ",R Drive Inches," + getRightEncoderInches() +  ",R Vel," + getRightEncoderVelocity() +
      ",High Gear," + Robot.shifter.isShifterInHighGear());
  }
}
