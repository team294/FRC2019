/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * Drive Train subsytem using Spark motors.
 */
public class NeoDriveTrain extends DriveTrain {

  private CANSparkMax leftMotor1 = new CANSparkMax(RobotMap.leftMotor1, MotorType.kBrushless);
  private CANSparkMax leftMotor2 = new CANSparkMax(RobotMap.leftMotor2, MotorType.kBrushless);
  private CANSparkMax leftMotor3 = new CANSparkMax(RobotMap.leftMotor3, MotorType.kBrushless);
  private CANSparkMax rightMotor1 = new CANSparkMax(RobotMap.rightMotor1, MotorType.kBrushless);
  private CANSparkMax rightMotor2 = new CANSparkMax(RobotMap.rightMotor2, MotorType.kBrushless);
  private CANSparkMax rightMotor3 = new CANSparkMax(RobotMap.rightMotor3, MotorType.kBrushless);

  private SensorCollection lShaftEncoder = new WPI_TalonSRX(RobotMap.elevatorMotor2).getSensorCollection();
  private SensorCollection rShaftEncoder = new WPI_TalonSRX(RobotMap.climbMotor1).getSensorCollection();

  public final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor2, rightMotor2);

  public NeoDriveTrain() {
    super();

    leftMotor1.setIdleMode(IdleMode.kCoast);
    leftMotor2.setIdleMode(IdleMode.kBrake);  // was brake
    leftMotor3.setIdleMode(IdleMode.kCoast);
    rightMotor1.setIdleMode(IdleMode.kCoast);
    rightMotor2.setIdleMode(IdleMode.kBrake);   // was Brake
    rightMotor3.setIdleMode(IdleMode.kCoast);

    leftMotor1.setInverted(true);
    leftMotor2.setInverted(true);
    leftMotor3.setInverted(true);
    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);
    rightMotor3.setInverted(true);
    
    leftMotor1.follow(leftMotor2);
    leftMotor3.follow(leftMotor2);
    rightMotor1.follow(rightMotor2);
    rightMotor3.follow(rightMotor2);

    leftMotor1.clearFaults();
    leftMotor2.clearFaults();
    leftMotor3.clearFaults();
    rightMotor1.clearFaults();
    rightMotor2.clearFaults();
    rightMotor3.clearFaults();

    robotDrive.setDeadband(0.05);
  }

  @Override
	public void driveAtCurve(double speedPct, double curve) {
		robotDrive.curvatureDrive(speedPct, curve, false);
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
  public void setLeftMotors(double percent) {
    leftMotor2.set(-percent);
  }

  @Override
  public void setRightMotors(double percent) {
    rightMotor2.set(percent);
  }

  // TODO: Check encoder directions (whether increasing while driving forward or what)

  @Override
  public double getLeftEncoderRaw() {
    return -lShaftEncoder.getQuadraturePosition();
  }

  @Override
  public double getRightEncoderRaw() {
    return -rShaftEncoder.getQuadraturePosition();
  }

  @Override
  public double getLeftEncoderVelocityRaw() {
    return -lShaftEncoder.getQuadratureVelocity();
  }

  @Override
  public double getRightEncoderVelocityRaw() {
    return rShaftEncoder.getQuadratureVelocity();
  }
  
  @Override
	public void setVoltageCompensation(boolean turnOn) {
		if (turnOn) {
      leftMotor1.enableVoltageCompensation(12.0);
		  leftMotor2.enableVoltageCompensation(12.0);
		  leftMotor3.enableVoltageCompensation(12.0);
		  rightMotor1.enableVoltageCompensation(12.0);
		  rightMotor2.enableVoltageCompensation(12.0);
      rightMotor3.enableVoltageCompensation(12.0);
    } else {
      leftMotor1.disableVoltageCompensation();
      leftMotor2.disableVoltageCompensation();
      leftMotor3.disableVoltageCompensation();
      rightMotor1.disableVoltageCompensation();
      rightMotor2.disableVoltageCompensation();
      rightMotor3.disableVoltageCompensation();
    }
  }

  @Override
  public void setDriveMode(boolean setCoast) {
    if (setCoast) {
     leftMotor1.setIdleMode(IdleMode.kCoast);
     leftMotor2.setIdleMode(IdleMode.kCoast);
     leftMotor3.setIdleMode(IdleMode.kCoast);
     rightMotor1.setIdleMode(IdleMode.kCoast);
     rightMotor2.setIdleMode(IdleMode.kCoast);
     rightMotor3.setIdleMode(IdleMode.kCoast);
    } else {
     leftMotor1.setIdleMode(IdleMode.kCoast);
     leftMotor2.setIdleMode(IdleMode.kBrake);
     leftMotor3.setIdleMode(IdleMode.kCoast);
     rightMotor1.setIdleMode(IdleMode.kCoast);
     rightMotor2.setIdleMode(IdleMode.kBrake);
     rightMotor3.setIdleMode(IdleMode.kCoast);
    }
    
  }

  @Override
  public void updateDriveLog (boolean logWhenDisabled) {
    Robot.log.writeLog(logWhenDisabled, "DriveTrain", "Update Variables",
      "Drive L1 Volts," + leftMotor1.getAppliedOutput() + ",Drive L2 Volts," + leftMotor2.get() + ",Drive L3 Volts," + leftMotor3.getAppliedOutput() +
      ",Drive L1 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor1PDP) + ",Drive L2 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor2PDP) + ",Drive L3 Amps," + Robot.pdp.getCurrent(RobotMap.leftMotor3PDP) + 
      ",Drive R1 Volts," + rightMotor1.getAppliedOutput() + ",Drive R2 Volts," + rightMotor2.get() + ",Drive R3 Volts," + rightMotor3.getAppliedOutput() + 
      ",Drive R1 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor1PDP) + ",Drive R2 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor2PDP) + ",Drive R3 Amps," + Robot.pdp.getCurrent(RobotMap.rightMotor3PDP) + 
      ",L Enc Ticks," + getLeftEncoderTicks() + ",L Drive Inches," + getLeftEncoderInches() + ",L Vel," + getLeftEncoderVelocity() +
      ",R Enc Ticks," + getRightEncoderTicks() + ",R Drive Inches," + getRightEncoderInches() +  ",R Vel," + getRightEncoderVelocity() +
      ",High Gear," + Robot.shifter.isShifterInHighGear());
  }

}
